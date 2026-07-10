#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Omokai take-home, Part 4 (Vision AI target detection + follow).
#
# Sources / citations:
#   - OpenCV ArUco + solvePnP (Apache-2.0): marker detection + 6-DoF pose.
#   - Ultralytics YOLO (AGPL-3.0): optional 'yolo' detection mode.
#   - Nav2 (nav2_msgs NavigateToPose): motion planner in 'nav2' control mode.
#
# ---------------------------------------------------------------------------
# CONTROL MODES
#   'direct' : detection -> P-controller -> cmd_vel. No map, no obstacle
#              avoidance. Drives straight at the target, into walls.
#   'nav2'   : detection -> PnP pose -> base_link -> map -> standoff goal ->
#              NavigateToPose. Nav2 plans around obstacles. This node never
#              publishes cmd_vel here; Nav2's controller owns the wheels.
#
# STATE MACHINE (nav2 mode)
#   SEARCHING  : never seen the target.
#   FOLLOWING  : target fresh & far -> chase a standoff goal behind it.
#   HOLDING    : target fresh & close -> stop (hysteresis prevents chatter).
#   RECOVERING : target lost. Drive to where it WAS, oriented along the
#                heading it was travelling, so the camera looks the way it
#                went. This is the fix for "target turns a corner".
#   LOST       : recovery expired. Stop.
#
# WHY RECOVERY NEEDS THE TARGET'S YAW, NOT JUST ITS POSITION
#   When the target vanishes, the follower has usually already arrived at the
#   standoff goal behind the target's last position -- so re-issuing that
#   position as a goal moves the robot nowhere. (That is precisely why the
#   previous version appeared to "just stand there": it had reached its goal,
#   the stale distance was below stop_distance, and the hysteresis latch put
#   it in HOLDING forever.) What it lacks is a *heading*: which way the target
#   went. The marker's orientation at the last good frame gives exactly that.
#   Inferring heading from the target's motion history fails for the case that
#   matters -- a sharp turn -- because the history still points the old way.
#
# CLOCK NOTE
#   All timing uses the ROS clock, not time.time(). Under use_sim_time the sim
#   may run slower than wall-clock (badly so under Docker's software
#   rendering), and a wall-clock timeout would expire after far less simulated
#   time than intended.
# ---------------------------------------------------------------------------

import json
import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a quaternion. Written out rather than using
    tf2_geometry_msgs, whose do_transform_pose signature changed across
    ROS distros."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def optical_to_body(v):
    """OpenCV optical (x right, y down, z fwd) -> ROS body (x fwd, y left, z up)."""
    return np.array([v[2], -v[0], -v[1]])


class VisionFollowNode(Node):

    def __init__(self):
        super().__init__('vision_follow_node')

        # ---- Detection --------------------------------------------------
        self.declare_parameter('detection_mode', 'aruco')
        self.declare_parameter('control_mode', 'nav2')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_marker_id', 0)
        self.declare_parameter('marker_length', 0.08)
        self.declare_parameter('infer_every_n_frames', 2)
        self.declare_parameter('camera_offset_x', 0.03)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.08)
        # Below this face-on cosine the marker is too oblique for its normal
        # (and therefore the target's heading) to be trusted. Position stays
        # usable; only the yaw update is skipped.
        self.declare_parameter('min_face_on_cosine', 0.35)

        # ---- 'direct' mode ----------------------------------------------
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('desired_distance', 0.6)
        self.declare_parameter('lost_target_grace_sec', 0.4)
        self.declare_parameter('recovery_rotation_speed', 0.3)

        # ---- 'nav2' mode: following geometry ----------------------------
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        # How far behind the target to park. Lower = tighter follow, but the
        # target robot is an OBSTACLE in the follower's costmap: the goal must
        # sit outside its inflated footprint or Nav2 will refuse it. With the
        # tightened follow params (inflation_radius 0.25, robot_radius 0.1)
        # ~0.45 m is about the floor. See params/burger_cam_follow_nav2_params.yaml.
        self.declare_parameter('standoff_distance', 0.5)
        self.declare_parameter('stop_distance', 0.6)
        self.declare_parameter('resume_distance', 0.85)
        self.declare_parameter('goal_update_period', 0.75)
        self.declare_parameter('goal_update_min_dist', 0.25)
        self.declare_parameter('tf_timeout_sec', 0.2)

        # ---- 'nav2' mode: lost-target recovery --------------------------
        # Treat the target as lost after this long with no detection.
        self.declare_parameter('target_lost_grace_sec', 5.0)
        # Then drive to its last known spot, facing where it went, and keep
        # looking for this long before giving up.
        self.declare_parameter('recovery_timeout_sec', 20.0)
        # Stop this far short of the target's exact last position, so that if
        # the target is in fact still sitting there we don't try to occupy it.
        # (Nav2's costmap is the real guard; this is belt-and-braces.)
        self.declare_parameter('recovery_goal_backoff', 0.35)

        self.declare_parameter('snapshot_dir', '/tmp/vision_follow_snapshots')
        self.declare_parameter('alert_cooldown_sec', 15.0)

        g = self.get_parameter
        self.detection_mode = g('detection_mode').value
        self.control_mode = g('control_mode').value
        self.target_class = g('target_class').value
        self.confidence_threshold = float(g('confidence_threshold').value)
        self.model_path = g('model_path').value
        self.device = g('device').value
        self.aruco_dictionary_name = g('aruco_dictionary').value
        self.aruco_marker_id = int(g('aruco_marker_id').value)
        self.marker_length = float(g('marker_length').value)
        self.infer_every_n_frames = int(g('infer_every_n_frames').value)
        self.cam_off = np.array([float(g('camera_offset_x').value),
                                 float(g('camera_offset_y').value),
                                 float(g('camera_offset_z').value)])
        self.min_face_on_cosine = float(g('min_face_on_cosine').value)

        self.max_linear_speed = float(g('max_linear_speed').value)
        self.max_angular_speed = float(g('max_angular_speed').value)
        self.kp_angular = float(g('kp_angular').value)
        self.kp_linear = float(g('kp_linear').value)
        self.desired_distance = float(g('desired_distance').value)
        self.lost_target_grace_sec = float(g('lost_target_grace_sec').value)
        self.recovery_rotation_speed = float(g('recovery_rotation_speed').value)

        self.global_frame = g('global_frame').value
        self.robot_base_frame = g('robot_base_frame').value
        self.standoff_distance = float(g('standoff_distance').value)
        self.stop_distance = float(g('stop_distance').value)
        self.resume_distance = float(g('resume_distance').value)
        self.goal_update_period = float(g('goal_update_period').value)
        self.goal_update_min_dist = float(g('goal_update_min_dist').value)
        self.tf_timeout_sec = float(g('tf_timeout_sec').value)

        self.target_lost_grace_sec = float(g('target_lost_grace_sec').value)
        self.recovery_timeout_sec = float(g('recovery_timeout_sec').value)
        self.recovery_goal_backoff = float(g('recovery_goal_backoff').value)

        self.snapshot_dir = g('snapshot_dir').value
        self.alert_cooldown_sec = float(g('alert_cooldown_sec').value)

        if self.control_mode not in ('direct', 'nav2'):
            raise ValueError(f"control_mode must be 'direct' or 'nav2', got '{self.control_mode}'")
        if self.control_mode == 'nav2':
            if self.stop_distance >= self.resume_distance:
                raise ValueError('resume_distance must exceed stop_distance (hysteresis band)')
            if self.standoff_distance > self.stop_distance:
                raise ValueError('standoff_distance must be <= stop_distance, else the follower '
                                 'reaches its goal while still "too far" and oscillates')

        # ---- State -------------------------------------------------------
        self.bridge = CvBridge()
        self.frame_count = 0
        self.target_locked = False
        self.last_alert_time = -1e9
        self.last_cmd = (0.0, 0.0)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_pose = None        # target in camera frame (direct mode)
        self.last_seen_time = None
        self.model = None
        self.class_names = {}

        # nav2-mode state
        self.state = 'SEARCHING'
        self.target_map = None       # np.array([x, y]) in global_frame
        self.target_yaw = None       # target's heading in global_frame (rad)
        self.target_map_time = None
        self.last_goal_xy = None
        self.following = True
        self.recovery_goal_sent = False
        self._goal_handle = None
        self._goal_in_flight = False

        if self.detection_mode == 'yolo':
            self._load_yolo_model()
        elif self.detection_mode == 'aruco':
            self._init_aruco()
        else:
            raise ValueError(f"Unknown detection_mode='{self.detection_mode}'")

        # ---- Pub / Sub ----------------------------------------------------
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, 'vision/set_target_class', self.set_target_callback, 10)

        self.detection_image_pub = self.create_publisher(Image, 'vision/detection_image', 10)
        self.alert_pub = self.create_publisher(String, 'vision/detection_alert', 10)
        self.locked_pub = self.create_publisher(Bool, 'vision/target_locked', 10)

        if self.control_mode == 'direct':
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
            self.goal_pub = None
            self.nav_client = None
            self.tf_buffer = None
        else:
            self.cmd_vel_pub = None   # Nav2 owns cmd_vel; never fight it.
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            ns = self.get_namespace().rstrip('/')
            action_name = f'{ns}/navigate_to_pose' if ns else '/navigate_to_pose'
            self.nav_client = ActionClient(self, NavigateToPose, action_name)
            self.goal_pub = self.create_publisher(PoseStamped, 'vision/follow_goal', 10)
            self.create_timer(self.goal_update_period, self.goal_timer_callback)
            self.get_logger().info(f"nav2 mode: NavigateToPose -> '{action_name}'")

        desc = (self.target_class if self.detection_mode == 'yolo'
                else f'aruco id={self.aruco_marker_id}')
        self.get_logger().info(
            f'vision_follow_node ready. detection={self.detection_mode} '
            f'control={self.control_mode} target={desc} '
            f'standoff={self.standoff_distance} stop={self.stop_distance}')

    # ------------------------------------------------------------------
    def _now(self):
        """ROS time in seconds. Honours use_sim_time."""
        return self.get_clock().now().nanoseconds * 1e-9

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.get_logger().info(
                f'Camera intrinsics latched (fx={self.camera_matrix[0, 0]:.1f}). PnP active.')

    # ------------------------------------------------------------------
    def _load_yolo_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                'ultralytics missing: pip install "numpy<2" ultralytics --break-system-packages')
            raise exc
        self.model = YOLO(self.model_path)
        names = self.model.names
        self.class_names = names if isinstance(names, dict) else dict(enumerate(names))

    def _init_aruco(self):
        dict_id = getattr(cv2.aruco, self.aruco_dictionary_name)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._aruco_new_api = True
            self._aruco_detector = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(dict_id), cv2.aruco.DetectorParameters())
        else:
            self._aruco_new_api = False
            self._aruco_dictionary = cv2.aruco.Dictionary_get(dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        h = self.marker_length / 2.0
        self._marker_obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                                        dtype=np.float32)

    # ------------------------------------------------------------------
    def set_target_callback(self, msg: String):
        v = msg.data.strip()
        if not v:
            return
        if self.detection_mode == 'yolo':
            if v != self.target_class:
                self.get_logger().info(f"target_class: '{self.target_class}' -> '{v}'")
                self.target_class = v
        else:
            try:
                new_id = int(v)
            except ValueError:
                self.get_logger().error(f"aruco mode expects an integer marker id, got '{v}'")
                return
            if new_id != self.aruco_marker_id:
                self.get_logger().info(f'marker id: {self.aruco_marker_id} -> {new_id}')
                self.aruco_marker_id = new_id
        self.target_locked = False
        self.last_pose = None
        self.target_map = None
        self.target_yaw = None
        self.target_map_time = None
        self.last_goal_xy = None
        self.recovery_goal_sent = False
        self.state = 'SEARCHING'
        self._cancel_goal()

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % max(self.infer_every_n_frames, 1) != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        h, w = frame.shape[:2]
        overlay = frame.copy()

        if self.detection_mode == 'aruco':
            det, seen = self._detect_aruco(frame, overlay)
        else:
            det, seen = self._detect_yolo(frame, overlay)

        self._draw_hud(overlay, w, h, seen)
        try:
            out = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            out.header = msg.header
            self.detection_image_pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f'overlay publish failed: {exc}')

        now = self._now()
        if det is not None:
            # BUGFIX: capture this BEFORE flipping target_locked, or the
            # "newly acquired" alert can never fire.
            newly_acquired = not self.target_locked

            self.target_locked = True
            self.last_seen_time = now
            if det.get('pose') is not None:
                self.last_pose = det['pose']
                if self.control_mode == 'nav2':
                    self._update_target_in_map(det, msg.header.stamp)

            if newly_acquired or (now - self.last_alert_time) >= self.alert_cooldown_sec:
                self._send_alert(overlay, det['label'], det['conf'], det.get('pose'), now)
                self.last_alert_time = now

            self.locked_pub.publish(Bool(data=True))
            if self.control_mode == 'direct':
                self._direct_control(det, w, h)
        else:
            if self.control_mode == 'direct':
                self._direct_no_detection(now)
            elif self.target_locked and self.last_seen_time is not None and \
                    (now - self.last_seen_time) > self.target_lost_grace_sec:
                self.target_locked = False
                self.locked_pub.publish(Bool(data=False))

    # ------------------------------------------------------------------
    # nav2: camera-frame detection -> map-frame target position + heading
    # ------------------------------------------------------------------
    def _update_target_in_map(self, det, stamp):
        p_base = optical_to_body(det['pose']) + self.cam_off

        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, stamp,
                timeout=Duration(seconds=self.tf_timeout_sec))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_base_frame, rclpy.time.Time())
            except Exception as exc:
                self.get_logger().warn(
                    f'No TF {self.global_frame}<-{self.robot_base_frame}: {exc}. '
                    'Is Nav2/AMCL up and localized?', throttle_duration_sec=5.0)
                return

        q, t = tf.transform.rotation, tf.transform.translation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_map = R @ p_base + np.array([t.x, t.y, t.z])
        if not np.all(np.isfinite(p_map)):
            return

        self.target_map = p_map[:2]
        self.target_map_time = self._now()

        # Target's heading, from the marker's normal. Only trust it when the
        # marker is reasonably face-on; a near-edge-on marker gives a garbage
        # normal (the same degeneracy that makes solvePnP return NaN).
        n_cam = det.get('normal_cam')
        if n_cam is None:
            return
        n_base = optical_to_body(n_cam)
        n_map = R @ n_base
        if abs(n_map[0]) < 1e-6 and abs(n_map[1]) < 1e-6:
            return
        # The normal points from the marker back at the camera; the marker is
        # on the target's REAR, so the target's forward is the opposite.
        self.target_yaw = math.atan2(-n_map[1], -n_map[0])

    def _follower_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rclpy.time.Time())
        except Exception:
            return None
        return np.array([tf.transform.translation.x, tf.transform.translation.y])

    # ------------------------------------------------------------------
    def goal_timer_callback(self):
        if self.target_map is None or self.nav_client is None:
            return

        now = self._now()
        age = now - self.target_map_time
        follower = self._follower_xy()
        if follower is None:
            self.get_logger().warn('No follower pose from TF yet.', throttle_duration_sec=5.0)
            return

        if age <= self.target_lost_grace_sec:
            self._follow(follower)
        else:
            self._recover(age)

    def _follow(self, follower):
        if self.recovery_goal_sent:
            self.get_logger().info('Target re-acquired; resuming follow.')
            self.recovery_goal_sent = False

        dist = float(np.linalg.norm(self.target_map - follower))

        # Hysteresis. Only meaningful while the target is FRESH -- applying it
        # to a stale target is what used to strand the follower in HOLDING
        # forever after the target rounded a corner.
        if self.following and dist <= self.stop_distance:
            self.following = False
            self._cancel_goal()
            self.state = 'HOLDING'
            self.get_logger().info(f'Within {dist:.2f} m; holding.')
            return
        if not self.following:
            if dist < self.resume_distance:
                return
            self.following = True
            self.get_logger().info(f'Target at {dist:.2f} m; resuming follow.')

        self.state = 'FOLLOWING'
        u = (self.target_map - follower) / dist
        goal_xy = self.target_map - u * self.standoff_distance
        goal_yaw = math.atan2(u[1], u[0])

        if self.last_goal_xy is not None and \
                float(np.linalg.norm(goal_xy - self.last_goal_xy)) < self.goal_update_min_dist:
            return
        if self._goal_in_flight:
            return
        self._send_goal(goal_xy, goal_yaw, 'follow')

    def _recover(self, age):
        """Target lost. Drive to where it was, facing where it went."""
        if age > (self.target_lost_grace_sec + self.recovery_timeout_sec):
            if self.state != 'LOST':
                self.get_logger().warn('Recovery timed out; target lost. Stopping.')
                self._cancel_goal()
                self.state = 'LOST'
            return

        if self.recovery_goal_sent or self._goal_in_flight:
            return

        if self.target_yaw is not None:
            h = np.array([math.cos(self.target_yaw), math.sin(self.target_yaw)])
            goal_yaw = self.target_yaw
        else:
            # No trustworthy heading (marker was too oblique). Fall back to
            # approaching along our own line of sight.
            follower = self._follower_xy()
            if follower is None:
                return
            d = self.target_map - follower
            n = float(np.linalg.norm(d))
            if n < 1e-6:
                return
            h = d / n
            goal_yaw = math.atan2(h[1], h[0])
            self.get_logger().warn('Recovering without a trusted target heading.')

        # Stop just short of the target's last position, along its heading, so
        # we never try to occupy the exact cell it may still be sitting in.
        goal_xy = self.target_map - h * self.recovery_goal_backoff

        self.state = 'RECOVERING'
        self.following = True   # so we resume cleanly on re-acquire
        self.get_logger().info(
            f'Target lost {age:.1f}s. Recovering to its last position '
            f'({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) facing {math.degrees(goal_yaw):.0f} deg.')
        self._send_goal(goal_xy, goal_yaw, 'recovery')
        self.recovery_goal_sent = True

    # ------------------------------------------------------------------
    def _send_goal(self, goal_xy, goal_yaw, kind):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('NavigateToPose server unavailable.',
                                   throttle_duration_sec=5.0)
            return
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal_xy[0])
        pose.pose.position.y = float(goal_xy[1])
        pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        if self.goal_pub is not None:
            self.goal_pub.publish(pose)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_in_flight = True
        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response_callback)
        self.last_goal_xy = np.asarray(goal_xy, dtype=float)
        self.get_logger().info(
            f'[{kind}] goal -> ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) '
            f'yaw {math.degrees(goal_yaw):.0f} deg')

    def _goal_response_callback(self, future):
        self._goal_in_flight = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Goal send failed: {exc}')
            return
        if not handle.accepted:
            # Most likely the goal is inside the target robot's inflated
            # footprint. Raise standoff_distance / recovery_goal_backoff, or
            # lower inflation_radius in the follow nav2 params.
            self.get_logger().warn('Nav2 REJECTED the goal (unreachable or in collision?).')
            self._goal_handle = None
            self.recovery_goal_sent = False   # allow a retry next tick
            return
        self._goal_handle = handle

    def _cancel_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self.last_goal_xy = None

    # ------------------------------------------------------------------
    def _detect_aruco(self, frame, overlay):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._aruco_new_api:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dictionary, parameters=self._aruco_params)

        seen = [] if ids is None else [int(i) for i in ids.flatten()]
        if ids is None:
            return None, seen

        for c, mid in zip(corners, ids.flatten()):
            if int(mid) != self.aruco_marker_id:
                continue
            pts = c.reshape(4, 2)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)
            cv2.polylines(overlay, [pts.astype(np.int32)], True, (0, 255, 0), 2)

            pose = None
            normal_cam = None
            if self.camera_matrix is not None:
                ok, rvec, tvec = cv2.solvePnP(
                    self._marker_obj_pts, pts.astype(np.float32),
                    self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    t = tvec.ravel()
                    if np.all(np.isfinite(t)) and t[2] > 0.0:
                        pose = (float(t[0]), float(t[1]), float(t[2]))
                        cv2.drawFrameAxes(overlay, self.camera_matrix, self.dist_coeffs,
                                          rvec, tvec, self.marker_length * 0.5, 2)
                        R, _ = cv2.Rodrigues(rvec)
                        n = R @ np.array([0.0, 0.0, 1.0])
                        # Orient the normal back toward the camera.
                        if float(np.dot(n, t)) > 0:
                            n = -n
                        # Reject an oblique marker's normal: its heading is noise.
                        cos_face_on = float(np.dot(n, -t / np.linalg.norm(t)))
                        if cos_face_on >= self.min_face_on_cosine:
                            normal_cam = n

            return ({'cx': float((x1 + x2) / 2), 'bbox_h': float(y2 - y1),
                     'label': f'aruco:{self.aruco_marker_id}', 'conf': 1.0,
                     'pose': pose, 'normal_cam': normal_cam}, seen)
        return None, seen

    def _detect_yolo(self, frame, overlay):
        results = self.model(frame, verbose=False, device=self.device)[0]
        best, best_conf, seen = None, 0.0, []
        for box in results.boxes:
            name = self.class_names.get(int(box.cls[0]), str(int(box.cls[0])))
            conf = float(box.conf[0])
            seen.append(f'{name}:{conf:.2f}')
            if name == self.target_class and conf >= self.confidence_threshold and conf > best_conf:
                best_conf, best = conf, box
        if best is None:
            return None, seen
        x1, y1, x2, y2 = best.xyxy[0].tolist()
        cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        return ({'cx': (x1 + x2) / 2, 'bbox_h': y2 - y1, 'label': self.target_class,
                 'conf': best_conf, 'pose': None, 'normal_cam': None}, seen)

    # ------------------------------------------------------------------
    def _draw_hud(self, overlay, w, h, seen):
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        if self.detection_mode == 'aruco':
            lines = [f'TARGET aruco id={self.aruco_marker_id} dict={self.aruco_dictionary_name}',
                     f'seen ids: {seen if seen else "none"}']
        else:
            lines = [f"TARGET class='{self.target_class}'", f'seen: {seen if seen else "none"}']

        pnp = 'PnP:on' if self.camera_matrix is not None else 'PnP:waiting camera_info'
        if self.control_mode == 'nav2':
            lines.append(f'state: {self.state}  ({pnp})')
            if self.target_map is not None:
                age = self._now() - self.target_map_time
                f = self._follower_xy()
                d = float(np.linalg.norm(self.target_map - f)) if f is not None else float('nan')
                yaw_s = ('n/a' if self.target_yaw is None
                         else f'{math.degrees(self.target_yaw):+.0f}d')
                lines.append(f'target {self.target_map[0]:+.2f},{self.target_map[1]:+.2f} '
                             f'yaw={yaw_s} dist={d:.2f}m age={age:.1f}s')
        else:
            lines.append(f'state: {"LOCKED" if self.target_locked else "SEARCHING"} ({pnp})')
            lin, ang = self.last_cmd
            lines.append(f'cmd lin={lin:+.2f} ang={ang:+.2f}')

        for i, text in enumerate(lines):
            yy = 22 + i * 22
            cv2.putText(overlay, text, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(overlay, text, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    # ------------------------------------------------------------------
    def _direct_control(self, det, w, h):
        error_x = (det['cx'] - w / 2.0) / (w / 2.0)
        angular_z = float(np.clip(-self.kp_angular * error_x,
                                  -self.max_angular_speed, self.max_angular_speed))
        if det.get('pose') is not None:
            err = det['pose'][2] - self.desired_distance
        else:
            err = 0.5 - (det['bbox_h'] / float(h))
        linear_x = float(np.clip(self.kp_linear * err,
                                 -0.5 * self.max_linear_speed, self.max_linear_speed))
        self._publish_cmd(linear_x, angular_z)
        self.last_cmd = (linear_x, angular_z)

    def _direct_no_detection(self, now):
        if self.target_locked and self.last_seen_time is not None:
            elapsed = now - self.last_seen_time
            if elapsed < self.lost_target_grace_sec:
                self._publish_cmd(*self.last_cmd)
                return
            if elapsed < (self.lost_target_grace_sec + self.recovery_timeout_sec):
                direction = -1.0 if (self.last_pose and self.last_pose[0] > 0) else 1.0
                self._publish_cmd(0.0, direction * self.recovery_rotation_speed)
                return
        self.target_locked = False
        self._publish_cmd(0.0, 0.0)
        self.last_cmd = (0.0, 0.0)
        self.locked_pub.publish(Bool(data=False))

    def _publish_cmd(self, linear, angular):
        if self.cmd_vel_pub is None:
            return
        linear, angular = float(linear), float(angular)
        # A NaN on cmd_vel makes the diff-drive plugin reject and latch,
        # freezing the robot. np.clip does not sanitize NaN.
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.get_logger().warn('Non-finite cmd_vel blocked; stopping.')
            linear, angular = 0.0, 0.0
            self.last_cmd = (0.0, 0.0)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_base_frame
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------
    def _send_alert(self, overlay, label, conf, pose, now):
        os.makedirs(self.snapshot_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.snapshot_dir, f"{label.replace(':', '_')}_{ts}.jpg")
        cv2.imwrite(path, overlay)

        alert = {'event': 'target_detected', 'detection_mode': self.detection_mode,
                 'control_mode': self.control_mode, 'target': label,
                 'confidence': round(conf, 3), 'timestamp': ts, 'image_path': path}
        if pose is not None:
            alert['range_m'] = round(pose[2], 3)
        if self.target_map is not None:
            alert['target_map_xy'] = [round(float(self.target_map[0]), 3),
                                      round(float(self.target_map[1]), 3)]
        if self.target_yaw is not None:
            alert['target_yaw_deg'] = round(math.degrees(self.target_yaw), 1)
        self.alert_pub.publish(String(data=json.dumps(alert)))
        extra = f' range={pose[2]:.2f}m' if pose is not None else ''
        self.get_logger().info(f"[OPERATOR ALERT] '{label}' detected{extra} -> {path}")


def main(args=None):
    rclpy.init(args=args)
    node = VisionFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
