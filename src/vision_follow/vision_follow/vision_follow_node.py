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
# Omokai take-home, Phase 4 / Challenge 3 (Vision AI target detection + follow).
#
# Sources / citations (Section 4.2):
#   - Ultralytics YOLO, https://github.com/ultralytics/ultralytics (AGPL-3.0),
#     public YOLO(...) inference API only, when detection_mode='yolo'.
#   - OpenCV ArUco + solvePnP (opencv-contrib, BSD), when
#     detection_mode='aruco'. Standard fiducial pose-estimation technique.
#   - monemati/PX4-ROS2-Gazebo-YOLOv8,
#     https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8 -- referenced for
#     the general "sim camera -> detector -> act on detection" shape only;
#     original implementation for a ROS 2 / Gazebo TurtleBot3 diff-drive base.
#
# Executor role: this node is the deterministic executor for the vision
# challenge. In aruco mode it recovers the target's full 3D pose relative to
# the follower via solvePnP on the marker corners, so both the follow control
# law AND the lost-target recovery act on real geometry (metric distance z +
# lateral bearing sign x), not pixel heuristics. Target selection
# (class name / marker id) is the only thing a future validated mission JSON
# sets, via 'vision/set_target_class'.

import json
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String


class VisionFollowNode(Node):

    def __init__(self):
        super().__init__('vision_follow_node')

        # ---- Parameters -----------------------------------------------
        self.declare_parameter('detection_mode', 'aruco')  # 'yolo' or 'aruco'

        # YOLO-mode params
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('device', 'cpu')

        # ArUco-mode params
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_marker_id', 0)
        # Real-world marker side length in metres. MUST match the physical
        # size of the marker on the target (in our sim, the aruco box face is
        # 0.08 m). Distance accuracy scales directly with this; bearing sign
        # stays correct even if it's slightly off.
        self.declare_parameter('marker_length', 0.08)

        self.declare_parameter('infer_every_n_frames', 2)

        # Follow-controller gains/limits. Caps default to the same magnitudes
        # turtlebot3_drive.py uses (0.15 / 0.4), staying in the fleet's safe
        # envelope. In aruco mode, distance control is metric:
        # desired_distance is a real setpoint in metres.
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('desired_distance', 0.6)          # metres (aruco/pnp)
        self.declare_parameter('desired_bbox_height_ratio', 0.5)  # fallback (yolo)

        # Lost-target recovery. With PnP we cache the last known pose and
        # steer recovery off it: rotate toward sign(last_x), and if the
        # target was far (large last_z) also creep forward along that bearing.
        self.declare_parameter('lost_target_grace_sec', 0.4)
        self.declare_parameter('recovery_enabled', True)
        self.declare_parameter('recovery_duration_sec', 8.0)
        self.declare_parameter('recovery_far_distance', 1.2)   # z above this = "lost far"
        self.declare_parameter('recovery_rotation_speed', 0.3)

        # Operator alert.
        self.declare_parameter('snapshot_dir', '/tmp/vision_follow_snapshots')
        self.declare_parameter('alert_cooldown_sec', 15.0)

        g = self.get_parameter
        self.detection_mode = g('detection_mode').value
        self.target_class = g('target_class').value
        self.confidence_threshold = float(g('confidence_threshold').value)
        self.model_path = g('model_path').value
        self.device = g('device').value
        self.aruco_dictionary_name = g('aruco_dictionary').value
        self.aruco_marker_id = int(g('aruco_marker_id').value)
        self.marker_length = float(g('marker_length').value)
        self.infer_every_n_frames = int(g('infer_every_n_frames').value)
        self.max_linear_speed = float(g('max_linear_speed').value)
        self.max_angular_speed = float(g('max_angular_speed').value)
        self.kp_angular = float(g('kp_angular').value)
        self.kp_linear = float(g('kp_linear').value)
        self.desired_distance = float(g('desired_distance').value)
        self.desired_bbox_height_ratio = float(g('desired_bbox_height_ratio').value)
        self.lost_target_grace_sec = float(g('lost_target_grace_sec').value)
        self.recovery_enabled = bool(g('recovery_enabled').value)
        self.recovery_duration_sec = float(g('recovery_duration_sec').value)
        self.recovery_far_distance = float(g('recovery_far_distance').value)
        self.recovery_rotation_speed = float(g('recovery_rotation_speed').value)
        self.snapshot_dir = g('snapshot_dir').value
        self.alert_cooldown_sec = float(g('alert_cooldown_sec').value)

        # ---- State ------------------------------------------------------
        self.bridge = CvBridge()
        self.frame_count = 0
        self.target_locked = False
        self.last_seen_time = None
        self.last_alert_time = 0.0
        self.last_cmd = (0.0, 0.0)
        self.model = None
        self.class_names = {}

        # Camera intrinsics (populated from camera_info). Until we have them,
        # PnP is skipped and we fall back to pixel-offset control.
        self.camera_matrix = None
        self.dist_coeffs = None

        # Last known target pose (camera frame) for recovery: (x, y, z).
        self.last_pose = None

        if self.detection_mode == 'yolo':
            self._load_yolo_model()
        elif self.detection_mode == 'aruco':
            self._init_aruco()
        else:
            raise ValueError(f"Unknown detection_mode='{self.detection_mode}'")

        # ---- Pub/Sub ----------------------------------------------------
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)
        self.target_sub = self.create_subscription(
            String, 'vision/set_target_class', self.set_target_callback, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.detection_image_pub = self.create_publisher(Image, 'vision/detection_image', 10)
        self.alert_pub = self.create_publisher(String, 'vision/detection_alert', 10)
        self.locked_pub = self.create_publisher(Bool, 'vision/target_locked', 10)

        desc = (self.target_class if self.detection_mode == 'yolo'
                else f"aruco id={self.aruco_marker_id} dict={self.aruco_dictionary_name}")
        self.get_logger().info(
            f"vision_follow_node ready. mode='{self.detection_mode}' target={desc} "
            f"marker_length={self.marker_length}m. Overlay on 'vision/detection_image'.")

    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        # Latch once; intrinsics don't change. k is row-major 3x3.
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.get_logger().info(
                f"Camera intrinsics latched: fx={self.camera_matrix[0,0]:.1f} "
                f"fy={self.camera_matrix[1,1]:.1f} "
                f"cx={self.camera_matrix[0,2]:.1f} cy={self.camera_matrix[1,2]:.1f}. "
                f"PnP pose estimation active.")

    # ------------------------------------------------------------------
    def _load_yolo_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                'ultralytics not installed: pip install "numpy<2" ultralytics --break-system-packages')
            raise exc
        self.model = YOLO(self.model_path)
        names = self.model.names
        self.class_names = names if isinstance(names, dict) else dict(enumerate(names))

    def _init_aruco(self):
        aruco_dict_id = getattr(cv2.aruco, self.aruco_dictionary_name)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._aruco_new_api = True
            dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            self._aruco_detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        else:
            self._aruco_new_api = False
            self._aruco_dictionary = cv2.aruco.Dictionary_get(aruco_dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()

        half = self.marker_length / 2.0
        # Object points in marker frame, matching ArUco's TL,TR,BR,BL order.
        self._marker_obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

    # ------------------------------------------------------------------
    def set_target_callback(self, msg: String):
        v = msg.data.strip()
        if not v:
            return
        if self.detection_mode == 'yolo':
            if v != self.target_class:
                self.get_logger().info(f"Target class: '{self.target_class}' -> '{v}'")
                self.target_class = v
        else:
            try:
                new_id = int(v)
            except ValueError:
                self.get_logger().error(f"aruco mode expects integer marker id, got '{v}'")
                return
            if new_id != self.aruco_marker_id:
                self.get_logger().info(f"Target marker id: {self.aruco_marker_id} -> {new_id}")
                self.aruco_marker_id = new_id
        self.target_locked = False
        self.last_seen_time = None
        self.last_pose = None

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % max(self.infer_every_n_frames, 1) != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return

        h, w = frame.shape[:2]
        overlay = frame.copy()

        if self.detection_mode == 'aruco':
            det, seen_ids = self._detect_aruco(frame, overlay)
            self._draw_hud(overlay, w, h, seen_ids)
        else:
            det, seen = self._detect_yolo(frame, overlay)
            self._draw_hud(overlay, w, h, seen)

        try:
            out = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            out.header = msg.header
            self.detection_image_pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f"overlay publish failed: {exc}")

        now = time.time()
        if det is not None:
            self._handle_detection(overlay, det, w, h, now)
        else:
            self._handle_no_detection(now)

    # ------------------------------------------------------------------
    # ArUco detection + PnP. Returns (det_dict | None, seen_ids list).
    # det_dict keys: cx, bbox_h, label, conf, pose(x,y,z)|None, rvec, tvec.
    # ------------------------------------------------------------------
    def _detect_aruco(self, frame, overlay):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._aruco_new_api:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dictionary, parameters=self._aruco_params)

        seen_ids = [] if ids is None else [int(i) for i in ids.flatten()]
        if ids is None:
            return None, seen_ids

        for c, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != self.aruco_marker_id:
                continue
            pts = c.reshape(4, 2)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)
            cx = float((x1 + x2) / 2.0)
            bbox_h = float(y2 - y1)

            cv2.polylines(overlay, [pts.astype(np.int32)], True, (0, 255, 0), 2)

            pose = None
            rvec = tvec = None
            if self.camera_matrix is not None:
                ok, rvec, tvec = cv2.solvePnP(
                    self._marker_obj_pts, pts.astype(np.float32),
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                    # Guard against degenerate PnP solutions (near edge-on
                    # markers can yield NaN/inf or nonsensical negative depth).
                    # A bad solve must not enter the pose cache or the control
                    # law -- treat it as "pose unavailable" for this frame and
                    # fall back to pixel-offset control, rather than poisoning
                    # cmd_vel with NaN.
                    if all(math.isfinite(v) for v in (tx, ty, tz)) and tz > 0.0:
                        pose = (tx, ty, tz)
                        cv2.drawFrameAxes(overlay, self.camera_matrix, self.dist_coeffs,
                                          rvec, tvec, self.marker_length * 0.5, 2)

            return ({'cx': cx, 'bbox_h': bbox_h, 'label': f"aruco:{self.aruco_marker_id}",
                     'conf': 1.0, 'pose': pose, 'rvec': rvec, 'tvec': tvec}, seen_ids)
        return None, seen_ids

    def _detect_yolo(self, frame, overlay):
        results = self.model(frame, verbose=False, device=self.device)[0]
        best, best_conf, seen = None, 0.0, []
        for box in results.boxes:
            name = self.class_names.get(int(box.cls[0]), str(int(box.cls[0])))
            conf = float(box.conf[0])
            seen.append(f"{name}:{conf:.2f}")
            if name == self.target_class and conf >= self.confidence_threshold and conf > best_conf:
                best_conf, best = conf, box
        if best is None:
            return None, seen
        x1, y1, x2, y2 = best.xyxy[0].tolist()
        cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        return ({'cx': (x1 + x2) / 2.0, 'bbox_h': y2 - y1, 'label': self.target_class,
                 'conf': best_conf, 'pose': None, 'rvec': None, 'tvec': None}, seen)

    # ------------------------------------------------------------------
    def _draw_hud(self, overlay, w, h, seen):
        """Always-on heads-up display so the frame is self-describing."""
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        if self.detection_mode == 'aruco':
            target_line = f"TARGET aruco id={self.aruco_marker_id} dict={self.aruco_dictionary_name}"
            seen_line = f"seen ids: {seen if seen else 'none'}"
        else:
            target_line = f"TARGET class='{self.target_class}' conf>={self.confidence_threshold}"
            seen_line = f"seen: {seen if seen else 'none'}"

        lock_txt = "LOCKED" if self.target_locked else "SEARCHING"
        lock_col = (0, 255, 0) if self.target_locked else (0, 165, 255)
        pnp_txt = "PnP:on" if self.camera_matrix is not None else "PnP:waiting camera_info"

        lines = [target_line, seen_line, f"state: {lock_txt}  ({pnp_txt})"]
        if self.last_pose is not None:
            x, y, z = self.last_pose
            lines.append(f"pose  x={x:+.2f}  z={z:.2f}m  bearing={'RIGHT' if x > 0 else 'LEFT'}")
        lin, ang = self.last_cmd
        lines.append(f"cmd   lin={lin:+.2f} m/s  ang={ang:+.2f} rad/s")

        y0 = 22
        for i, text in enumerate(lines):
            yy = y0 + i * 22
            cv2.putText(overlay, text, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            col = lock_col if i == 2 else (255, 255, 255)
            cv2.putText(overlay, text, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)

    # ------------------------------------------------------------------
    def _handle_detection(self, overlay, det, w, h, now):
        newly = not self.target_locked
        if newly or (now - self.last_alert_time) >= self.alert_cooldown_sec:
            self._send_alert(overlay, det['label'], det['conf'], det.get('pose'), now)
            self.last_alert_time = now

        self.target_locked = True
        self.last_seen_time = now
        if det.get('pose') is not None:
            self.last_pose = det['pose']

        # --- Control law ---
        # Angular: center the target horizontally. With PnP we have the real
        # lateral offset, but pixel offset is smoother frame-to-frame for the
        # turn command, so use pixel error for angular and reserve the pose
        # vector for distance + recovery.
        error_x = (det['cx'] - w / 2.0) / (w / 2.0)  # [-1, 1]
        angular_z = float(np.clip(-self.kp_angular * error_x,
                                   -self.max_angular_speed, self.max_angular_speed))

        # Linear: metric distance if PnP available, else bbox-height proxy.
        if det.get('pose') is not None:
            z = det['pose'][2]
            error_dist = z - self.desired_distance   # positive = too far -> advance
            linear_x = float(np.clip(self.kp_linear * error_dist,
                                      -0.5 * self.max_linear_speed, self.max_linear_speed))
        else:
            bbox_ratio = det['bbox_h'] / float(h)
            error_dist = self.desired_bbox_height_ratio - bbox_ratio
            linear_x = float(np.clip(self.kp_linear * error_dist,
                                      -0.5 * self.max_linear_speed, self.max_linear_speed))

        self._publish_cmd(linear_x, angular_z)
        self.last_cmd = (linear_x, angular_z)
        self.locked_pub.publish(Bool(data=True))

    # ------------------------------------------------------------------
    def _handle_no_detection(self, now):
        if self.target_locked and self.last_seen_time is not None:
            elapsed = now - self.last_seen_time

            if elapsed < self.lost_target_grace_sec:
                self._publish_cmd(*self.last_cmd)   # coast through a dropped frame
                return

            if self.recovery_enabled and elapsed < (self.lost_target_grace_sec + self.recovery_duration_sec):
                self._publish_cmd(*self._recovery_cmd())
                return

            self.get_logger().info("Lost target; recovery timed out, halting.")

        self.target_locked = False
        self._publish_cmd(0.0, 0.0)
        self.last_cmd = (0.0, 0.0)
        self.locked_pub.publish(Bool(data=False))

    def _recovery_cmd(self):
        """Steer recovery off the last known pose. Rotate toward the side the
        target was last on (sign of lateral x); if it was far when lost, also
        creep forward along that bearing to close distance."""
        if self.last_pose is None:
            # No pose ever (e.g. lost before camera_info arrived): blind spin.
            return 0.0, self.recovery_rotation_speed

        x, _, z = self.last_pose
        # x>0 means target was to the RIGHT -> negative angular_z turns right.
        direction = -1.0 if x > 0 else 1.0
        angular = direction * self.recovery_rotation_speed
        linear = 0.0
        if z > self.recovery_far_distance:
            linear = 0.5 * self.max_linear_speed   # target outran us; push forward
        return linear, angular

    # ------------------------------------------------------------------
    def _publish_cmd(self, linear, angular):
        linear = float(linear)
        angular = float(angular)
        # Hard safety gate: a NaN/inf reaching cmd_vel makes the diff-drive
        # plugin reject the command and (worse) can latch, freezing the robot.
        # np.clip does NOT sanitize NaN, so any upstream numerical fault must
        # be caught here, at the single choke point before the wheels.
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.get_logger().warn(
                f"Non-finite cmd_vel blocked (lin={linear}, ang={angular}); sending stop.")
            linear, angular = 0.0, 0.0
            self.last_cmd = (0.0, 0.0)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------
    def _send_alert(self, overlay, label, conf, pose, now):
        os.makedirs(self.snapshot_dir, exist_ok=True)
        ts = datetime.fromtimestamp(now).strftime('%Y%m%d_%H%M%S')
        safe = label.replace(':', '_')
        path = os.path.join(self.snapshot_dir, f"{safe}_{ts}.jpg")
        cv2.imwrite(path, overlay)

        alert = {'event': 'target_detected', 'detection_mode': self.detection_mode,
                 'target': label, 'confidence': round(conf, 3), 'timestamp': ts,
                 'image_path': path}
        if pose is not None:
            alert['range_m'] = round(pose[2], 3)
            alert['lateral_m'] = round(pose[0], 3)
        self.alert_pub.publish(String(data=json.dumps(alert)))

        extra = f" range={pose[2]:.2f}m" if pose is not None else ""
        self.get_logger().info(
            f"[OPERATOR ALERT] '{label}' detected (conf={conf:.2f}){extra} "
            f"-> snapshot {path}, published on 'vision/detection_alert'")


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
