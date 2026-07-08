#!/usr/bin/env python3
"""Deterministic, auditable executor.

Consumes /mission/validated, resolves it to a concrete list of Nav2 poses,
and drives the robot via Nav2's FollowWaypoints action. No LLM, no
randomness: the same validated JSON always yields the same goal sequence.
Every run logs an audit line (mission_id + sha256(json) + pose count).

FollowWaypoints (not NavigateThroughPoses) is used deliberately: it visits
each waypoint as its own separate navigation goal, so every point in a loop
is actually checked and visited. NavigateThroughPoses instead treats the
whole route as ONE continuous path and only checks arrival at the FINAL
pose -- if a repeated command's final pose happens to equal the robot's
current position (e.g. issuing "patrol twice" again right after finishing
a loop there), it reports the goal reached instantly without traversing
anything. FollowWaypoints avoids that: only an already-satisfied individual
waypoint is skipped, not the rest of the route.
"""
import hashlib
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from mission_schemas.schema import CommandType, Mission


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MissionExecutor(Node):
    def __init__(self):
        super().__init__("mission_executor")
        self.declare_parameter("routes_file", "")
        self.routes = self._load_routes(self.get_parameter("routes_file").value)
        self.sub = self.create_subscription(
            String, "/mission/validated", self.on_validated, 10)
        self.ac = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.get_logger().info("Executor ready (backend: FollowWaypoints).")

    def _load_routes(self, path: str) -> dict:
        if path and os.path.exists(path):
            with open(path) as fh:
                return (yaml.safe_load(fh) or {}).get("routes", {})
        self.get_logger().warn("routes_file not found; only explicit waypoints work.")
        return {}

    def on_validated(self, msg: String):
        mission = Mission.model_validate_json(msg.data)
        audit = hashlib.sha256(msg.data.encode()).hexdigest()[:12]
        poses = self._resolve_poses(mission)
        self.get_logger().info(
            f"AUDIT mission={mission.mission_id} sha={audit} "
            f"cmd={mission.command_type.value} poses={len(poses)}")
        if not poses:
            self.get_logger().error("no poses resolved; nothing to execute")
            return
        if not self.ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "follow_waypoints server unavailable (is Nav2 active?)")
            return
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self.get_logger().info(f"sending {len(poses)} waypoints to Nav2...")
        fut = self.ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _resolve_poses(self, m: Mission):
        if m.command_type == CommandType.PATROL_LOOP and m.route_name:
            pts = self.routes.get(m.route_name, [])
            one_loop = [(p["x"], p["y"], p.get("yaw", 0.0)) for p in pts]
        else:
            one_loop = [(wp.x, wp.y, wp.yaw) for wp in m.waypoints]
        seq = one_loop * max(1, m.loops)
        return [self._pose(x, y, yaw) for (x, y, yaw) in seq]

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _on_feedback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f"heading to waypoint index {idx}...")

    def _on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal")
            return
        self.get_logger().info("goal accepted; navigating waypoints...")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        result = fut.result().result
        missed = list(result.missed_waypoints) if result.missed_waypoints else []
        if missed:
            # missed_waypoints' element type differs across ROS distros
            # (Humble: int32 index; Jazzy: WaypointStatus struct), so just
            # report the count rather than assume a specific field shape.
            self.get_logger().warn(
                f"mission complete WITH {len(missed)} missed waypoint(s)")
        else:
            self.get_logger().info("mission complete: all waypoints visited")


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
