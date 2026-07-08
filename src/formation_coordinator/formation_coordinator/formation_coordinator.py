#!/usr/bin/env python3
"""Formation coordinator (Challenge 1).

Consumes /mission/validated. For command_type == formation_sweep it takes the
squad's CENTER path (a named route or explicit waypoints), computes each
robot's offset path for the requested formation (line/column/wedge), and
drives each namespaced robot via its own /<ns>/follow_waypoints action.

The LLM issues squad-level INTENT (formation type + spacing + which route);
per-robot geometry and dispatch are deterministic here. Single-robot commands
(patrol_loop / waypoint_nav) are ignored -- those belong to mission_executor.
"""
import hashlib
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String

from formation_coordinator.geometry import robot_path
from mission_schemas.schema import CommandType, Mission


class FormationCoordinator(Node):
    def __init__(self):
        super().__init__("formation_coordinator")
        self.declare_parameter(
            "robot_namespaces", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter("robots_yaml", "")
        self.declare_parameter("routes_file", "")
        self.ns_list = self._resolve_namespaces(
            self.get_parameter("robot_namespaces").value,
            self.get_parameter("robots_yaml").value)
        self.routes = self._load_routes(self.get_parameter("routes_file").value)
        self._action_clients = {
            ns: ActionClient(self, FollowWaypoints, f"/{ns}/follow_waypoints")
            for ns in self.ns_list
        }
        self.sub = self.create_subscription(
            String, "/mission/validated", self.on_validated, 10)
        self.get_logger().info(
            f"Formation coordinator ready. Robots: {self.ns_list}")

    def _resolve_namespaces(self, explicit_ns, robots_yaml_path):
        """Single source of truth for squad membership: if `robot_namespaces`
        is explicitly passed, use it verbatim (override). Otherwise, derive
        the list from the SAME robots.yaml that spawns the robots (only
        entries with enabled: true) -- so the spawn list and the drive list
        can never silently drift apart. Falls back to ['tb1','tb3'] if
        neither is usable, matching the known-working demo squad."""
        if explicit_ns:
            return list(explicit_ns)
        if robots_yaml_path and os.path.exists(robots_yaml_path):
            with open(robots_yaml_path) as f:
                robots = yaml.safe_load(f).get("robots", [])
            names = [r["name"] for r in robots if r.get("enabled", True)]
            if names:
                return names
        self.get_logger().warn(
            "No robot_namespaces param and no usable robots_yaml; "
            "falling back to ['tb1', 'tb3'].")
        return ["tb1", "tb3"]

    def _load_routes(self, path):
        if path and os.path.exists(path):
            with open(path) as fh:
                return (yaml.safe_load(fh) or {}).get("routes", {})
        self.get_logger().warn("routes_file not found; only explicit waypoints work.")
        return {}

    def on_validated(self, msg):
        mission = Mission.model_validate_json(msg.data)
        if mission.command_type != CommandType.FORMATION_SWEEP:
            return  # single-robot command; mission_executor handles it
        center = self._center_path(mission)
        if not center:
            self.get_logger().error("no center path resolved")
            return
        ftype = mission.formation.type.value if mission.formation else "line"
        spacing = mission.formation.spacing_m if mission.formation else 0.5
        loops = max(1, mission.loops)
        audit = hashlib.sha256(msg.data.encode()).hexdigest()[:12]
        n = len(self.ns_list)
        self.get_logger().info(
            f"AUDIT mission={mission.mission_id} sha={audit} "
            f"formation={ftype} spacing={spacing} robots={n} loops={loops}")
        for i, ns in enumerate(self.ns_list):
            pts = robot_path(center, i, n, ftype, spacing) * loops
            poses = [self._pose(x, y) for (x, y) in pts]
            self._send(ns, poses)

    def _center_path(self, m):
        if m.route_name and m.route_name in self.routes:
            return [(p["x"], p["y"]) for p in self.routes[m.route_name]]
        return [(wp.x, wp.y) for wp in m.waypoints]

    def _pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        return p

    def _send(self, ns, poses):
        client = self._action_clients[ns]
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"/{ns}/follow_waypoints unavailable (is {ns} Nav2 up?)")
            return
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self.get_logger().info(f"{ns}: sending {len(poses)} waypoints")
        client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = FormationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
