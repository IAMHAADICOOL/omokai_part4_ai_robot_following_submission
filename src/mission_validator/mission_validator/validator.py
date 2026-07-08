#!/usr/bin/env python3
"""Guardrail. Re-validates candidate JSON against the schema, then runs
safety/semantic checks. Passes -> /mission/validated, fails -> /mission/rejected.

Phase 1 commands: patrol_loop, waypoint_nav (single robot).
Phase 2 commands: formation_sweep (multi-robot, handled by formation_coordinator).
"""
import os

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from mission_schemas.schema import CommandType, Mission

SUPPORTED = {
    CommandType.PATROL_LOOP,
    CommandType.WAYPOINT_NAV,
    CommandType.FORMATION_SWEEP,
}
HARD_MAX_LINEAR = 0.22
HARD_MAX_ANGULAR = 2.84
MAX_LOOPS = 10
MAX_SPACING = 2.0
VALID_FORMATIONS = {"line", "column", "wedge"}


class MissionValidator(Node):
    def __init__(self):
        super().__init__("mission_validator")
        self.declare_parameter("routes_file", "")
        self.routes = self._load_routes(self.get_parameter("routes_file").value)
        self.sub = self.create_subscription(
            String, "/mission/candidate", self.on_candidate, 10)
        self.pub_ok = self.create_publisher(String, "/mission/validated", 10)
        self.pub_no = self.create_publisher(String, "/mission/rejected", 10)
        self.get_logger().info(
            f"Validator ready. Known routes: {sorted(self.routes)}")

    def _load_routes(self, path):
        if path and os.path.exists(path):
            with open(path) as fh:
                return (yaml.safe_load(fh) or {}).get("routes", {})
        self.get_logger().warn("routes_file not found; route checks limited.")
        return {}

    def on_candidate(self, msg):
        raw = msg.data
        try:
            mission = Mission.model_validate_json(raw)
        except Exception as exc:  # noqa: BLE001
            return self._reject(f"schema validation failed: {exc}")
        err = self._safety_check(mission)
        if err:
            return self._reject(err, mission_id=mission.mission_id)
        out = String()
        out.data = mission.model_dump_json()
        self.pub_ok.publish(out)
        self.get_logger().info(
            f"ACCEPTED {mission.mission_id} ({mission.command_type.value})")

    def _safety_check(self, m):
        if m.command_type not in SUPPORTED:
            return f"command_type {m.command_type.value} not supported"
        if m.constraints.max_linear_speed > HARD_MAX_LINEAR:
            return (f"max_linear_speed {m.constraints.max_linear_speed} "
                    f"> limit {HARD_MAX_LINEAR}")
        if m.constraints.max_angular_speed > HARD_MAX_ANGULAR:
            return (f"max_angular_speed {m.constraints.max_angular_speed} "
                    f"> limit {HARD_MAX_ANGULAR}")
        if not (1 <= m.loops <= MAX_LOOPS):
            return f"loops {m.loops} out of range 1..{MAX_LOOPS}"

        route_ok = (m.route_name is None) or (not self.routes) or \
            (m.route_name in self.routes)

        if m.command_type == CommandType.PATROL_LOOP:
            if m.route_name is None and not m.waypoints:
                return "patrol_loop needs a route_name or explicit waypoints"
            if not route_ok:
                return (f"unknown route_name {m.route_name!r}; "
                        f"known: {sorted(self.routes)}")
        elif m.command_type == CommandType.WAYPOINT_NAV:
            if not m.waypoints:
                return "waypoint_nav needs at least one waypoint"
        elif m.command_type == CommandType.FORMATION_SWEEP:
            if m.formation is None:
                return "formation_sweep needs a formation (type + spacing_m)"
            if m.formation.type.value not in VALID_FORMATIONS:
                return (f"unknown formation type {m.formation.type.value!r}; "
                        f"valid: {sorted(VALID_FORMATIONS)}")
            if not (0.0 < m.formation.spacing_m <= MAX_SPACING):
                return (f"spacing_m {m.formation.spacing_m} out of range "
                        f"(0, {MAX_SPACING}]")
            if m.route_name is None and not m.waypoints:
                return "formation_sweep needs a center route_name or waypoints"
            if not route_ok:
                return (f"unknown route_name {m.route_name!r}; "
                        f"known: {sorted(self.routes)}")
        return None

    def _reject(self, reason, mission_id="?"):
        msg = String()
        msg.data = f"{mission_id}: {reason}"
        self.pub_no.publish(msg)
        self.get_logger().warn(f"REJECTED {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
