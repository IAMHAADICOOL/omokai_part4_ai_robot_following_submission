#!/usr/bin/env python3
"""LLM planner: NL prompt -> CANDIDATE mission JSON via local Ollama.

Uses Ollama structured outputs (the Mission JSON schema is passed to
`format`) at temperature 0. The LLM ONLY proposes; the validator polices it.
"""
import os
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mission_schemas.schema import mission_json_schema

try:
    import ollama
except Exception:  # pragma: no cover
    ollama = None

SYSTEM_PROMPT = """You are a mission planner for TurtleBot3 ground robots.
Convert the operator instruction into ONE mission object matching the schema.

AVAILABLE ROUTES (use exactly these strings for route_name):
  "perimeter" - single-robot patrol loop around the arena
  "sweep"     - squad center path for formation moves

COMMAND TYPES:

1. patrol_loop  (single robot)
   Fields: command_type="patrol_loop", route_name="perimeter", loops=N
   Example prompt: "Patrol the perimeter twice"
   -> route_name="perimeter", loops=2

2. waypoint_nav  (single robot)
   Fields: command_type="waypoint_nav", waypoints=[...]

3. formation_sweep  (multi-robot squad)
   Fields: command_type="formation_sweep", route_name="sweep",
           formation={type, spacing_m}, loops=1
   formation.type must be one of: "line", "column", "wedge"
   ALWAYS set route_name="sweep" for formation commands.
   NEVER leave route_name null or empty for formation_sweep.

   Example prompts and correct output:
   "Sweep the area in a wedge"
   -> command_type="formation_sweep", route_name="sweep",
      formation={"type":"wedge","spacing_m":0.5}, loops=1

   "Move in a line formation"
   -> command_type="formation_sweep", route_name="sweep",
      formation={"type":"line","spacing_m":0.5}, loops=1

   "Drive in single file"
   -> command_type="formation_sweep", route_name="sweep",
      formation={"type":"column","spacing_m":0.5}, loops=1

RULES:
- max_linear_speed <= 0.22, max_angular_speed <= 2.84
- Do not invent waypoint coordinates; always use route_name
- Return only the mission JSON object, nothing else"""


# Explicit formation keywords -> type. The local 3B model is unreliable at this
# one categorical choice (it often returns a valid-but-wrong enum), so when the
# operator states the formation explicitly we set it deterministically. The LLM
# still handles command_type, route, loops, etc.
FORMATION_KEYWORDS = [
    ("column", ["single file", "single-file", "column", "one behind", "in file", "line astern"]),
    ("wedge",  ["wedge", "v-shape", "v shape", "v formation", "arrow", "chevron"]),
    ("line",   ["line formation", "in a line", "abreast", "side by side", "side-by-side", "line abreast"]),
]


def _formation_type_from_prompt(prompt):
    """Return 'line'/'column'/'wedge' if the prompt names one explicitly, else None."""
    p = prompt.lower()
    for ftype, words in FORMATION_KEYWORDS:
        if any(w in p for w in words):
            return ftype
    return None


class LLMPlanner(Node):
    def __init__(self):
        super().__init__("llm_planner")
        self.model = os.environ.get("LLM_MODEL", "qwen2.5:3b")
        self.schema = mission_json_schema()
        self.sub = self.create_subscription(
            String, "/mission/prompt", self.on_prompt, 10)
        self.pub = self.create_publisher(String, "/mission/candidate", 10)
        if ollama is None:
            self.get_logger().error(
                "python 'ollama' package not importable. pip install ollama")
        self.get_logger().info(f"Planner ready (model={self.model}).")

    def on_prompt(self, msg):
        prompt = msg.data
        self.get_logger().info(f"prompt in: {prompt!r} -> querying Ollama...")
        if ollama is None:
            return
        try:
            resp = ollama.chat(
                model=self.model,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": prompt},
                ],
                format=self.schema,
                options={"temperature": 0},
            )
            candidate = resp["message"]["content"]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Ollama call failed: {exc}")
            return

        candidate = self._ensure_id(candidate)
        candidate = self._apply_formation_keyword(prompt, candidate)
        out = String()
        out.data = candidate
        self.pub.publish(out)
        self.get_logger().info(f"candidate JSON out:\n{candidate}")

    @staticmethod
    def _apply_formation_keyword(prompt, candidate):
        """If the operator named a formation explicitly, force that type in the
        candidate JSON (overriding whatever the LLM guessed)."""
        import json
        ftype = _formation_type_from_prompt(prompt)
        if ftype is None:
            return candidate
        try:
            data = json.loads(candidate)
        except Exception:
            return candidate
        if data.get("command_type") == "formation_sweep":
            fm = data.get("formation") or {}
            fm["type"] = ftype
            fm.setdefault("spacing_m", 0.5)
            data["formation"] = fm
            data.setdefault("route_name", "sweep")
            return json.dumps(data)
        return candidate

    @staticmethod
    def _ensure_id(candidate):
        import json
        try:
            data = json.loads(candidate)
        except Exception:
            return candidate
        if not data.get("mission_id"):
            data["mission_id"] = f"m-{uuid.uuid4().hex[:8]}"
        return json.dumps(data)


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
