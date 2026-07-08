"""Mission contract shared across the whole pipeline.

ONE Pydantic model, three jobs:
  1. Serialized to JSON schema -> Ollama `format` (constrains LLM output shape).
  2. Re-validated by mission_validator (Ollama can truncate) + safety checks.
  3. Types the input to mission_executor.

Safety/semantic checks live in mission_validator, NOT here.
"""
from __future__ import annotations

from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class CommandType(str, Enum):
    PATROL_LOOP = "patrol_loop"        # Phase 1 - repeat a named route N times
    WAYPOINT_NAV = "waypoint_nav"      # Phase 1 - go through explicit waypoints
    FORMATION_SWEEP = "formation_sweep"  # Phase 2
    EXPLORE_SLAM = "explore_slam"      # Phase 3
    FOLLOW_TARGET = "follow_target"    # Phase 4


class FormationType(str, Enum):
    LINE = "line"
    COLUMN = "column"
    WEDGE = "wedge"


class Waypoint(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class Formation(BaseModel):
    type: FormationType
    spacing_m: float = 1.0


class Target(BaseModel):
    object_class: str


class Constraints(BaseModel):
    # TB3 burger physical ceilings; validator enforces <= these.
    max_linear_speed: float = 0.22
    max_angular_speed: float = 2.84
    geofence: Optional[List[Waypoint]] = None
    timeout_s: int = 300


class Mission(BaseModel):
    """The validated artifact the executor consumes."""
    mission_id: str
    command_type: CommandType
    robot_ids: List[str] = Field(default_factory=lambda: ["tb3_0"])
    route_name: Optional[str] = None   # for patrol_loop: name resolved from routes.yaml
    waypoints: List[Waypoint] = Field(default_factory=list)
    loops: int = 1
    formation: Optional[Formation] = None
    target: Optional[Target] = None
    constraints: Constraints = Field(default_factory=Constraints)


def mission_json_schema() -> dict:
    return Mission.model_json_schema()
