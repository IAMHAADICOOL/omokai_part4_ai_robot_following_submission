# mission_schemas

**The contract.** This package defines the exact shape a mission (a robot plan)
must have. It contains no robot logic and no ROS node — it's a pure data
definition that every other package imports so they all speak the same language.

## Its role in the pipeline
Think of it as the "form" everyone fills out and checks against:
- The **LLM planner** turns this form into instructions for the AI, so the AI's
  answer already has the right shape.
- The **validator** re-checks a plan against this form.
- The **executor** reads a plan through this form to know what to drive.

If this one file changes, the whole pipeline's vocabulary changes with it — that's
why it's the single source of truth.

## Input / output
- **Input:** none at runtime (it's a definition, not a running program).
- **Output:** a Python class `Mission` and a function that turns it into a JSON
  schema. Imported by `mission_llm_planner`, `mission_validator`, `mission_executor`.

## The file: `mission_schemas/schema.py`

It uses **Pydantic**, a library that both *describes* data and *checks* it.

- `CommandType` — the allowed kinds of mission, as fixed strings:
  `patrol_loop`, `waypoint_nav`, `formation_sweep`, `explore_slam`,
  `follow_target`. Only the first two matter for the core task.
- `Waypoint` — a point the robot can go to: `x`, `y`, and a facing angle `yaw`.
- `Constraints` — safety ceilings. `max_linear_speed: 0.22` and
  `max_angular_speed: 2.84` are the TurtleBot3 Burger's real physical limits;
  the validator refuses anything faster.
- `Mission` — the whole plan, tying it together:
  - `mission_id` — a unique label for this plan (used in the audit log).
  - `command_type` — which kind of mission (from `CommandType`).
  - `route_name` — for a patrol, the name of a route (e.g. `"perimeter"`) that
    gets looked up later in `routes.yaml`.
  - `waypoints`, `loops`, `formation`, `target`, `constraints` — the rest of the
    plan's details.
- `mission_json_schema()` — returns the `Mission` shape as a JSON schema
  dictionary. The planner hands this to the AI so the AI's output is forced into
  this exact structure.

## Why safety checks are NOT here
This file only says *what a plan looks like*. Whether a plan is *safe or sensible*
(speed within limits, route exists) is decided by `mission_validator`. Keeping
"shape" and "rules" separate keeps each piece simple.
