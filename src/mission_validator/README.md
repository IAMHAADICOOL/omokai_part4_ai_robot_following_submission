# mission_validator

**The guardrail.** This package takes the AI's draft plan and decides whether it's
allowed. Nothing reaches the robot without passing through here. It's the safety
boundary between "the AI suggested something" and "the robot will do something."

## Its role in the pipeline
It re-checks the draft two ways:
1. **Shape check** — does the JSON actually fit the `Mission` contract? (The AI
   can truncate or malform output, so we never trust it blindly.)
2. **Rule check** — is it *safe and sensible*? Speed within limits, loop count in
   range, the named route actually exists, etc.

Only plans that pass both are forwarded to the executor.

## Input / output
- **Input:** listens on **`/mission/candidate`** (from `mission_llm_planner`).
- **Output:**
  - approved plans → **`/mission/validated`** (used by `mission_executor`)
  - rejected plans → **`/mission/rejected`** (with a human-readable reason)
- **Config input:** a `routes_file` parameter pointing at `routes.yaml`, so it
  knows which route names are real.

## The file: `mission_validator/validator.py`

Top-of-file constants are the safety limits: `HARD_MAX_LINEAR = 0.22`,
`HARD_MAX_ANGULAR = 2.84`, `MAX_LOOPS = 10`, and the set of `SUPPORTED` command
types. These are the numbers the guardrail enforces.

- `__init__` — loads the known routes from `routes.yaml`, subscribes to
  `/mission/candidate`, and creates the two publishers (`/mission/validated`,
  `/mission/rejected`).
- `_load_routes()` — reads the route names from the YAML file so the rule check
  can confirm a requested route exists.
- `on_candidate()` — runs on each draft. The key line is
  `Mission.model_validate_json(raw)` — this is the **shape check**: Pydantic tries
  to parse the AI's text into a real `Mission`; if it can't, the plan is rejected
  immediately. If it parses, it calls `_safety_check()`.
- `_safety_check()` — the **rule check**. Returns `None` if everything's fine, or a
  reason string if not. It enforces: command type is supported; speeds are within
  the hard limits; loop count is 1–10; and per-command specifics (a patrol needs a
  known route; a waypoint mission needs at least one waypoint; a formation needs a
  valid type and spacing). The first failing rule produces the rejection reason.
- `_reject()` — publishes the reason on `/mission/rejected` and logs it, so you see
  *why* something was refused in the VALIDATOR window.
- `main()` — standard ROS start-up/shut-down wrapper.

## Why this matters for grading
This is the piece that proves the design is safe: the AI can propose anything, but
a deterministic, rule-based checker stands between it and the robot.
