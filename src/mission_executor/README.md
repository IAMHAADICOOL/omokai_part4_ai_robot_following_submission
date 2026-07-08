# mission_executor

**The driver.** This package takes an **approved** plan and actually moves the
robot. It contains no AI and no randomness: the same approved plan always produces
the exact same robot behavior. It also logs a fingerprint of every plan it runs,
so the system is auditable.

## Its role in the pipeline
It's the last stage. It turns an approved `Mission` into a concrete list of goal
points and hands them to Nav2 (the navigation stack) to drive there, one point at
a time.

## Input / output
- **Input:** listens on **`/mission/validated`** (from `mission_validator`).
- **Config input:** a `routes_file` parameter (`routes.yaml`) to turn a route
  *name* like `"perimeter"` into actual coordinates.
- **Output:** sends navigation goals to Nav2 via the **`follow_waypoints`** action.
  Also prints an `AUDIT` log line for each mission.
- **Who uses the output:** Nav2 / Gazebo (the robot drives). Nothing downstream in
  our own pipeline — this is the end of the line.

## The file: `mission_executor/executor.py`

- `yaw_to_quat()` — converts a facing angle into the quaternion form Nav2 expects
  for orientation. (A small math helper.)
- `__init__` — loads routes, subscribes to `/mission/validated`, and creates an
  **action client** for `follow_waypoints` (an action is a long-running command
  Nav2 accepts and reports progress on).
- `on_validated()` — the heart of it. On each approved plan it:
  1. computes `sha256(json)` — a short **fingerprint** — and prints the `AUDIT`
     line (`mission_id`, fingerprint, command, number of points). This is what
     makes runs traceable and reproducible.
  2. calls `_resolve_poses()` to get the actual list of points.
  3. waits for the Nav2 server, then sends the points as one goal.
- `_resolve_poses()` — turns the plan into points. For a `patrol_loop`, it looks
  the `route_name` up in `routes.yaml` and repeats that loop `loops` times. The
  line `seq = one_loop * max(1, m.loops)` is what makes "patrol twice" actually go
  around twice.
- `_pose()` — builds one Nav2 `PoseStamped` (position + orientation) in the `map`
  frame.
- `_on_feedback()` / `_on_goal_response()` / `_on_result()` — progress callbacks:
  they print which waypoint the robot is heading to, whether Nav2 accepted the
  goal, and a completion message at the end.
- `main()` — standard ROS start-up/shut-down wrapper.

## One deliberate design choice: FollowWaypoints
It uses Nav2's **FollowWaypoints** (visit each point as its own goal) rather than
NavigateThroughPoses (one continuous path). The reason is in the file's top
comment: with a single continuous path, re-issuing "patrol twice" right after
finishing — when the robot is already sitting on the final point — makes Nav2
report "done" instantly without moving. FollowWaypoints checks each point
individually, so the loop is always actually driven.
