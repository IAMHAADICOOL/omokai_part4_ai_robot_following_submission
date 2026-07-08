# formation_coordinator

**The squad driver (Part 2).** This package takes one approved *squad* plan and
turns it into a separate path for each robot, then drives every robot in
formation. It's the multi-robot equivalent of `mission_executor`: the AI proposes
a single squad intent; this package deterministically works out where each robot
goes.

## Its role in the pipeline
For Part 2, the pipeline is the same up to the validator, then branches to here:
```
prompt → llm_planner → validator → (formation_sweep plan) → formation_coordinator → each robot's Nav2
```
`mission_executor` is untouched and single-robot; formation missions get their own
driver so the two never interfere.

## Input / output
- **Input:** listens on **`/mission/validated`** (from `mission_validator`), but
  only acts on plans whose `command_type` is `formation_sweep`; it ignores
  single-robot plans.
- **Config input:**
  - `routes_file` (`routes.yaml`) — to look up the squad's center path by name.
  - `robots_yaml` — the multi-robot package's `robots.yaml`, so the squad list is
    read from the SAME file that spawns the robots (they can't drift apart).
  - `robot_namespaces` — optional explicit override of the squad list.
- **Output:** sends goals to each robot's **`/<name>/follow_waypoints`** action
  (e.g. `/tb1/follow_waypoints`, `/tb3/follow_waypoints`).
- **Who uses the output:** each robot's Nav2 stack (from `tb3_multi_robot`).

## The files

### `formation_coordinator/geometry.py` — the offset math (pure, testable)
No ROS here — just functions, so the math can be unit-tested on its own.
- `offsets_for_robot(index, n_robots, type, spacing)` — returns how far robot
  `index` sits **sideways** and **front/back** from the squad's center, as
  multiples of `spacing`:
  - **line** → `lateral = index - (n_robots-1)/2` → robots spread symmetrically
    side-to-side across the direction of travel.
  - **column** → `longitudinal = -index` → single file, each robot one step behind.
  - **wedge** → robot 0 is the apex; others alternate left/right and fan out behind.
- `robot_path(center_path, index, n_robots, type, spacing)` — walks the center
  path, works out the heading at each point, and applies that robot's sideways +
  forward offset to every point, producing that robot's own path.

### `formation_coordinator/formation_coordinator.py` — the ROS node
- `__init__` — reads the squad list (`_resolve_namespaces`), loads routes, and
  creates one **FollowWaypoints action client per robot** (stored in
  `self._action_clients` — note the underscore: `clients` is a reserved name on a
  ROS node, so we can't use it).
- `_resolve_namespaces()` — the single-source-of-truth logic: if
  `robot_namespaces` is given, use it; otherwise read the enabled robots from
  `robots.yaml`; otherwise fall back to `['tb1','tb3']`. This is what lets you
  scale the squad by editing only `robots.yaml`.
- `on_validated()` — the core. On a `formation_sweep` plan it: prints an `AUDIT`
  line (`mission_id` + `sha256` + formation + robot count); gets the center path;
  then, for each robot, calls `robot_path(...)` to compute its offset path and
  sends it with `_send()`.
- `_center_path()` — resolves the plan's `route_name` (via `routes.yaml`) or
  explicit waypoints into the list of center points.
- `_pose()` — builds one Nav2 `PoseStamped` in the `map` frame.
- `_send()` — waits for that robot's Nav2 action server and sends its waypoints.
- `main()` — standard ROS start-up/shut-down wrapper.

## Why it scales without code changes
Nothing here hardcodes "2 robots": the client list, the loop, and the geometry all
take the robot count as data. Enable more robots in `robots.yaml` and the squad
grows automatically; the route file never changes, because the route is the
squad's *center* and the math offsets each robot around it.
