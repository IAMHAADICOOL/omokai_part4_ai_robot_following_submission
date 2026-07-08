# omokai_bringup

**The "start everything" package.** This doesn't contain pipeline logic — it holds
the launch files, the map, the routes, the robot settings, and the spawn
configuration that tie the whole system together and start it with one command.

## Its role in the pipeline
Everything else is a worker; this is the conductor. It starts the simulator,
starts Nav2 (localization + navigation), opens RViz, and launches the four
pipeline nodes (interface → planner → validator → executor) in the right order.

## What's inside

### `launch/core_pipeline.launch.py` — the main entry point
Starts the whole system, with two modes chosen by `slam:=`:
- `slam:=False` (default) — localize in the saved map and run the full pipeline.
- `slam:=True` — mapping mode (drive around to build a new map).

Key things it does:
- Reads **`config/spawn_pose.yaml`** (see below) — the single source of truth for
  where the robot starts.
- `_render_nav2_params()` — copies `nav2_params.yaml` and injects the `map_seed`
  into AMCL's `set_initial_pose`, so the robot localizes correctly at startup with
  **no manual "2D Pose Estimate"**. This happens once, at cold start (no repeated
  re-seeding that could disturb localization).
- Includes `sim.launch.py` (the simulator) with the `world_spawn` pose.
- Includes Nav2's `bringup_launch.py`, passing the `slam` flag and the seeded
  params.
- Starts RViz and, in run mode only, the four pipeline nodes — each in its own
  xterm window, after a short delay so Nav2 is ready first.

### `launch/sim.launch.py` — the simulator
Starts Gazebo with `turtlebot3_world` and spawns the robot at the `x_pose` /
`y_pose` / `yaw` passed to it (which come from `spawn_pose.yaml`).

### `launch/formation.launch.py` — Part 2 formation pipeline
Starts ONLY the pipeline nodes (interface → planner → validator →
**formation_coordinator**), each in its own xterm. The robots and per-robot Nav2
are started separately by the `tb3_multi_robot` package. This launch passes the
coordinator two things: `routes_file` (for the squad's center path) and
`robots_yaml` (the multi-robot package's `robots.yaml`, so the squad list matches
the robots that actually spawned). Run it as Terminal C in the Part 2 flow (see
the root README).

### `config/spawn_pose.yaml` — where the robot starts (single source of truth)
Holds two poses in two different frames:
- `world_spawn` — where Gazebo puts the robot (simulator frame).
- `map_seed` — that same spot in the saved-map frame → given to AMCL.

Because SLAM anchors the map's origin at the spawn point, `map_seed` is normally
`(0,0,0)` and never needs changing. Feeding both the simulator and the localizer
from one file is what stops the "robot starts in the wrong place" problem.

### `config/routes.yaml` — named routes
Maps a route name (e.g. `perimeter`) to a list of coordinates. The planner refers
to routes by name; the executor looks the name up here to get the actual points.

### `config/nav2_params.yaml` — navigation settings
All of Nav2's tuning (AMCL localization, costmaps, controller, etc.). The launch
file injects the spawn seed into this before handing it to Nav2.

### `maps/` — the saved map
Put your `turtlebot3_world.yaml` + `.pgm` here. The default demo localizes in this
map.

## Input / output
- **Input:** the `slam` and `map` launch arguments; the config files above.
- **Output:** running processes — the simulator, Nav2, RViz, and the four pipeline
  nodes. It's the thing you actually run.
