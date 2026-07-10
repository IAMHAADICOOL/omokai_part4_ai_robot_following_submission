# vision_follow — Part 4 (see a target, alert the operator, follow it)

One robot watches its camera, recognises a target you choose, sends the operator
a photo the instant it spots it, and then **follows it around obstacles** using
Nav2 — the same navigation stack Parts 1 and 2 use to drive robots around the map.

That last part is the thing to understand up front, because it changed partway
through building this: the first version drove the robot with a simple "turn
toward the target, drive at it" controller and had no idea a wall existed. That's
still in here as a fallback (`control_mode:=direct`), but the **default is now
`nav2`**, which turns each detection into a goal position on the map and lets
Nav2's planner route around pillars, walls, and the target robot itself.

## The two modes, and why both exist

| | `control_mode:=direct` | `control_mode:=nav2` (**default**) |
|---|---|---|
| How it drives | a P-controller writes straight to `cmd_vel` | sends goals to Nav2's `NavigateToPose` action |
| Avoids obstacles | **no** — drives straight at the target, into walls | **yes** — Nav2 plans a path around them |
| Needs a map / AMCL | no | yes — see `follow_nav2.launch.py` below |
| If the target leaves view | the node has to remember which way to spin | the goal is a *map coordinate* — still valid even with nothing in frame |
| Good for | seeing raw visual servoing with nothing else running | the real demo |

`detection_mode` is a separate choice, orthogonal to the above:

- **`aruco` (default)** — the follower looks for a printed ArUco marker on the
  target robot. A plain TurtleBot has no reliable YOLO/COCO class ("robot" isn't
  one of the 80 categories YOLO knows), so a marker is the dependable way for one
  robot to recognise another. This is also the only mode with a real distance
  estimate (see PnP below), so it's required for `control_mode:=nav2`.
- **`yolo`** — follows a normal object class from pretrained
  [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) (`person`,
  `chair`, `backpack`, …). Works in `direct` mode. Doesn't work in `nav2` mode:
  YOLO gives a bounding box, not a 3-D position, so there's nothing to place a
  map goal at.

## Files in this package

```
vision_follow/
├── vision_follow/vision_follow_node.py   the node — everything below happens here
├── launch/
│   ├── follow_nav2.launch.py             starts Nav2 for ONE robot (the follower)
│   └── vision_follow.launch.py           starts the vision_follow_node itself
└── params/
    └── burger_cam_follow_nav2_params.yaml  Nav2 params tuned to follow closely
```

Two launch files because they start two different things, on purpose: Nav2 (a
heavy stack of ~14 lifecycle nodes, two costmaps, AMCL) and the vision node (one
lightweight Python node) are independent, and you may want to restart one without
the other while tuning.

### Why `follow_nav2.launch.py` exists instead of reusing `tb3_nav2.launch.py`

`tb3_multi_robot`'s own `tb3_nav2.launch.py` starts Nav2 for **every enabled
robot** in `robots.yaml` — in this project's default setup, that's both `tb1`
(the follower) and `tb3` (the target you drive by hand). That's wasteful here:
the target doesn't plan anything, it's driven by teleop. Giving it a full Nav2
stack anyway costs real CPU for nothing, **and** its `collision_monitor` ends up
publishing to `/tb3/cmd_vel` — the same topic your teleop is trying to drive —
so the two fight each other.

`follow_nav2.launch.py` starts Nav2 for exactly one named robot
(`robot_name:=tb1`), using this package's own tuned params file instead of
`tb3_multi_robot`'s stock one, and seeds AMCL the same way `tb3_nav2.launch.py`
does — from the robot's spawn pose in `robots.yaml`, published to `/<ns>/initialpose`
once, ~10 seconds after startup (a `TimerAction`, so it fires after AMCL is
actually up and listening). **Wait for that seed to complete before doing
anything else** — until it does, there is no `map` frame, and both AMCL and the
costmaps will spam `frame does not exist` warnings. That's expected startup
noise, not a fault.

## Why it needs a tuned Nav2 params file at all

Copy of `tb3_multi_robot/params/burger_cam_nav2_params.yaml` with exactly four
values changed (diff-checked — nothing else touched):

| Parameter | Stock | Tuned | Why |
|---|---|---|---|
| `inflation_radius` (both costmaps) | `0.5` | `0.25` | See below — this is the one that matters. |
| `cost_scaling_factor` (both costmaps) | `5.0` | `8.0` | Steeper cost falloff to match the smaller radius. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | `0.25` / `0.25` | `0.12` / `0.20` | Lets Nav2 call a closer goal "reached" instead of circling it. |
| planner `tolerance` | `0.5` | `0.25` | Matches the tighter goal tolerance. |

**`inflation_radius` is a hard cutoff, not a soft suggestion.** Nav2's inflation
cost is `exp(-cost_scaling_factor * distance)` out to `inflation_radius`, then
**exactly zero** beyond it — not fading, just gone. The target robot is an
obstacle in the follower's own costmap (its laser sees it), so with the stock
`0.5 m` radius, any goal within about 0.6 m of the target's centre lands in
inflated or lethal cost and **Nav2 refuses it outright**. That's what actually
blocks close following — not the vision node, Nav2's own safety margin. Shrinking
the radius to `0.25 m` lowers that floor to roughly `0.45 m`. The visible side
effect: the costmap in RViz goes from a soft gradient wash to sharp, tight rings
around each obstacle — that's this same cutoff, not a bug (see the tradeoff
callout further down).

## The full pipeline, end to end

```
camera frame
   │
   ▼
detect the marker (cv2.aruco) ─── if not found: nothing this frame
   │
   ▼
solvePnP on the 4 corners ─── marker's (x, y, z) in the CAMERA's optical frame
   │  x = right, y = down, z = forward (OpenCV convention)
   ▼
rotate into base_link ─── ROS convention: x = forward, y = left, z = up
   │  (optical_to_body(): [x,y,z] -> [z, -x, -y], plus the camera's mount offset)
   ▼
TF: base_link -> map ─── needs AMCL localized; this is why follow_nav2 seeds AMCL
   │
   ▼
target's (x, y) in the MAP frame, cached with a timestamp
   │
   ▼
every goal_update_period seconds: the state machine (below) decides whether to
send Nav2 a new goal, hold still, or start recovering
```

## The state machine (`nav2` mode)

```
SEARCHING ──first detection──► FOLLOWING ◄──dist > resume_distance──┐
                                    │                                │
                          dist <= stop_distance                 HOLDING
                                    │                                ▲
                                    ▼                                │
                    (target goes stale, > target_lost_grace_sec)    │
                                    │                                │
                                    ▼                                │
                              RECOVERING ──target re-seen───────────┘
                                    │
                    (> target_lost_grace_sec + recovery_timeout_sec)
                                    ▼
                                  LOST
```

- **FOLLOWING** — target is fresh (seen within `target_lost_grace_sec`) and far
  enough away. Goal = a point `standoff_distance` metres short of the target,
  along the line from follower to target, facing the target.
- **HOLDING** — target is fresh and close (`dist <= stop_distance`). Cancel the
  active goal and stop. Only re-engage once the target pulls out past
  `resume_distance` — that gap between `stop_distance` and `resume_distance` is
  **hysteresis**: with a single threshold, a target hovering right at the
  boundary makes the follower start/stop/start/stop every cycle. Two thresholds
  with a gap between them stop that.
- **RECOVERING** — target hasn't been seen in over `target_lost_grace_sec`. Drive
  to where it was last seen, **facing the direction it was heading**, so the
  camera looks the way it went instead of just stopping and staring at an empty
  spot. See "Recovering with a heading" below — this is the part that actually
  fixes "target turns a corner and the follower just stands there."
- **LOST** — recovery ran for `recovery_timeout_sec` with no luck. Stop.

## Recovering with a heading, not just a position

The first version of this only remembered the target's *last position*, and it
had a real bug: the follower had usually already driven to its standoff goal —
sitting right next to where the target was — by the time the target vanished.
Re-issuing that same position as a "recovery goal" moved the robot nowhere; it
was already there. What actually helps is knowing **which way the target was
facing**, so the follower can drive to that spot and look down the corridor the
target took.

That heading comes from the ArUco marker's surface normal at the last good
frame — `cv2.Rodrigues` turns the marker's rotation vector into a direction, that
direction gets carried through the same optical → body → map chain as the
position, and the recovery goal ends up at
`target_last_position - heading × recovery_goal_backoff`, facing along
`heading`. The `backoff` matters: it stops the follower trying to drive to the
*exact* cell the target might still be occupying.

One catch, handled explicitly: a marker viewed nearly edge-on gives a
**degenerate, untrustworthy normal** — exactly the situation right as a target
turns a sharp corner and the marker face rotates out of view. `min_face_on_cosine`
gates this: below that threshold, the position update still happens, but the
heading update is skipped, and recovery falls back to driving straight toward the
target's last known position instead of trusting a noisy heading.

## Two guards that exist because they were hit for real

**Non-finite `cmd_vel` guard** (`_publish_cmd`). A near-edge-on marker can make
`solvePnP` return `NaN`. `numpy.clip(nan, lo, hi)` returns `nan` — it does **not**
sanitize it — so a single bad pose used to reach `cmd_vel` unfiltered and the
Gazebo diff-drive plugin would reject and **latch** on it, freezing the robot
mid-follow. Every value is checked with `math.isfinite()` immediately before
publish, with a hard fallback to a full stop.

**ROS clock, not wall clock.** Every timeout (`target_lost_grace_sec`,
`recovery_timeout_sec`, the alert cooldown) is measured against
`self.get_clock().now()`, not Python's `time.time()`. Under `use_sim_time` the
simulation can run slower than real time — badly so under Docker's forced
CPU rendering — and a wall-clock timeout would fire after far less *simulated*
time than intended, cutting recovery short for no visible reason.

## Method-by-method (what each piece of `vision_follow_node.py` actually does)

**`__init__`** — declares every tunable parameter (all listed below), builds
either the `direct`-mode `cmd_vel` publisher or the `nav2`-mode action client +
TF listener depending on `control_mode`, and refuses to start if the geometry
parameters contradict each other (`standoff_distance > stop_distance`, or
`stop_distance >= resume_distance` — both would make the follower oscillate).

**`camera_info_callback`** — latches the camera's intrinsic matrix once, the
first time it arrives. Nothing pose-related works before this fires; the HUD
shows `PnP:waiting camera_info` until it does.

**`image_callback`** — the per-frame entry point. Runs detection, draws the
overlay, and — this is a fixed bug, worth knowing about if you're comparing to
an older copy — captures `newly_acquired = not self.target_locked` **before**
setting `target_locked = True`. Originally that flag was read after being
flipped, so the "target newly acquired" operator alert could never fire.

**`_update_target_in_map`** — the optical → body → map chain described above.
Looks up `map → base_link` at the detection's own timestamp first (most
accurate); if TF doesn't have that exact moment yet, falls back to the latest
available transform rather than dropping the detection.

**`_follower_xy`** — the follower's own position in the map frame, straight from
TF. Used every tick to compute distance-to-target for the hysteresis check.

**`goal_timer_callback`** — fires every `goal_update_period` seconds (not on
every camera frame — Nav2 *preempts* on each new goal, so goal-per-frame at 15Hz
would mean it replans constantly and never actually drives anywhere). Decides
`_follow` vs `_recover` based on how stale the cached target is.

**`_follow`** — the hysteresis + standoff-goal logic described in the state
machine above. Only re-sends a goal if it's moved more than
`goal_update_min_dist` since the last one, to avoid needless preemption for
tiny jitter.

**`_recover`** — the heading-aware recovery logic. Sends exactly one recovery
goal (not one per tick) and waits for either a timeout or re-acquisition before
sending another.

**`_send_goal` / `_goal_response_callback` / `_cancel_goal`** — the actual
`NavigateToPose` action-client plumbing. If Nav2 **rejects** a goal (logged
loudly), it's almost always because the goal landed inside the target's
inflated footprint — see the params table above for the knob that controls that
floor.

**`_detect_aruco`** — runs marker detection, and if camera intrinsics are
available, `solvePnP` for the 3-D pose plus the marker's surface normal. Rejects
non-finite or negative-depth solutions outright (the same class of degenerate
solve that makes NaN dangerous), and separately gates the *normal* on
`min_face_on_cosine` even when the position itself is fine.

**`_detect_yolo`** — pretrained COCO inference, filtered to `target_class` above
`confidence_threshold`. Always returns `pose: None` — this is why `yolo` +
`nav2` don't combine.

**`_draw_hud`** — draws the always-on overlay on `vision/detection_image`:
target id/class, every id currently visible (not just the one being tracked —
useful for picking a marker id you didn't know), state, cached target position
and heading, distance, and (in `direct` mode) the live command being sent.

**`_direct_control` / `_direct_no_detection`** — the `direct`-mode P-controller
and its own, simpler lost-target handling (coast briefly, then blind-spin toward
the last known side, then stop). Kept deliberately separate from the `nav2`-mode
logic rather than sharing a code path, since the two modes' failure semantics are
genuinely different.

**`_publish_cmd`** — the single choke point before the wheels in `direct` mode;
see the NaN guard above.

**`_send_alert`** — writes a timestamped snapshot to `snapshot_dir`, publishes a
JSON alert (target, confidence, range, map position/heading if available) on
`vision/detection_alert`. This is the "send the operator a photo" requirement —
a real deployment would have a small bridge node subscribe to this topic and
forward it over email/Slack/webhook; that hook is deliberately left as a
separate concern rather than baked into this node.

## What it reads and produces

**In:** `camera/image_raw`, `camera/camera_info`, `vision/set_target_class`
(retarget at runtime, no restart — publish a marker id in `aruco` mode or a class
name in `yolo` mode).

**Out:** `vision/detection_image` (**open this in `rqt_image_view`** — it's
always-on, not just-on-detection, so a blank feed means check the topic name,
not "is it broken"), `vision/detection_alert`, `vision/target_locked`,
`vision/follow_goal` (nav2 mode — the goal being sent, add a `PoseStamped`
display for it in RViz if you want to see it plotted), and `cmd_vel` (direct
mode only — nav2 mode never touches it, Nav2's own controller owns the wheels,
and a second publisher would fight it).

All topic names are **relative**: the node runs inside the robot's namespace, so
`camera/image_raw` resolves to `/tb1/camera/image_raw`.

## Every tunable parameter

The table below is the set you'd realistically touch. The node also declares:
`global_frame` / `robot_base_frame` (TF frame names), `infer_every_n_frames`
(run detection on every Nth frame), `alert_cooldown_sec`, `snapshot_dir`,
`tf_timeout_sec`, `confidence_threshold` and `model_path` (yolo mode), and
`kp_angular` / `kp_linear` / `lost_target_grace_sec` / `recovery_rotation_speed`
(direct mode only). Run
`ros2 param list /tb1/vision_follow_node` for the live, authoritative list.

| Parameter | Default | Meaning |
|---|---|---|
| `control_mode` | `nav2` | `nav2` or `direct` |
| `detection_mode` | `aruco` | `aruco` or `yolo` |
| `aruco_marker_id` | `0` | marker id to follow |
| `aruco_dictionary` | `DICT_4X4_50` | marker family |
| `marker_length` | `0.08` | real marker side length (m) — scales the PnP distance |
| `min_face_on_cosine` | `0.35` | below this, the marker's heading is untrusted (position still used) |
| `target_class` | `person` | COCO class (yolo mode) |
| `device` | `cpu` | yolo inference device; `cpu` for portability, `cuda:0` if you've confirmed `torch.cuda.is_available()` |
| `standoff_distance` | `0.5` | how far behind the target to park (nav2 mode) |
| `stop_distance` | `0.6` | stop chasing once this close |
| `resume_distance` | `0.85` | resume once the target pulls back out to here — must exceed `stop_distance` |
| `goal_update_period` | `0.75` | seconds between goal updates |
| `goal_update_min_dist` | `0.25` | don't preempt Nav2 for a smaller goal move than this |
| `target_lost_grace_sec` | `5.0` | seconds with no detection before recovery starts |
| `recovery_timeout_sec` | `20.0` | how long recovery keeps trying before giving up |
| `recovery_goal_backoff` | `0.35` | stop this far short of the target's exact last position |
| `desired_distance` | `0.6` | following distance (direct mode's bbox/PnP-based control) |
| `max_linear_speed` / `max_angular_speed` | `0.15` / `0.4` | speed caps — same envelope as the fleet's teleop driver |

## Tradeoff worth knowing: tighter following vs. RViz looking "empty"

If you shrink `inflation_radius` further to follow even closer, the visible
inflation gradient around every obstacle shrinks with it — Nav2 still avoids
walls correctly (the underlying cost function is doing its job at whatever
radius you set), but the RViz costmap view goes from a soft, wide gradient wash
to sharp, thin rings hugging each obstacle. That's not a bug or a display
mismatch; it's the direct, mechanical consequence of a smaller hard cutoff.
Purely cosmetic, but worth knowing before you assume something broke.

## Packaging note: `setup.cfg`

This package ships a `setup.cfg`:
```
[develop]
script_dir=$base/lib/vision_follow
[install]
install_scripts=$base/lib/vision_follow
```
This explicitly forces `colcon`'s script install directory rather than trusting
whatever the installed `setuptools` version defaults to. It's the standard file
`ros2 pkg create --build-type ament_python` generates automatically; without it,
some `setuptools` versions install each package's console-script entry point to
`install/<pkg>/bin/` instead of `install/<pkg>/lib/<pkg>/` — the directory
`ros2 run`/`ros2 launch` actually search — so the build reports success but the
node is unlaunchable. See the root `README.md` and `Dockerfile` for the matching
`setuptools` version pin kept as a second line of defence.
