# vision_follow — Part 4 (see a target, alert the operator, follow it)

This package is **Part 4**. One robot uses its camera to recognise a target you
choose, sends the operator a photo the instant it spots it, and then drives after
it — turning to keep the target centred and moving to hold a set following
distance.

It is **standalone**: unlike Part 1 and Part 2, there is **no AI planner and no
Nav2** in the loop here. This node reads camera frames and publishes velocity
commands straight to the robot. That keeps the vision demo simple to run and easy
to reason about, and it's why Part 4 has its own launch command rather than
plugging into the `prompt → planner → validator` pipeline.

## Where it sits

```
camera image  ─►  vision_follow_node  ─►  /<robot>/cmd_vel        (drive the robot)
(/<robot>/         │  detect target       /<robot>/vision/detection_image  (annotated view)
 camera/image_raw) │  estimate its pose    /<robot>/vision/detection_alert  (operator photo + JSON)
                   │  decide a velocity     /<robot>/vision/target_locked    (true/false)
camera_info  ──────┘
(/<robot>/camera/camera_info)
```

There are **two detection modes**, chosen with the `detection_mode` launch
argument:

- **`aruco` (default)** — follows another robot that carries a printed **ArUco
  marker** (a black-and-white fiducial square). This is the intended
  robot-follows-robot demo. A plain TurtleBot has no reliable YOLO/COCO class, so
  a marker is the dependable way for one robot to recognise another.
- **`yolo`** — follows a normal object class from a pretrained
  [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) model
  (`person`, `chair`, `backpack`, …). Useful for "follow the person"; not used for
  the robot-follows-robot demo because a TurtleBot isn't a COCO class.

## What it reads (input)

| Topic | Type | Published by |
|---|---|---|
| `camera/image_raw` | `sensor_msgs/Image` | the `image_bridge` node in `tb3_multi_robot`'s `tb3_world.launch.py` (only present when `TURTLEBOT3_MODEL=burger_cam`) |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | the `parameter_bridge` in the same launch — carries the camera intrinsics used for pose estimation |
| `vision/set_target_class` | `std_msgs/String` | you (or any node) — changes the target at runtime without a restart |

Topic names are **relative**: the node is launched inside a robot namespace
(e.g. `tb1`), so `camera/image_raw` resolves to `/tb1/camera/image_raw`, exactly
like the other per-robot nodes in this project.

## What it produces (output)

| Topic | Type | Meaning |
|---|---|---|
| `cmd_vel` | `geometry_msgs/TwistStamped` | the drive command (turn + forward speed) |
| `vision/detection_image` | `sensor_msgs/Image` | the camera view with the marker outline, 3-D pose axes, and a text overlay (target id, seen ids, lock state, distance, current command). **View this topic in `rqt_image_view`.** |
| `vision/detection_alert` | `std_msgs/String` | a JSON alert (target, confidence, range, snapshot file path) published the moment a target is acquired — the "send the operator a picture" step. A real deployment would have a small bridge subscribe here and forward it over email/Slack/webhook. |
| `vision/target_locked` | `std_msgs/Bool` | whether a target is currently being tracked |

Snapshots are also written to disk (default `/tmp/vision_follow_snapshots/`).

## How the important parts work (plain language)

**`camera_info_callback`** — latches the camera intrinsics once. Without them the
node can only steer by pixel offset; with them it runs real pose estimation.

**`_detect_aruco`** — finds every ArUco marker in the frame, keeps the one whose
id matches the target, and draws its outline. If intrinsics are available it runs
**`cv2.solvePnP`** on the marker's four corners to recover the marker's full 3-D
position relative to the camera: `x` (how far left/right, in metres) and `z` (how
far ahead, in metres). Those two numbers are what make the following *and* the
recovery smart rather than guesswork.

**Degenerate-pose guard** — when the marker is viewed nearly edge-on, `solvePnP`
can return a nonsense or `NaN` pose. Any non-finite or negative-depth solution is
rejected on the spot so it never reaches the control law.

**`_handle_detection`** — the follow controller. It turns to bring the target to
the centre of the image, and (with pose) drives forward/back to hold
`desired_distance` metres. On first sighting (and periodically after) it fires the
operator alert.

**`_handle_no_detection` / `_recovery_cmd`** — what to do when the target is lost.
It briefly coasts (to ride out a single dropped frame), then recovers using the
**last known pose**: it rotates toward the side the target was last on (the sign
of `x`), and if the target was already far away when lost (large `z`), it also
creeps forward. This is why a sharp turn by the target is recoverable — the node
remembers which way it went instead of spinning blindly.

**`_publish_cmd`** — the single choke point before the wheels. It blocks any
non-finite command outright (a `NaN` reaching `cmd_vel` makes Gazebo's diff-drive
plugin reject and latch, freezing the robot), substituting a safe stop.

**`set_target_callback`** — retargeting at runtime. Publish a marker id (aruco
mode) or a class name (yolo mode) to `vision/set_target_class` and it switches
targets without a restart. This is the seam where a higher-level planner could
plug in later, but Part 4 does not require it.

## Configurable target (the challenge's "target type should be user-configurable")

- At launch: `aruco_marker_id:=3` or `target_class:=person`.
- At runtime, no restart:
  ```bash
  ros2 topic pub --once /tb1/vision/set_target_class std_msgs/msg/String "{data: '3'}"       # aruco id
  ros2 topic pub --once /tb1/vision/set_target_class std_msgs/msg/String "{data: 'person'}"  # yolo class
  ```

## Key parameters

| Parameter | Default | Meaning |
|---|---|---|
| `detection_mode` | `aruco` | `aruco` or `yolo` |
| `aruco_marker_id` | `0` | marker id to follow (aruco mode) |
| `aruco_dictionary` | `DICT_4X4_50` | marker family |
| `marker_length` | `0.08` | real marker side length in metres — scales the distance estimate |
| `target_class` | `person` | COCO class to follow (yolo mode) |
| `device` | `cpu` | `cpu` or `cuda:0` (yolo inference; `cpu` for portability) |
| `desired_distance` | `0.6` | following distance in metres (aruco/pnp) |
| `max_linear_speed` / `max_angular_speed` | `0.15` / `0.4` | speed caps (same envelope as the fleet's teleop driver) |

See the root **README.md** for the exact run commands, and
`src/tb3_multi_robot/OMOKAI_CHANGES.md` for the camera + marker additions to the
robot model that this package relies on.
