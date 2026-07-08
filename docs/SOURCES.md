# Cited Sources (Part 1 + Part 2 + Part 4)

Every external repo/tool this repository builds on, with license and exactly what
we take (Condition 4.2: cite every source).

| Component | Repo / URL | License | What we use | Part |
|---|---|---|---|---|
| Sim base + robot | ROBOTIS-GIT/turtlebot3_simulations (jazzy) | Apache-2.0 | robot model, worlds, ros_gz launch | 1 |
| TB3 core pkgs | ROBOTIS-GIT/turtlebot3 | Apache-2.0 | description, bringup | 1 |
| Navigation | ros-navigation/navigation2 (Nav2) | Apache-2.0 | FollowWaypoints, costmaps, AMCL, bringup | 1 & 2 |
| LLM runtime | ollama/ollama | MIT | local inference + structured outputs | 1 & 2 |
| Architecture ref | Auromix/ROS-LLM | Apache-2.0 | NL->ROS node pattern (reference only) | 1 |
| Architecture ref | Gaurang-1402/ChatDrones | MIT | ROSGPT-style JSON-emit pattern (reference only) | 1 |
| Multi-robot sim | arshadlab/tb3_multi_robot | Apache-2.0 | namespaced multi-TB3 + per-robot Nav2 (vendored fork in src/tb3_multi_robot; changes in OMOKAI_CHANGES.md) | 2 |
| Formation geometry | Original (this repo) | n/a | line/column/wedge offset math in formation_coordinator/geometry.py, written from scratch, unit-tested | 2 |
| Vision detection | opencv/opencv (cv2.aruco + solvePnP) | Apache-2.0 | ArUco marker detection + PnP pose estimation (public API only) | 4 |
| Object detection | ultralytics/ultralytics (YOLO) | AGPL-3.0 | pretrained COCO inference, public `YOLO(...)` API only, in the optional `yolo` mode | 4 |
| Vision pipeline ref | monemati/PX4-ROS2-Gazebo-YOLOv8 | (see repo) | reference only for the "sim camera → detector → act on detection" shape; no code reused (it targets a PX4 drone) | 4 |
| Follow controller | Original (this repo) | n/a | ArUco/PnP visual-servo follow + last-pose recovery in vision_follow, written from scratch | 4 |

## Our changes to third-party code
`arshadlab/tb3_multi_robot` (Apache 2.0, by Arshad Mehmood) is **vendored** in
`src/tb3_multi_robot/` — included directly, not cloned, so the build needs no
network and uses our exact tested version. The upstream README is preserved as
`UPSTREAM_README.md`; our changes are documented in `OMOKAI_CHANGES.md`. In
summary: `model.sdf` odom rate 5 → 30 Hz; per-robot AMCL auto-seed added to
`tb3_nav2.launch.py`; `tb3_world.launch.py` adapted for ROS 2 Jazzy / Gazebo
Harmonic.

## Why no repo from the task's "Multi-agent / swarm" table
The task's listed swarm repos (PX4_Swarm_Controller, px4_multi_drone_sim,
gym-pybullet-drones, Crazyswarm2, mavsdk_drone_show) are all PX4/MAVLink/Crazyflie
(drone) stacks. This project is a ground robot (TurtleBot3 + Nav2), and the task
requires each challenge to extend the core task rather than run a separate stack.
The formation geometry is the same *idea* PX4_Swarm_Controller describes
("configurable formation geometry") but is independently written for Nav2's
FollowWaypoints action — no line of that repo's code was read or reused.

## Part 4 (vision): what we used, and what we deliberately did not

The task's "Vision AI" table lists `ultralytics/ultralytics`,
`monemati/PX4-ROS2-Gazebo-YOLOv8`, and
`Autonomous-Drone-Navigation-and-Human-Search`.

- **Ultralytics YOLO** — used directly (public inference API) but only in the
  optional `yolo` mode. It is **not** used for the main robot-follows-robot demo:
  a plain TurtleBot is not a COCO class, so YOLO cannot reliably recognise one
  robot from another. For that we use an **ArUco marker + OpenCV**, which is
  deterministic, needs no training data, and additionally gives a metric 3-D pose
  (via `solvePnP`) that drives both the follow distance and the lost-target
  recovery.
- **PX4-ROS2-Gazebo-YOLOv8** and **Autonomous-Drone-Navigation-and-Human-Search**
  — both are **PX4/drone** stacks. As with the swarm table, this project is a
  ground robot, and each challenge must extend the ground-robot core task rather
  than run a separate flight stack. They were read only as references for the
  general "camera → detector → act on detection" shape; **no code was copied**.
  The `vision_follow` node (detection, PnP pose, follow control, recovery) is
  written from scratch.
