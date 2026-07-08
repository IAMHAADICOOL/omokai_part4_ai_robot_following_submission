# Exact dependency / version list (Part 1 + Part 2)

Target OS: **Ubuntu 24.04 LTS (Noble)**  ·  ROS 2: **Jazzy Jalisco**  ·  Gazebo: **Harmonic (gz-sim 8)**

## System / ROS packages (apt)
| Package | Purpose | Part |
|---|---|---|
| ros-jazzy-desktop | ROS 2 Jazzy + RViz2 (native install) | 1 & 2 |
| ros-jazzy-ros-gz | Gazebo Harmonic + ROS↔Gazebo bridge | 1 & 2 |
| ros-jazzy-turtlebot3, -msgs, -simulations | robot model, world, sim launch | 1 & 2 |
| ros-jazzy-navigation2, ros-jazzy-nav2-bringup | Nav2 stack (AMCL, costmaps, controllers) | 1 & 2 |
| ros-jazzy-slam-toolbox | 2D SLAM (mapping mode) | 1 & 2 |
| ros-jazzy-teleop-twist-keyboard | drive robots by keyboard (mapping / manual) | 1 & 2 |
| ros-jazzy-ros-gz-image | camera image bridge (Gazebo → ROS) for the burger_cam model | 4 |
| python3-opencv | ArUco marker detection + solvePnP pose estimation | 4 |
| xterm | each pipeline node runs in its own terminal | 1 & 2 |
| git | clone the multi-robot package | 2 |
| python3-pip, python3-colcon-common-extensions | build tooling | 1 & 2 |

## Third-party ROS package (cloned, not apt)
| Repo | Purpose | Part |
|---|---|---|
| github.com/arshadlab/tb3_multi_robot | multi-robot TurtleBot3 sim + per-robot Nav2 | 2 |

This is **vendored** (included in `src/tb3_multi_robot/`), not cloned. It's our
fork (Apache 2.0); our changes are in `src/tb3_multi_robot/OMOKAI_CHANGES.md`.

## Python packages (pip)
| Package | Version | Purpose | Part |
|---|---|---|---|
| pydantic | >= 2.0 | mission JSON schema + validation | 1 & 2 |
| ollama | >= 0.3.0 | client for the local LLM | 1 & 2 |
| numpy | < 2 | pinned for cv_bridge ABI compatibility (numpy 2.x breaks `import cv_bridge`) | 4 |
| ultralytics | >= 8.0 | YOLO detection (optional `yolo` mode only; the default `aruco` mode needs only OpenCV) | 4 |

## LLM
| Tool | Version | Notes |
|---|---|---|
| Ollama | latest | local inference server |
| Model | qwen2.5:3b | ~2 GB; runs CPU-only if no GPU |

## Environment variables
| Var | Value | Why |
|---|---|---|
| TURTLEBOT3_MODEL | burger (Parts 1 & 2) / burger_cam (Part 4) | robot model; Part 4 sets `burger_cam` to get the camera + ArUco marker |
| LLM_MODEL | qwen2.5:3b | model the planner requests |
| OLLAMA_HOST | localhost:11434 (native) / ollama:11434 (Docker) | where the planner reaches Ollama |
