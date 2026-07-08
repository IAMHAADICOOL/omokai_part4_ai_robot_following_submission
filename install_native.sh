#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Native (no-Docker) setup for Ubuntu 24.04 + ROS 2 Jazzy, for Part 1 + Part 2.
# Installs ROS 2, Gazebo Harmonic, TurtleBot3, Nav2, Ollama, the Python deps,
# clones + patches the multi-robot package, then builds. Run from the repo root:
#     bash install_native.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

echo "==> [1/7] ROS 2 Jazzy apt repository"
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "==> [2/7] ROS 2 Jazzy + Gazebo Harmonic + TurtleBot3 + Nav2"
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-ros-gz-image \
  python3-opencv \
  xterm python3-pip python3-colcon-common-extensions

echo "==> [3/7] Python deps (pydantic + ollama client + Part 4 vision)"
pip3 install --break-system-packages "pydantic>=2.0" "ollama>=0.3.0"
# Part 4 (vision): OpenCV ArUco is covered by python3-opencv (apt, below).
# Ultralytics (optional YOLO mode) is pip-only; numpy pinned <2 to stay
# ABI-compatible with cv_bridge (numpy 2.x breaks `import cv_bridge`).
pip3 install --break-system-packages "numpy<2" "ultralytics>=8.0"

echo "==> [4/7] Ollama (local LLM runtime) + model"
if ! command -v ollama >/dev/null 2>&1; then
  curl -fsSL https://ollama.com/install.sh | sh
fi
(ollama serve >/dev/null 2>&1 &) || true
sleep 3
ollama pull qwen2.5:3b

echo "==> [5/7] Multi-robot package: already vendored in src/tb3_multi_robot"
# Our fork of github.com/arshadlab/tb3_multi_robot ships INSIDE this repo
# (src/tb3_multi_robot). Nothing to clone. See src/tb3_multi_robot/OMOKAI_CHANGES.md.

echo "==> [6/7] Build the workspace"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo "==> [7/7] Environment variables (added to ~/.bashrc if missing)"
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
add_line "source /opt/ros/jazzy/setup.bash"
add_line "source $(pwd)/install/setup.bash"
add_line "export TURTLEBOT3_MODEL=burger"
add_line "export LLM_MODEL=qwen2.5:3b"

echo
echo "Done. Open a NEW terminal, then see the README 'Launching' section:"
echo "  Part 1 (single robot): ros2 launch omokai_bringup core_pipeline.launch.py"
echo "  Part 2 (formations)  : 3 terminals -> tb3_world, tb3_nav2, formation"
echo "  Part 4 (vision follow): export TURTLEBOT3_MODEL=burger_cam, then"
echo "                          tb3_world + vision_follow (see README Part 4)"
