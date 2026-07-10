#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Native (no-Docker) setup for Ubuntu 24.04 + ROS 2 Jazzy.
# Covers Part 1 (core pipeline), Part 2 (formations) and Part 4 (vision follow).
#
# Part 3 (Graph SLAM) lives in a SEPARATE repository and has its own installer,
# because it needs the Stonefish simulator and GTSAM. See the README.
#
# Run from the repo root:   bash install_native.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

echo "==> [1/6] ROS 2 Jazzy apt repository"
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "==> [2/6] ROS 2 Jazzy + Gazebo Harmonic + TurtleBot3 + Nav2 + vision deps"
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-image \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  python3-opencv \
  xterm python3-pip python3-colcon-common-extensions
#  ros-jazzy-ros-gz-image  -> Part 4: bridges the camera image out of Gazebo
#  python3-opencv          -> Part 4: ArUco detection + solvePnP (the default mode)

echo "==> [3/6] Python deps"
pip3 install --break-system-packages "pydantic>=2.0" "ollama>=0.3.0"
# Part 4's OPTIONAL 'yolo' mode needs Ultralytics (pip-only). numpy is pinned <2
# because cv_bridge's compiled extension is built against NumPy 1.x's ABI --
# installing NumPy 2.x makes `import cv_bridge` fail at runtime with an
# unrelated-looking error. The default 'aruco' mode needs neither.
pip3 install --break-system-packages "numpy<2" "ultralytics>=8.0"

echo "==> [4/6] Ollama (local LLM runtime) + model"
if ! command -v ollama >/dev/null 2>&1; then
  curl -fsSL https://ollama.com/install.sh | sh
fi
(ollama serve >/dev/null 2>&1 &) || true
sleep 3
ollama pull qwen2.5:3b

echo "==> [5/6] setuptools pin, then build the workspace"
# Some setuptools versions change how `setup.py develop` honours colcon's
# requested script-install directory: `colcon build --symlink-install` then
# SUCCEEDS with no error, but installs each Python node's executable to
# install/<pkg>/bin/ instead of install/<pkg>/lib/<pkg>/ -- where `ros2 run` and
# `ros2 launch` actually look. Symptom: "package 'X' found ... but libexec
# directory '.../lib/X' does not exist".
# Ref: https://github.com/ros2/ros2_documentation/issues/4213
pip3 install --break-system-packages "setuptools==75.6.0"

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo "==> [6/6] Environment variables (added to ~/.bashrc if missing)"
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
add_line "source /opt/ros/jazzy/setup.bash"
add_line "source $(pwd)/install/setup.bash"
add_line "export TURTLEBOT3_MODEL=burger"
add_line "export LLM_MODEL=qwen2.5:3b"

echo
echo "Done. Open a NEW terminal, then see the README:"
echo "  Part 1 (single robot) : ros2 launch omokai_bringup core_pipeline.launch.py"
echo "  Part 2 (formations)   : 3 terminals -> tb3_world, tb3_nav2, formation"
echo "  Part 4 (vision follow): export TURTLEBOT3_MODEL=burger_cam, then"
echo "                          tb3_world + follow_nav2 + vision_follow + teleop"
