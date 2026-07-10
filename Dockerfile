# ─────────────────────────────────────────────────────────────────────────────
# Omokai image (core pipeline + multi-robot formations + vision follow).
#   Parts 1, 2 and 4.  Part 3 (SLAM) lives in a separate repository -- see README.
#
# SPACE NOTE — this is designed to "extend" Part 1 without a second full copy:
# the base + apt + pip layers below are BYTE-IDENTICAL to Part 1's Dockerfile.
# Docker caches layers by their exact command, so if you already built the Part 1
# image on this machine, Docker REUSES those heavy layers here instead of
# rebuilding/re-storing them. The only NEW disk this image adds is the small
# Part-2 delta (teleop) and the small Part-4 delta (camera bridge + vision deps),
# plus the rebuilt workspace. Each delta is its own layer placed after the shared
# block, so building Part 4 on top of a Part 1/2 machine only rebuilds from that
# layer down. The image is still fully standalone, so an examiner can build it
# from a fresh clone.
#
# GPU: this image works both with and without GPU passthrough. See the README --
# `docker compose up` gives you CPU rendering (runs anywhere); adding
# `-f docker-compose.gpu.yml` gives you hardware-accelerated Gazebo.
#
# It does NOT auto-launch: it starts and stays alive; you open terminals into it
# (docker compose exec ros bash) and run the launch commands yourself (see README).
# ─────────────────────────────────────────────────────────────────────────────
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# ══ IDENTICAL TO PART 1 (keep byte-for-byte so Docker reuses Part 1's layers) ══
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-turtlebot3 \
      ros-jazzy-turtlebot3-msgs \
      ros-jazzy-turtlebot3-simulations \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-rviz2 \
      xterm curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
      "pydantic>=2.0" "ollama>=0.3.0"
# ══ END identical-to-Part-1 block ═════════════════════════════════════════════

# ── Part 2 extra (new, small layer): keyboard teleop for driving robots ──
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# ── Part 4 extra (new, small layer): camera bridge + vision deps ──
# Placed AFTER the shared + Part 2 layers so it doesn't disturb the layers
# above: anyone who already built Part 1/2 on this machine only rebuilds from
# here down.
#   - ros-jazzy-ros-gz-image: the image_bridge that carries the camera feed
#     from Gazebo into ROS (only spawned for the camera-equipped burger_cam
#     model; see src/tb3_multi_robot/OMOKAI_CHANGES.md).
#   - python3-opencv: ArUco marker detection + solvePnP pose estimation --
#     this is the DEFAULT vision mode and needs nothing else.
#   - ultralytics + numpy<2: the OPTIONAL 'yolo' detection mode only. numpy is
#     pinned <2 because cv_bridge's compiled extension is built against NumPy
#     1.x's ABI; installing NumPy 2.x makes `import cv_bridge` crash at
#     runtime with an unrelated-looking ImportError.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-image \
      python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
      "numpy<2" "ultralytics>=8.0"

# ── setuptools pin (found while building Part 4; the failure mode is general,
# not Part-4-specific -- it would hit Part 1/2 too the next time any pip layer
# nudges setuptools into the broken range). Some setuptools versions change how
# `setup.py develop` honours colcon's requested script-install directory:
# `colcon build --symlink-install` then SUCCEEDS with no error, but installs
# every ament_python package's console_scripts entry point to
# install/<pkg>/bin/ instead of install/<pkg>/lib/<pkg>/ -- the directory
# `ros2 run`/`ros2 launch` actually look in. Symptom if this regresses: "package
# 'X' found... but libexec directory '.../lib/X' does not exist". Confirmed
# working version pinned here, deliberately as the LAST pip install before the
# build that needs it, so no later dependency resolution can silently move it.
# Ref: https://github.com/ros2/ros2_documentation/issues/4213
RUN pip3 install --no-cache-dir --break-system-packages "setuptools==75.6.0"

WORKDIR /ws
# All packages, INCLUDING our vendored tb3_multi_robot (our fork of
# github.com/arshadlab/tb3_multi_robot -- see src/tb3_multi_robot/OMOKAI_CHANGES.md).
COPY src/ /ws/src/

# Build the whole workspace.
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Make every interactive shell (incl. `docker compose exec ros bash`) have ROS 2
# and the workspace sourced automatically. The entrypoint only sources for the
# main process; exec'd shells don't run it, so we add it to root's .bashrc too.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash"   >> /root/.bashrc

# Runtime environment.
ENV TURTLEBOT3_MODEL=burger \
    LLM_MODEL=qwen2.5:3b \
    OLLAMA_HOST=http://ollama:11434 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Keep the container alive so you exec terminals in and launch manually.
CMD ["sleep", "infinity"]
