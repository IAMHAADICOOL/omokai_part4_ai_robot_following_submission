# Omokai — Core Task + Multi-Robot Formations + Vision Follow (Part 1 + Part 2 + Part 4)
### Type an instruction in plain English → robot(s) drive themselves in a simulator

**Part 1 (core):** you type something like *"patrol the perimeter twice."* A local
AI turns that sentence into a small plan. A safety checker makes sure the plan is
allowed. Then a simple, predictable program drives the robot around the map in
Gazebo (the simulator). **The AI only suggests the plan — it never drives the robot.**

**Part 2 (multi-robot formations):** the same idea, scaled to a squad. You type
*"sweep the area in a wedge,"* and a coordination layer turns that one squad
instruction into a separate path for each robot (line / column / wedge), then
drives every robot in formation — each with its own navigation stack.

```
Part 1:  your words → AI planner → draft plan → safety checker → approved plan → driver → 1 robot
Part 2:  your words → AI planner → draft plan → safety checker → approved plan → coordinator → N robots
                      (suggests)   (JSON)       (guardrail)                      (per-robot paths)
```

Parts 1 and 2 share the core packages; Part 2 adds the formation coordinator and
the multi-robot simulation on top.

---

**Part 4 (vision AI — see a target, alert the operator, follow it):** a separate,
self-contained track. One robot watches its **camera**, recognises a target you
choose (by default an **ArUco marker** worn by another robot), **sends the
operator a snapshot** the moment it spots it, and then **follows** it — turning to
keep it centred and driving to hold a set distance.

```
Part 4:  robot's camera → detector (ArUco/YOLO) → snapshot to operator + follow the target
                          (recognise)             (alert)                (drive after it)
```

Part 4 is deliberately **standalone**: it has **no AI planner and no Nav2** in the
loop. It reads camera frames and drives the robot directly. That's why it has its
own run instructions (**Section "Running Part 4"** below) rather than using the
`prompt → planner → validator` pipeline of Parts 1 and 2. It reuses the same
multi-robot simulation, with a camera-equipped robot variant (`burger_cam`).

This repository **extends Part 1 → Part 2 → Part 4**, additively: each part is
added on top without changing what already works.

Each stage is a separate ROS 2 package. **Every package has its own README.md**
inside its folder (open the folder on GitHub to read it) that explains that piece
in detail — what it does, what it reads, and what it produces. Start here for
setup; go into a package's README to understand that piece.

---

## Getting the code

Clone the repository and move into its folder. **Every command in this guide is
run from the repository's root folder** (the one containing `docker-compose.yml`,
`Dockerfile`, and the `src/` folder) unless it says otherwise.

```bash
git clone https://github.com/IAMHAADICOOL/omokai_part4_ai_robot_following_submission.git omokai
cd omokai
```

Check you're in the right place:
```bash
ls
# you should see: Dockerfile  docker-compose.yml  install_native.sh  src  docs  README.md ...
```
Stay in this folder for everything below. The only time you go elsewhere is
*inside* the Docker box (Section 5).

---

## 0. One-time: add your saved map (for Part 1)

Part 1's demo drives around a **saved map** of the world. From the repo root, put
your two map files at these paths before you build:
```
src/omokai_bringup/maps/turtlebot3_world.yaml
src/omokai_bringup/maps/turtlebot3_world.pgm
```
Don't have a map yet? Make one — see **Section 4**. (Part 2 uses the multi-robot
package's own map, so this step is only needed for Part 1.)

---

## 1. Run it with Docker (easiest — recommended)

You don't need to know Docker. Think of Docker as a **pre-built, sealed
computer-in-a-box**: everything (ROS 2, the simulator, all the software) is
already installed inside. We start **two boxes**: one runs the AI, the other runs
the robot software. (Why two? See Section 6.)

> **⚠️ Performance note:** this Docker setup renders the simulator (Gazebo) using
> the **CPU**, not the GPU (`LIBGL_ALWAYS_SOFTWARE=1` — see Section "Supported
> platform" below for why). This makes it work on any machine regardless of GPU,
> but it also means everything — the simulation, the robots' motion, the
> formations — **can look and feel noticeably slow**, especially with two
> full Nav2 stacks and the AI running at the same time. This is expected; it is
> not a bug in the pipeline. **If you want fast, full-speed execution, use the
> native installation instead (Section 2)** — it runs directly on your machine's
> real GPU/CPU with no container overhead.

**Important difference from a "one command and it runs" setup:** here, starting
the boxes does **not** auto-launch anything. The robot box starts and *stays
ready*; you then open terminals into it and run the Part 1 or Part 2 launch
commands yourself. This is so you can choose which part to run, and so Part 2 can
use several terminals. See **Section 1d (Launching)**.

### Supported platform
Built and tested on **Linux (Ubuntu 24.04), x86_64 (amd64)**.
- **Linux + x86_64** → fully supported (intended setup).
- **Linux + ARM64** → images are multi-arch so it *may* build, but untested; expect slower CPU rendering.
- **macOS / Windows** → **not supported for the GUI** (the windows rely on Linux's X11 display; macOS/Windows handle it differently). Use a Linux machine or a Linux VM with a display.

### Disk space — this reuses Part 1's image layers

If you already built the Part 1 image on this machine, building Part 2 costs very
little extra disk. Docker builds images in **layers** and stores identical layers
**once**. Part 2's base + system-packages + Python layers are written to be
byte-identical to Part 1's, so Docker **reuses** Part 1's big ROS/Gazebo layer
instead of rebuilding or re-storing it. The only new disk Part 2 adds is its small
delta (a couple of extra tools, the multi-robot clone, and the compiled
workspace). Nothing extra to do — just build Part 1 first (or in either order),
and the shared layers are stored a single time. The image is still fully
standalone, so building it from a fresh clone (e.g. on the examiner's machine)
also works.

### 1a. Install Docker (only once, ever)
```bash
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER    # lets you run docker without sudo
# now LOG OUT and back in (so the group change takes effect)
```
If apt complains about a `containerd.io` / `containerd` conflict, you already have
a different Docker — use it, or remove the old one:
```bash
sudo apt remove -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt install -y docker.io docker-compose-v2
```

### 1b. Let the boxes open windows on your screen (once each time you log in)
```bash
xhost +local:docker
```
This lets the container show the Gazebo / RViz / terminal windows on your desktop.
Without it, the software runs but no windows appear.

**Tip:** run this once per login. To avoid re-typing it, add it to your `~/.bashrc`:
```bash
echo "xhost +local:docker >/dev/null 2>&1" >> ~/.bashrc
```

### 1c. Build and start the boxes (does NOT launch the robot software)
```bash
docker compose up -d --build
```
What this does:
- **builds** the box from the recipe (`Dockerfile`) — installs ROS 2, Gazebo,
  compiles everything (the multi-robot package is already included in src/)
- **starts** the AI box and downloads the ~2 GB AI model (first time only)
- **starts** the robot box and leaves it **running and idle**, ready for you

The `-d` means "in the background." First run is slow (building + downloading);
later runs are fast. Nothing will pop up yet — that's expected.

### 1d. Launching — open terminals into the box and run a part

Every launch happens **inside** the robot box. To open a terminal inside it:
```bash
docker compose exec ros bash
```
Run that in a new terminal window each time you need another one (Part 2 needs
three). Inside, ROS 2 and the project are already loaded.

**► Part 1 — single robot (core pipeline).** One terminal:
```bash
docker compose exec ros bash
# inside the box:
ros2 launch omokai_bringup core_pipeline.launch.py
```
Gazebo, RViz, and four small pipeline terminals appear. In the **"1 INTERFACE"**
window type, e.g., `Patrol the perimeter loop twice`.

**► Part 2 — multi-robot formations.** Three terminals into the box:
```bash
# Terminal A — the simulator (Gazebo world + the robots):
docker compose exec ros bash
ros2 launch tb3_multi_robot tb3_world.launch.py

# Terminal B — per-robot navigation (wait until both robots localize):
docker compose exec ros bash
ros2 launch tb3_multi_robot tb3_nav2.launch.py

# Terminal C — the formation pipeline:
docker compose exec ros bash
ros2 launch omokai_bringup formation.launch.py
```
Then in the **"1 INTERFACE"** window (from Terminal C) type, e.g.:
```
Sweep the area in a wedge
```
Both robots drive off in formation. Try also `Move in a line formation` and
`Drive in single file`.

**► Part 4 — vision follow (one robot follows another).** This one is different:
no typing, no AI planner. You start the simulator with the **camera robot**, start
the vision node, watch the annotated camera view, and drive the *target* robot
around by keyboard so the *follower* chases it. Four terminals into the box:

```bash
# Terminal A — simulator with the CAMERA robot variant (note the env var):
docker compose exec ros bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch tb3_multi_robot tb3_world.launch.py

# Terminal B — the vision follow node, attached to tb1 (the follower):
docker compose exec ros bash
ros2 launch vision_follow vision_follow.launch.py robot_name:=tb1 \
    detection_mode:=aruco aruco_marker_id:=0

# Terminal C — the annotated camera view (pick /tb1/vision/detection_image):
docker compose exec ros bash
ros2 run rqt_image_view rqt_image_view

# Terminal D — drive the TARGET robot (tb3) by keyboard so tb1 follows it:
docker compose exec ros bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3/cmd_vel -p stamped:=true
```

In the `rqt_image_view` window choose **`/tb1/vision/detection_image`** (not
`camera/image_raw`). You'll see the marker outlined, 3-D pose axes drawn on it,
and a text overlay (target id, ids seen, LOCKED/SEARCHING, distance in metres, and
the live drive command). Drive `tb3` around with Terminal D and `tb1` follows it;
each time `tb1` first sees the marker it saves a snapshot to
`/tmp/vision_follow_snapshots/` and prints an `[OPERATOR ALERT]` line in Terminal B.

> **Why `burger_cam`?** Parts 1 and 2 use the plain `burger` robot (no camera).
> Part 4 needs a camera and a marker, so it uses a `burger_cam` variant, selected
> only by `export TURTLEBOT3_MODEL=burger_cam` before launching the simulator.
> Set it in **every** terminal for Part 4, or set it once and reuse the same
> terminals. The default remains `burger`, so Parts 1 and 2 are unaffected.

> **⚠️ Performance note (Part 4):** in Docker, Gazebo renders on the **CPU**
> (`LIBGL_ALWAYS_SOFTWARE=1`). Camera rendering plus detection is heavier than the
> other parts, so the feed and the follow may lag. The default `aruco` mode is
> light and runs fine on CPU. The optional `yolo` mode does CPU inference and will
> be slow in Docker — **for smooth vision, use the native install (Section 2)**,
> and for YOLO on a GPU pass `device:=cuda:0` (see the `vision_follow` README).

### 1e. Stop everything
```bash
docker compose down
```
This shuts down and removes both boxes cleanly (and kills any stuck simulator).

### The handful of Docker commands you'll actually use
| Command | Plain-English meaning |
|---|---|
| `docker compose up -d --build` | build (if needed) and start the boxes, idle |
| `docker compose up -d` | start the boxes (already built — faster) |
| `docker compose exec ros bash` | **open a terminal inside** the robot box (run the launches here) |
| `docker compose logs -f ros` | watch the robot box's messages |
| `docker compose down` | stop and remove the boxes |

---

## 2. Run it without Docker (native install)

Clean **Ubuntu 24.04** machine. From the repo root:
```bash
bash install_native.sh    # installs ROS 2, Gazebo, TB3, Nav2, Ollama, then builds (tb3_multi_robot is already in src/)
# open a NEW terminal (so settings load), then run a part:
```
**Part 1:**
```bash
ros2 launch omokai_bringup core_pipeline.launch.py
```
**Part 2 (three terminals):**
```bash
ros2 launch tb3_multi_robot tb3_world.launch.py     # T1: simulator
ros2 launch tb3_multi_robot tb3_nav2.launch.py      # T2: per-robot Nav2
ros2 launch omokai_bringup formation.launch.py      # T3: formation pipeline
```

**Part 4 — vision follow (four terminals):**
```bash
export TURTLEBOT3_MODEL=burger_cam                                              # in each terminal
ros2 launch tb3_multi_robot tb3_world.launch.py                                 # T1: sim (camera robot)
ros2 launch vision_follow vision_follow.launch.py robot_name:=tb1 \
    detection_mode:=aruco aruco_marker_id:=0                                    # T2: follower's vision
ros2 run rqt_image_view rqt_image_view                                         # T3: pick /tb1/vision/detection_image
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3/cmd_vel -p stamped:=true                        # T4: drive target tb3
```
Native runs at full GPU speed, so the camera feed and follow are smooth (unlike
the CPU-rendered Docker path). Exact versions are in `DEPENDENCIES.md`.

---

## 3. What to type, and what should happen

**Part 1 (single robot):**
| You type | What happens |
|---|---|
| `Patrol the perimeter loop twice` | robot drives the perimeter route, 2 times |
| `Speed 5 m/s around the loop` | **rejected** — too fast; the VALIDATOR window says why |

**Part 2 (formations):**
| You type | What happens |
|---|---|
| `Sweep the area in a wedge` | both robots drive in a wedge along the sweep route |
| `Move in a line formation` | robots drive side-by-side |
| `Drive in single file` | robots drive one behind the other (column) |

The pipeline windows (left to right): **INTERFACE** (you type) → **LLM PLANNER**
(draft plan as JSON) → **VALIDATOR** (accepted/rejected + reason) → **EXECUTOR**
(Part 1) or **FORMATION COORDINATOR** (Part 2), which prints an `AUDIT` line and
the per-robot goals.

**Part 4 (vision follow):** there's nothing to *type* — you drive the target robot
and watch the follower react.
| You do | What happens |
|---|---|
| Drive `tb3` around (Terminal D) | `tb1` turns to keep the marker centred and follows, holding ~0.6 m |
| `tb1` first sees the marker | a snapshot is saved to `/tmp/vision_follow_snapshots/` and an `[OPERATOR ALERT]` line prints |
| Drive `tb3` out of view | `tb1` rotates toward where the marker was last seen (last-pose recovery), then stops if it can't re-find it |
| `ros2 topic pub --once /tb1/vision/set_target_class std_msgs/msg/String "{data: '3'}"` | switches the followed marker id at runtime, no restart |

The annotated view on `/tb1/vision/detection_image` shows the marker outline, its
pose axes, and a live text overlay (target, ids seen, lock state, distance,
command).

---

## 4. Make your own map (Part 1, mapping mode)
```bash
ros2 launch omokai_bringup core_pipeline.launch.py slam:=True
ros2 run turtlebot3_teleop teleop_keyboard    # drive around to build the map
ros2 run nav2_map_server map_saver_cli -f src/omokai_bringup/maps/turtlebot3_world
```
Then rebuild: `colcon build --symlink-install`. Write `slam:=True`/`slam:=False`
with a **capital** letter — Nav2 reads it as Python.

Where the (single) robot starts is set once in
`src/omokai_bringup/config/spawn_pose.yaml` — it feeds both the simulator and the
localizer so they always agree (no manual "2D Pose Estimate"). For the multi-robot
part, the equivalent single source of truth is `tb3_multi_robot`'s
`config/robots.yaml` (name + spawn pose + enabled flag per robot).

---

## 5. Working inside the Docker box

### Open another terminal inside the running robot box
```bash
docker compose exec ros bash
```
You're now inside the box; ROS 2 and the project are loaded. Type `exit` to leave
(the box keeps running). This is how you open the multiple terminals Part 2 needs.

### Add a new ROS package to the box
1. Put your package under `src/` on your computer: `src/my_new_package/`
2. With the default Docker setup, your local `src/` is bind-mounted into the
  container at `/ws/src`, so code edits appear immediately in the running box.
  For changes that affect installed entry points or launch/data files, rebuild
  the workspace once inside the container:
  ```bash
  docker compose exec ros bash
  cd /ws && colcon build --symlink-install && source install/setup.bash
  ```
3. If you add or remove packages, or change dependencies, rebuild the image
  once so the container picks up the new package set:
  ```bash
  docker compose up -d --build
  ```

---

## 6. Why two separate boxes (ROS box + Ollama box)?
- **Each box does one job.** The AI box is a ready-made Ollama image; the robot
  box only needs ROS/Gazebo. Mixing them makes a bigger, more fragile image.
- **The model is downloaded once and kept**, in the AI box's own storage (a Docker
  volume), so rebuilding the robot box doesn't wipe or re-download it.
- **They restart independently.**
- **It mirrors reality** — on a real robot the LLM often runs as its own service.

They talk over a private network Docker sets up: the robot box reaches the AI at
`http://ollama:11434` (the `OLLAMA_HOST` variable in `docker-compose.yml`).

---

## 7. How the pieces fit together (architecture)

Each item is a ROS 2 package with its **own detailed README.md** inside its folder.

**Shared core (Part 1):**
- **mission_schemas** — the shared "contract": the exact shape a valid plan must have.
- **mission_interface** — puts the sentence you type on `/mission/prompt`.
- **mission_llm_planner** — asks the local AI to turn the sentence into a draft plan. It only *suggests*.
- **mission_validator** — the guardrail: re-checks the draft against the contract **and** safety rules. Good → `/mission/validated`, bad → `/mission/rejected`.
- **mission_executor** — Part 1's driver: reads approved plans, drives one robot via Nav2, logs a `sha256` fingerprint of each plan.
- **omokai_bringup** — the "start everything" package: launch files (`core_pipeline.launch.py` for Part 1, `formation.launch.py` for Part 2), the map, routes, robot settings, spawn config.

**Multi-robot (Part 2):**
- **formation_coordinator** — the squad driver: turns one `formation_sweep` plan into a separate offset path per robot (line/column/wedge) and drives each robot's `/<name>/follow_waypoints`.
- **tb3_multi_robot** *(vendored fork, included in `src/`)* — spawns multiple namespaced TurtleBot3s in one Gazebo world, each with its own Nav2 stack. It's our fork of [github.com/arshadlab/tb3_multi_robot](https://github.com/arshadlab/tb3_multi_robot) (Apache 2.0); the changes we made are documented in `src/tb3_multi_robot/OMOKAI_CHANGES.md`, and the upstream docs are kept in `src/tb3_multi_robot/UPSTREAM_README.md`. For Part 4 it also provides the **`burger_cam`** model variant (camera + ArUco marker); see its `OMOKAI_CHANGES.md`.

**Vision follow (Part 4):**
- **vision_follow** *(standalone)* — one robot's camera → recognise a target (ArUco marker by default, or a YOLO/COCO object) → send the operator a snapshot on first sight → follow it with a visual-servo controller (turn to centre, drive to hold distance), using `solvePnP` for metric distance and last-pose recovery when the target leaves view. No AI planner and no Nav2: it publishes `cmd_vel` directly. Full detail in `src/vision_follow/README.md`.

This is why the design is safe and gradeable: for Parts 1 and 2 the AI is kept
**out of the control loop**, every plan is **schema-checked**, and the drivers are
**predictable and logged**; Part 4 keeps the same spirit — the detector only
*reports*, and a fixed, auditable control law does the driving (with a hard
non-finite-command guard before the wheels).

Sources and licenses: `docs/SOURCES.md`.
