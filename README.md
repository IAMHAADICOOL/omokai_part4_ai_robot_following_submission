# Omokai — Core Task + Multi-Robot Formations + Vision Follow (Parts 1, 2, 4)
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

**Part 4 (vision AI — see a target, alert the operator, follow it):** no typing.
One robot's camera recognises a target — by default an **ArUco marker** worn by
another robot — **sends the operator a snapshot** the instant it spots it, then
**follows it using Nav2**, planning around obstacles rather than driving straight
at it.

```
Part 4:  camera → detect marker → snapshot to operator → map-frame goal → Nav2 drives around obstacles
                  (ArUco+solvePnP)  (alert)              (standoff dist)   (plans + avoids)
```

> ### Part 3 (SLAM) lives in a separate repository
> Part 3 builds a **line-feature Graph SLAM** system: the robot drives an unknown
> circuit, turns LiDAR points into straight wall segments, and uses them to
> correct its own drifting odometry — mapping and localizing at the same time.
>
> It is split out because it uses a **different simulator** (Stonefish, not
> Gazebo) and a **different solver** (GTSAM), neither of which the other three
> parts need. Keeping them apart means you don't have to install a marine-robotics
> physics engine just to run the LLM pipeline.
>
> **→ [Part 3: Graph SLAM with line features](https://github.com/IAMHAADICOOL/omokai_part3_slam)**


Each stage is a separate ROS 2 package. **Every package has its own README.md**
inside its folder that explains that piece in detail — what it does, what it
reads, what it produces. Start here for setup; go into a package's README to
understand that piece.

---

## 0. Getting the code

```bash
git clone https://github.com/IAMHAADICOOL/omokai_part4_ai_robot_following_submission.git omokai_1_2_4
cd omokai_1_2_4
```

**Every command in this guide is run from the repository root**, unless it says
otherwise. If a command fails with "no such file," check you're in the folder
that contains `docker-compose.yml`.

---

## 1. Run with Docker (recommended)

**Supported:** Linux, x86_64. Tested there.
**Untested:** ARM64 (Apple Silicon, Raspberry Pi).
**Not supported:** macOS and Windows — the GUI relies on X11 forwarding through
`/tmp/.X11-unix`, which those platforms don't provide natively.

### 1a. Decide: GPU or no GPU?

This is the one choice you need to make up front, and it changes only *which
command you type* — not the code, not the image contents.

| | Without GPU | With GPU (recommended) |
|---|---|---|
| Command | `docker compose up -d --build` | `docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d --build` |
| Renders on | the CPU (Mesa llvmpipe) | your NVIDIA card |
| Works on | **any machine** | machines with an NVIDIA GPU + Container Toolkit |
| Speed | slow — Gazebo, Nav2 and the robots all visibly lag | full speed |
| Setup needed | none | see 1b below |

The two compose files stack: the base one is always used, and adding
`-f docker-compose.gpu.yml` **layers changes on top of it**. That's what the
repeated `-f` flags mean — Docker Compose merges them left to right, so the GPU
file only has to state what differs (turn software rendering off, expose the
GPU) rather than duplicate the whole service.

Everything below works either way. **All of Parts 1, 2 and 4 run with GPU
support** — the GPU file is not Part-specific.

### 1b. One-time GPU setup (skip if you're not using the GPU)

You need three things on the **host machine**, not inside the container:

1. **An NVIDIA GPU with the proprietary driver.** Verify:
   ```bash
   nvidia-smi
   ```
   If that prints a table of your GPU, you're good. If it says "command not
   found," install the driver first.

2. **The NVIDIA Container Toolkit** — this is what lets Docker hand a GPU to a
   container at all. Follow NVIDIA's official guide:

   **→ https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html**

   After installing it, **restart the Docker daemon** (`sudo systemctl restart docker`),
   otherwise Docker won't pick up the new runtime.

3. Confirm Docker can actually see the GPU:
   ```bash
   docker run --rm --gpus all ubuntu nvidia-smi
   ```
   If this prints the same GPU table, passthrough works. If it errors, fix that
   before going further — nothing downstream will work until it does.

### 1c. Let the container draw on your screen

```bash
xhost +local:docker
```

This grants the container permission to open windows on your X display. It
**resets on every logout**. If Gazebo or RViz silently fails to appear, this is
almost always why. To avoid retyping it:
```bash
echo "xhost +local:docker >/dev/null 2>&1" >> ~/.bashrc
```

### 1d. Build and start

**Without GPU:**
```bash
docker compose up -d --build
```

**With GPU:**
```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d --build
```

What the flags do:
- `-f <file>` — which compose file(s) to use. Repeating it merges them in order.
- `up` — create and start the containers.
- `-d` — detached: run in the background and give you your terminal back.
- `--build` — (re)build the image first if the Dockerfile or `src/` changed.

> **Disk space.** The heavy `apt` and `pip` layers in the Dockerfile are shared
> across parts, so if you've built this image before, Docker reuses those layers
> rather than re-downloading them. Only the small per-part deltas rebuild.

This starts **two containers**: `ollama` (the local language model) and `ros`
(everything else). Neither launches any robot software — the `ros` container just
stays alive, waiting for you.

### 1e. Open a terminal inside

Every launch command below runs *inside* the container. Each time you need
another terminal, open another one this way:

```bash
docker compose exec ros bash
```

- `exec` — run a command in an already-running container.
- `ros` — which service (the one from `docker-compose.yml`).
- `bash` — what to run: an interactive shell.

ROS is already sourced inside that shell (the Dockerfile appends the `source`
lines to `/root/.bashrc`), so `ros2` just works.

---

## 2. Running Part 1 — core pipeline (1 terminal)

```bash
ros2 launch omokai_bringup core_pipeline.launch.py
```

Several `xterm` windows open, one per pipeline stage — **INTERFACE** (where you
type), **LLM PLANNER** (shows the draft plan as JSON), **VALIDATOR** (accepted /
rejected, with the reason), **EXECUTOR** (prints an `AUDIT` line and drives).

Type into the **INTERFACE** window:
```
Patrol the perimeter twice
```

| You type | What happens |
|---|---|
| `Patrol the perimeter twice` | robot drives the perimeter route, twice |
| `Drive to the inspection point` | single-waypoint navigation |
| `Fly to 500 metres` | **rejected** by the validator — outside allowed bounds. This is the guardrail working, not a bug. |

### 2a. Changing the perimeter route for your own map

The perimeter loop is just a list of coordinates in
`src/omokai_bringup/config/routes.yaml`. To get coordinates for a *different*
map, read them straight out of RViz:

**Step 1 — publish a point in RViz.**
1. In RViz's top toolbar, click the **Publish Point** button.
2. Click anywhere in the 3D view to select that point.

**Step 2 — read it in a terminal.** Every time you click, RViz publishes a
`geometry_msgs/PointStamped` message on the topic `/clicked_point`. In another
terminal (inside the container):

```bash
ros2 topic echo /tb1/clicked_point
```

**Step 3 — click points in RViz and watch them appear.** Each click prints:

```yaml
header:
  stamp:
    sec: 1698745200
    nanosec: 123456789
  frame_id: "map"
point:
  x: 2.34
  y: -1.05
  z: 0.0
```

Take the `x` and `y` from each click and paste them into `routes.yaml` in the
order you want the robot to visit them. The `z` is always `0.0` for a ground
robot, and `frame_id: "map"` confirms the coordinates are in the map frame —
which is exactly what the executor expects.

> **If `ros2 topic echo` says "command not found"**, you haven't sourced ROS in
> that terminal. Inside the container it's automatic; on a native install run
> `source /opt/ros/jazzy/setup.bash` first.

> Rebuild after editing (`colcon build --packages-select omokai_bringup`) or,
> because `src/` is live-mounted and the build uses `--symlink-install`, just
> relaunch — config files are picked up without a rebuild.

---

## 3. Running Part 2 — multi-robot formations (3 terminals)

```bash
# Terminal A — the simulator, both robots
ros2 launch tb3_multi_robot tb3_world.launch.py

# Terminal B — a full Nav2 stack for each robot
ros2 launch tb3_multi_robot tb3_nav2.launch.py

# Terminal C — the prompt → planner → validator → coordinator pipeline
ros2 launch omokai_bringup formation.launch.py
```

Wait for Terminal B to settle (Nav2 seeds AMCL a few seconds in — until it does,
you'll see `frame does not exist` warnings; that's normal startup noise, not a
fault). Then type into the **INTERFACE** window from Terminal C:

```
Sweep the area in a wedge
```

| You type | What happens |
|---|---|
| `Sweep the area in a wedge` | both robots drive in a wedge along the sweep route |
| `Move in a line formation` | robots drive side-by-side |
| `Drive in single file` | robots drive one behind the other (column) |

---

## 4. Running Part 4 — vision follow (5 terminals)

Here `tb1` is the **follower** (it has the camera) and `tb3` is the **target**
(it wears the ArUco marker, and you drive it by hand).

```bash
# Terminal A — simulator, with the CAMERA robot variant
export TURTLEBOT3_MODEL=burger_cam
ros2 launch tb3_multi_robot tb3_world.launch.py

# Terminal B — Nav2 for the FOLLOWER ONLY
export TURTLEBOT3_MODEL=burger_cam
ros2 launch vision_follow follow_nav2.launch.py robot_name:=tb1

# Terminal C — the vision + follow logic
export TURTLEBOT3_MODEL=burger_cam
ros2 launch vision_follow vision_follow.launch.py robot_name:=tb1

# Terminal D — the annotated camera view
ros2 run rqt_image_view rqt_image_view
#   then pick  /tb1/vision/detection_image  from the dropdown
#   (NOT camera/image_raw -- that's the raw, undecorated feed)

# Terminal E — drive the TARGET robot by keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3/cmd_vel -p stamped:=true
```

What the arguments mean:
- `export TURTLEBOT3_MODEL=burger_cam` — selects the camera+marker robot variant.
  Parts 1 and 2 use the plain `burger`; this leaves them untouched. **Set it in
  every Part 4 terminal that launches something.**
- `robot_name:=tb1` — which robot to attach Nav2 / the vision node to.
- `-r cmd_vel:=/tb3/cmd_vel` — *remaps* teleop's output topic so it drives `tb3`
  (the target) rather than the default `/cmd_vel`.
- `-p stamped:=true` — publish `TwistStamped` instead of `Twist`; the Gazebo
  bridge in this project expects the stamped form.

> **Wait for Terminal B before starting C.** Nav2 can't plan until AMCL is
> localized, which requires a `map` frame to exist. `follow_nav2.launch.py` seeds
> AMCL automatically ~10 s after startup and **logs a line when it does**
> (`[follow_nav2] seeding tb1 AMCL...`). Until then, Terminal B repeats
> `Timed out waiting for transform ... frame does not exist`. That's expected —
> give it time rather than restarting.

> **Why a separate Nav2 launch?** `tb3_nav2.launch.py` (from Part 2) starts Nav2
> for *every* enabled robot. Here that's wasteful — the target is driven by
> teleop and plans nothing — and actively harmful, because the target's
> `collision_monitor` would publish to its own `cmd_vel` and fight your teleop.
> `follow_nav2.launch.py` starts one stack, for the follower, with parameters
> tuned to let it follow more closely. Details in `src/vision_follow/README.md`.

**Nothing to type — you drive and watch:**

| You do | What happens |
|---|---|
| Drive `tb3` around (Terminal E) | `tb1` turns to keep the marker centred, follows it, planning around obstacles, holding ~0.5–0.6 m |
| `tb1` first sees the marker | a snapshot is saved to `/tmp/vision_follow_snapshots/`, and Terminal C prints an `[OPERATOR ALERT]` line |
| Drive `tb3` sharply out of view | `tb1` drives to where it was last seen, **facing the direction it was heading**, then stops if it still can't re-find it |
| `ros2 topic pub --once /tb1/vision/set_target_class std_msgs/msg/String "{data: '3'}"` | switches the followed marker id at runtime, no restart |

---

## 5. Stop everything

```bash
# without GPU
docker compose down

# with GPU (same files you started with)
docker compose -f docker-compose.yml -f docker-compose.gpu.yml down
```

`down` stops and removes the containers. Your named volume `ollama_models` is
kept, so the language model isn't re-downloaded next time.

---

## 6. Run natively (no Docker)

```bash
bash install_native.sh
```

Then open a **new** terminal (the script appends `source` lines to `~/.bashrc`)
and use the exact same launch commands from Sections 2, 3 and 4 — minus the
`docker compose exec ros bash` step.

Natively the GPU is used directly, so no override is needed. On a
hybrid-graphics laptop, prefix a launch with
`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` to force it onto
the NVIDIA card.

Exact package versions are in `DEPENDENCIES.md`.

---

## 7. Working inside the Docker box

**Another terminal:** `docker compose exec ros bash`

**After editing code:** `src/` is live-mounted, so your host edits are already
inside the container. Rebuild there:
```bash
cd /ws && colcon build --symlink-install && source install/setup.bash
```
No image rebuild needed. You only need `--build` again if you change the
`Dockerfile` itself.

**Adding a package:** create it under `src/` on the host, then `colcon build`
inside the container as above.

---

## 8. Why two separate containers

`ollama` runs the language model; `ros` runs everything else. Keeping them apart
means each does one job, the downloaded model persists in a named volume across
`ros` image rebuilds, and it mirrors how a real robot would run its LLM as its
own service rather than bolted into the control stack.

---

## 9. How the pieces fit together (architecture)

**Shared core (Parts 1 & 2):**
- **mission_schemas** — the Pydantic `Mission` model. The one contract every other package agrees on.
- **mission_interface** — you type; it publishes to `/mission/prompt`.
- **mission_llm_planner** — asks the local LLM, emits a draft plan on `/mission/candidate`.
- **mission_validator** — schema + safety rules. Publishes `/mission/validated` or `/mission/rejected`.

**Single robot (Part 1):**
- **mission_executor** — drives one robot via Nav2's `FollowWaypoints`, with a sha256 audit log.

**Multi-robot (Part 2):**
- **formation_coordinator** — turns one squad plan into a per-robot offset path (line/column/wedge) and drives each robot's `/<name>/follow_waypoints`.
- **tb3_multi_robot** *(vendored fork)* — spawns N namespaced TurtleBot3s in one Gazebo world, each with its own Nav2 stack. Our fork of [arshadlab/tb3_multi_robot](https://github.com/arshadlab/tb3_multi_robot) (Apache 2.0); our changes are in `src/tb3_multi_robot/OMOKAI_CHANGES.md`, upstream docs in `UPSTREAM_README.md`. It also provides the **`burger_cam`** variant (camera + ArUco marker) that Part 4 needs.

**Vision follow (Part 4):**
- **vision_follow** — camera → ArUco (or YOLO) detection → operator snapshot on first sighting → follow. In `nav2` mode (default) each detection becomes a map-frame goal a fixed standoff short of the target, sent to `NavigateToPose`, so the follower routes around obstacles. Includes a heading-aware recovery state machine for when the target leaves view. Full detail in `src/vision_follow/README.md`.

**SLAM (Part 3):** separate repository — see the link at the top.

This is why the design is safe and gradeable: for Parts 1 and 2 the AI is kept
**out of the control loop**, every plan is **schema-checked**, and the drivers are
**predictable and logged**. Part 4 keeps the same spirit — the vision node only
*reports* what it sees, and a fixed, auditable control law (with a hard
non-finite-command guard before the wheels) decides what to do about it.

Sources and licenses: `docs/SOURCES.md`.
