"""Phase 1 pipeline with two modes, selected by `slam:=`.

MAPPING (build a map to save):
    ros2 launch omokai_bringup core_pipeline.launch.py slam:=True
    # in another terminal, drive:
    ros2 run turtlebot3_teleop teleop_keyboard
    # when the map looks complete, save it INTO the package:
    ros2 run nav2_map_server map_saver_cli -f \
        src/omokai_bringup/maps/turtlebot3_world
    # then rebuild so the map installs:
    colcon build --symlink-install

RUN (localize in the saved map + full prompt->LLM->JSON->executor pipeline):
    ros2 launch omokai_bringup core_pipeline.launch.py            # slam:=False (default)
    ros2 launch omokai_bringup core_pipeline.launch.py map:=/abs/path/map.yaml

WHERE THE ROBOT STARTS is defined ONCE in config/spawn_pose.yaml:
    world_spawn -> where Gazebo puts the robot (simulator world frame)
    map_seed    -> that spot in the saved-map frame -> AMCL's startup pose
This file drives BOTH the Gazebo spawn and the AMCL seed, so they can never
disagree (which is what used to make the robot start mislocalized). See the
comments in spawn_pose.yaml for why map_seed is normally (0,0,0).

NOTE: pass slam:=True / slam:=False with a capital letter -- nav2's own
bringup_launch.py evaluates this argument with a Python-style eval() internally,
and only True/False (not true/false) are valid Python literals.

Requires: sudo apt install xterm
"""
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _render_nav2_params(nav2_params_path, map_seed):
    """Return a path to a copy of nav2_params.yaml whose AMCL section is seeded
    at `map_seed`. This is how the robot localizes correctly at startup with no
    manual estimate: AMCL's own set_initial_pose is set to the map-frame spawn.
    Done here (not by a /initialpose republish) so the particle cloud is seeded
    exactly once, at cold start -- no later disturbance."""
    with open(nav2_params_path) as f:
        params = yaml.safe_load(f)
    amcl = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl["set_initial_pose"] = True
    amcl["initial_pose"] = {
        "x": float(map_seed["x"]),
        "y": float(map_seed["y"]),
        "z": 0.0,
        "yaw": float(map_seed["yaw"]),
    }
    out = os.path.join(tempfile.gettempdir(), "omokai_nav2_params.yaml")
    with open(out, "w") as f:
        yaml.dump(params, f)
    return out


def generate_launch_description():
    bringup = get_package_share_directory("omokai_bringup")
    nav2 = get_package_share_directory("nav2_bringup")

    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")

    routes_file = os.path.join(bringup, "config", "routes.yaml")
    nav2_params = os.path.join(bringup, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup, "maps", "turtlebot3_world.yaml")

    # ── Single source of truth for where the robot starts ────────────────────
    # world_spawn -> Gazebo (world frame); map_seed -> AMCL (map frame).
    # Both read from ONE file so the simulator and the localizer always agree.
    with open(os.path.join(bringup, "config", "spawn_pose.yaml")) as f:
        spawn_cfg = yaml.safe_load(f)
    world_spawn = spawn_cfg["world_spawn"]
    map_seed = spawn_cfg["map_seed"]

    # Bake the map_seed into AMCL's initial pose.
    seeded_nav2_params = _render_nav2_params(nav2_params, map_seed)

    declare_slam = DeclareLaunchArgument(
        "slam", default_value="False",
        description="True = map the world (drive to build a map); "
                    "False = localize in saved map + run the pipeline")
    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Saved map yaml (used when slam:=False)")

    # world_spawn -> Gazebo spawn pose
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "sim.launch.py")),
        launch_arguments={
            "x_pose": str(world_spawn["x"]),
            "y_pose": str(world_spawn["y"]),
            "yaw": str(world_spawn["yaw"]),
        }.items())

    # nav2 bringup switches internally on `slam`:
    #   slam:=True  -> slam_toolbox + navigation (mapping)
    #   slam:=False -> map_server + amcl (seeded at map_seed) + navigation
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2, "launch", "bringup_launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "slam": slam,
            "map": map_yaml,
            "params_file": seeded_nav2_params,
        }.items())

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(nav2, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": True}])

    # Spine nodes run only in RUN mode (slam:=False), each in its own xterm.
    def xterm(pkg, exe, title, params=None):
        return Node(
            package=pkg, executable=exe, name=exe, output="screen",
            prefix=[f'xterm -T "{title}" -geometry 100x24 -hold -e'],
            parameters=params or [],
            condition=UnlessCondition(slam))

    interface = xterm("mission_interface", "prompt_publisher",
                      "1 INTERFACE  (type prompts here)")
    planner = xterm("mission_llm_planner", "llm_planner", "2 LLM PLANNER")
    validator = xterm("mission_validator", "mission_validator", "3 VALIDATOR",
                      [{"routes_file": routes_file}])
    executor = xterm("mission_executor", "mission_executor", "4 EXECUTOR",
                     [{"use_sim_time": True, "routes_file": routes_file}])

    # Delay so map_server + amcl + nav2 are active before the executor sends goals.
    spine = TimerAction(
        period=10.0, condition=UnlessCondition(slam),
        actions=[interface, planner, validator, executor])

    return LaunchDescription([
        declare_slam, declare_map, sim, nav2_bringup, rviz, spine,
    ])
