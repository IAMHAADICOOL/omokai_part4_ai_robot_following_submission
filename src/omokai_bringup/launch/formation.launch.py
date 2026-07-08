"""Phase 2: multi-robot formation pipeline (Challenge 1).

Runs ONLY the prompt->LLM->JSON->coordinator pipeline. The robots + per-robot
Nav2 are started separately by tb3_multi_robot, which exposes
/<ns>/follow_waypoints for each enabled robot. The formation_coordinator
drives those namespaced actions.

SQUAD MEMBERSHIP (single source of truth): the coordinator reads
tb3_multi_robot's config/robots.yaml itself and drives every robot with
enabled: true -- the SAME file that decides which robots spawn. Enabling more
robots there (e.g. tb2, tb4) is the only change needed to scale the squad; no
launch-file edit required.

── Terminal 1 : sim (Gazebo turtlebot3_world + enabled robots) ──
    ros2 launch tb3_multi_robot tb3_world.launch.py
── Terminal 2 : per-robot Nav2 (localization + navigation) ──
    ros2 launch tb3_multi_robot tb3_nav2.launch.py
    # wait until every robot is localized (map+robot visible in each RViz)
── Terminal 3 : this formation pipeline ──
    ros2 launch omokai_bringup formation.launch.py
    # then in the INTERFACE xterm, type e.g.:  Sweep the area in a wedge

Requires: sudo apt install xterm ; and Ollama running (LLM_MODEL=qwen2.5:3b).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("omokai_bringup")
    routes_file = os.path.join(bringup, "config", "routes.yaml")

    tb3_multi = get_package_share_directory("tb3_multi_robot")
    robots_yaml = os.path.join(tb3_multi, "config", "robots.yaml")

    def xterm(pkg, exe, title, params=None):
        return Node(
            package=pkg, executable=exe, name=exe, output="screen",
            prefix=[f'xterm -T "{title}" -geometry 100x24 -hold -e'],
            parameters=params or [])

    interface = xterm("mission_interface", "prompt_publisher",
                      "1 INTERFACE  (type prompts here)")
    planner = xterm("mission_llm_planner", "llm_planner", "2 LLM PLANNER")
    validator = xterm("mission_validator", "mission_validator", "3 VALIDATOR",
                      [{"routes_file": routes_file}])
    coordinator = xterm("formation_coordinator", "formation_coordinator",
                        "4 FORMATION COORDINATOR",
                        [{"use_sim_time": True,
                          "routes_file": routes_file,
                          "robots_yaml": robots_yaml}])

    return LaunchDescription([interface, planner, validator, coordinator])
