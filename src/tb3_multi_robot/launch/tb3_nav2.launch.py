#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from multi_robot_scripts.utils import generate_rviz_config

def generate_launch_description():
    # ───── Set up paths and environment ────────────────────────────────────────
    tb3_multi_dir = get_package_share_directory('tb3_multi_robot')
    robot_config_path = os.path.join(tb3_multi_dir, 'config', 'robots.yaml')
    rviz_template_path = os.path.join(tb3_multi_dir, 'rviz', 'tb3_navigation2.rviz')
    default_map_path = os.path.join(tb3_multi_dir, 'map', 'map.yaml')

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')

    param_file_name = f"{tb3_model}_nav2_params.yaml"
    param_file_path = os.path.join(tb3_multi_dir, 'params', param_file_name)

    # ───── Launch arguments ────────────────────────────────────────────────────
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map file to load.'
    )

    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=param_file_path,
        description='Full path to the navigation parameter file.'
    )

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.'
    )

    # Create main launch description
    ld = LaunchDescription()
    ld.add_action(declare_map_arg)
    ld.add_action(declare_params_arg)
    ld.add_action(declare_sim_time_arg)

    # Load robot configurations
    with open(robot_config_path, 'r') as f:
        robots = [r for r in yaml.safe_load(f)['robots'] if r.get('enabled', True)]

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map')
    params_path = LaunchConfiguration('params_file')

    # Per-robot bringup
    for robot in robots:
        robot_name = robot['name']
        namespace = f'/{robot_name}'

        rviz_config = generate_rviz_config(robot_name, rviz_template_path)

        ld.add_action(LogInfo(msg=['[Launch] Using param file: ', params_path]))

        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': params_path,
                'use_namespace': 'true',
                'namespace': robot_name
            }.items()
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time, 'log_level': 'warn'}],
            remappings=[
                ('/tf', f'tf'),
                ('/tf_static', f'tf_static')
            ],
            output='screen'
        )

        ld.add_action(nav2_launch)
        ld.add_action(rviz_node)

        # ── AMCL auto-seed (FIX) ─────────────────────────────────────────────
        # The params ship with set_initial_pose: false, and every robot gets the
        # same params file, so AMCL never localizes -> no map->odom -> robots
        # can't be related to the map and won't move. We seed each robot's AMCL
        # at its known spawn pose (from robots.yaml) by publishing to
        # /<ns>/initialpose a few seconds after startup, once amcl is active.
        # This is the programmatic equivalent of RViz's "2D Pose Estimate".
        x = float(robot['x_pose'])
        y = float(robot['y_pose'])
        cov = [0.0] * 36
        cov[0] = 0.25    # x variance
        cov[7] = 0.25    # y variance
        cov[35] = 0.068  # yaw variance
        cov_str = "[" + ", ".join(str(c) for c in cov) + "]"
        initialpose_msg = (
            "{header: {frame_id: map}, "
            "pose: {pose: {position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, "
            "covariance: " + cov_str + "}}")
        seed_amcl = TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                cmd=["ros2", "topic", "pub", "-t", "6", "-r", "2",
                     f"/{robot_name}/initialpose",
                     "geometry_msgs/msg/PoseWithCovarianceStamped",
                     initialpose_msg],
                output="screen")])
        ld.add_action(seed_amcl)

    return ld
