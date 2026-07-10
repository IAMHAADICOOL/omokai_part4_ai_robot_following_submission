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
# Part 4: bring up Nav2 for the FOLLOWER ROBOT ONLY.
#
# tb3_multi_robot/launch/tb3_nav2.launch.py loops over every enabled robot in
# robots.yaml and starts a full Nav2 stack for each. For the vision-follow demo
# only the follower needs to plan; the target is driven by teleop. Starting a
# second Nav2 stack costs ~14 extra lifecycle nodes, two costmaps, and an AMCL
# particle filter for nothing -- and its collision_monitor also publishes to the
# target's cmd_vel, which fights teleop.
#
# This launch starts exactly one stack, with the tuned close-follow params.
#
#   ros2 launch vision_follow follow_nav2.launch.py robot_name:=tb1
#
# Nav2 params default to vision_follow/params/burger_cam_follow_nav2_params.yaml
# (a copy of tb3_multi_robot's burger_cam params with only the "how close may I
# get" knobs changed), so Parts 1 and 2 are completely untouched.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from multi_robot_scripts.utils import generate_rviz_config


def _spawn_pose_for(robot_name, robots_yaml):
    """Single source of truth: the robot's spawn pose in robots.yaml is also
    the pose we seed AMCL with, so the two can never disagree."""
    with open(robots_yaml, 'r') as f:
        robots = yaml.safe_load(f)['robots']
    for r in robots:
        if r['name'] == robot_name:
            return float(r['x_pose']), float(r['y_pose'])
    raise RuntimeError(
        f"robot '{robot_name}' not found in {robots_yaml}. "
        f"Known: {[r['name'] for r in robots]}")


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    tb3_dir = get_package_share_directory('tb3_multi_robot')
    robots_yaml = os.path.join(tb3_dir, 'config', 'robots.yaml')
    x, y = _spawn_pose_for(robot_name, robots_yaml)

    # Same RViz config tb3_nav2.launch.py uses (Nav2 panel, Selector, Docking
    # panel, costmaps, laser scan, robot model, goal tool, waypoints marker
    # array) -- generate_rviz_config() stamps the robot's namespace into the
    # <ROBOT_NAME> placeholder in the shared template.
    rviz_template_path = os.path.join(tb3_dir, 'rviz', 'tb3_navigation2.rviz')
    rviz_config = generate_rviz_config(robot_name, rviz_template_path)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'use_namespace': 'true',
            'namespace': robot_name,
        }.items())

    # AMCL seed. Without this there is no map->odom, so no 'map' frame, so the
    # vision node can never transform a detection into a goal. Delayed until
    # amcl is active. Same technique tb3_nav2.launch.py uses.
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25
    cov[35] = 0.068
    initialpose = (
        '{header: {frame_id: map}, pose: {pose: {position: {x: ' + str(x) +
        ', y: ' + str(y) + ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, '
        'w: 1.0}}, covariance: [' + ', '.join(str(c) for c in cov) + ']}}')

    seed_amcl = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg=f'[follow_nav2] seeding {robot_name} AMCL at spawn pose '
                        f'({x}, {y}) -- map frame should appear right after this'),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '-t', '6', '-r', '2',
                     f'/{robot_name}/initialpose',
                     'geometry_msgs/msg/PoseWithCovarianceStamped', initialpose],
                output='screen'),
        ])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        namespace=f'/{robot_name}',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'log_level': 'warn'}],
        # Per-robot TF lives on /<ns>/tf, not the global /tf.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen')

    return [
        LogInfo(msg=f'[follow_nav2] starting Nav2 for ONE robot: {robot_name} '
                    f'(waiting ~10s for AMCL seed before the map frame exists)'),
        nav2_launch,
        seed_amcl,
        rviz,
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('vision_follow'),
        'params', 'burger_cam_follow_nav2_params.yaml')
    default_map = os.path.join(
        get_package_share_directory('tb3_multi_robot'), 'map', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='tb1',
            description='The FOLLOWER robot. Only this robot gets a Nav2 stack.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 params. Defaults to the tuned close-follow set.'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz.'),
        OpaqueFunction(function=_setup),
    ])
