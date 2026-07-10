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
# Part 4: vision detect + follow, attached to an already-spawned burger_cam.
#
#   nav2 mode (default) -- obstacle-aware. Run tb3_world + follow_nav2 first:
#     ros2 launch vision_follow vision_follow.launch.py robot_name:=tb1
#
#   follow more closely (see README for the Nav2 params that must move too):
#     ros2 launch vision_follow vision_follow.launch.py robot_name:=tb1 \
#         standoff_distance:=0.45 stop_distance:=0.5 resume_distance:=0.7
#
#   direct mode -- reactive, no map, no obstacle avoidance. tb3_world only:
#     ros2 launch vision_follow vision_follow.launch.py \
#         robot_name:=tb1 control_mode:=direct

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_name', default_value='tb1',
                              description='Namespace of the FOLLOWER robot.'),
        DeclareLaunchArgument('control_mode', default_value='nav2',
                              description="'nav2' (plans around obstacles) or 'direct'."),
        DeclareLaunchArgument('detection_mode', default_value='aruco',
                              description="'aruco' or 'yolo'."),
        DeclareLaunchArgument('aruco_marker_id', default_value='0'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_length', default_value='0.08',
                              description='Real marker side in metres; scales PnP distance.'),
        DeclareLaunchArgument('target_class', default_value='person'),
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='cpu'),

        # ---- how closely to follow (issue 2) ----
        DeclareLaunchArgument(
            'standoff_distance', default_value='0.5',
            description='Goal is placed this far behind the target. Lower = tighter. '
                        'Must clear the target robot\'s inflated footprint in the '
                        'follower costmap, else Nav2 rejects the goal: with '
                        'inflation_radius 0.25 + robot_radius 0.1 the floor is ~0.45.'),
        DeclareLaunchArgument(
            'stop_distance', default_value='0.6',
            description='Stop chasing once this close. Must be >= standoff_distance.'),
        DeclareLaunchArgument(
            'resume_distance', default_value='0.85',
            description='Resume once the target pulls out to here. Must exceed '
                        'stop_distance; the gap is anti-chatter hysteresis.'),
        DeclareLaunchArgument('goal_update_period', default_value='0.75'),
        DeclareLaunchArgument('goal_update_min_dist', default_value='0.25'),

        # ---- lost-target recovery (issue 1) ----
        DeclareLaunchArgument(
            'target_lost_grace_sec', default_value='5.0',
            description='Seconds without a detection before recovery starts.'),
        DeclareLaunchArgument(
            'recovery_timeout_sec', default_value='20.0',
            description='How long to keep trying to re-acquire before giving up.'),
        DeclareLaunchArgument(
            'recovery_goal_backoff', default_value='0.35',
            description="Stop this far short of the target's last position, so we "
                        "never try to occupy a cell it may still be in."),
        DeclareLaunchArgument(
            'min_face_on_cosine', default_value='0.35',
            description='Reject the target-heading estimate when the marker is more '
                        'oblique than this (its normal, and so its yaw, is noise).'),

        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    lc = LaunchConfiguration
    passthrough = [
        'control_mode', 'detection_mode', 'aruco_marker_id', 'aruco_dictionary',
        'marker_length', 'target_class', 'model_path', 'confidence_threshold',
        'device', 'standoff_distance', 'stop_distance', 'resume_distance',
        'goal_update_period', 'goal_update_min_dist', 'target_lost_grace_sec',
        'recovery_timeout_sec', 'recovery_goal_backoff', 'min_face_on_cosine',
        'global_frame', 'robot_base_frame', 'use_sim_time',
    ]

    node = Node(
        package='vision_follow',
        executable='vision_follow_node',
        name='vision_follow_node',
        namespace=lc('robot_name'),
        output='screen',
        # CRITICAL for nav2 mode: each robot's TF tree lives on /<ns>/tf. Without
        # this remap the TransformListener reads the global /tf, never finds
        # map->base_link, and silently never emits a goal. Mirrors what
        # tb3_nav2.launch.py does for its RViz node.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{k: lc(k) for k in passthrough}],
    )

    return LaunchDescription(args + [node])
