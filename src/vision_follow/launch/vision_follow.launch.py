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
# Brings up vision_follow_node in the namespace of an already-spawned
# burger_cam robot (from tb3_multi_robot's tb3_world.launch.py). Run
# tb3_world.launch.py first.
#
# Two usage modes:
#   YOLO (follow a named COCO object):
#     ros2 launch vision_follow vision_follow.launch.py \
#         robot_name:=tb1 detection_mode:=yolo target_class:=person
#
#   ArUco (follow another robot wearing a printed marker):
#     ros2 launch vision_follow vision_follow.launch.py \
#         robot_name:=tb1 detection_mode:=aruco aruco_marker_id:=0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='tb1',
        description='Namespace of the already-spawned robot to attach vision/follow to (e.g. tb1).')
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode', default_value='yolo',
        description="'yolo' (pretrained COCO classes) or 'aruco' (fiducial marker on the target robot).")
    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='person',
        description=(
            "[yolo mode] Initial COCO class to detect/follow. Can be "
            "changed at runtime by publishing std_msgs/String to "
            "'<robot_name>/vision/set_target_class'."))
    aruco_dictionary_arg = DeclareLaunchArgument(
        'aruco_dictionary', default_value='DICT_4X4_50',
        description="[aruco mode] cv2.aruco predefined dictionary name.")
    aruco_marker_id_arg = DeclareLaunchArgument(
        'aruco_marker_id', default_value='0',
        description=(
            "[aruco mode] Marker id to follow. Can be changed at runtime "
            "by publishing the id (as a string) to "
            "'<robot_name>/vision/set_target_class'."))
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n.pt',
        description='[yolo mode] Path or name of the Ultralytics YOLO weights file.')
    confidence_arg = DeclareLaunchArgument('confidence_threshold', default_value='0.5')
    device_arg = DeclareLaunchArgument(
        'device', default_value='cpu',
        description=(
            "[yolo mode] Inference device, e.g. 'cpu' or 'cuda:0'. Defaults "
            "to 'cpu' for portability. Confirm torch.cuda.is_available() "
            "is True before passing device:=cuda:0."))
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    robot_name = LaunchConfiguration('robot_name')
    detection_mode = LaunchConfiguration('detection_mode')
    target_class = LaunchConfiguration('target_class')
    aruco_dictionary = LaunchConfiguration('aruco_dictionary')
    aruco_marker_id = LaunchConfiguration('aruco_marker_id')
    model_path = LaunchConfiguration('model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    device = LaunchConfiguration('device')
    use_sim_time = LaunchConfiguration('use_sim_time')

    vision_follow_node = Node(
        package='vision_follow',
        executable='vision_follow_node',
        name='vision_follow_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'detection_mode': detection_mode,
            'target_class': target_class,
            'aruco_dictionary': aruco_dictionary,
            'aruco_marker_id': aruco_marker_id,
            'model_path': model_path,
            'confidence_threshold': confidence_threshold,
            'device': device,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        detection_mode_arg,
        target_class_arg,
        aruco_dictionary_arg,
        aruco_marker_id_arg,
        model_path_arg,
        confidence_arg,
        device_arg,
        use_sim_time_arg,
        vision_follow_node,
    ])
