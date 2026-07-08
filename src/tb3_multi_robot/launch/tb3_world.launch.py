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
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


from multi_robot_scripts.utils import load_sdf_with_namespace, create_namespaced_bridge_yaml

def generate_launch_description():
    # Paths
    tb3_multi_dir = get_package_share_directory('tb3_multi_robot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Simulation config
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = os.path.join(tb3_multi_dir, 'worlds', 'tb3_world.world')

    # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
    )

    # Main LaunchDescription
    ld = LaunchDescription()

    # Add GZ model path to env FIRST, before anything that can start Gazebo.
    # AppendEnvironmentVariable only affects processes that spawn AFTER it
    # executes. Gazebo's actual process spawn happens a couple of
    # IncludeLaunchDescription layers deep (inside gz_sim.launch.py), so
    # adding this action late (as it originally was, right before `return
    # ld`) raced against that spawn -- sometimes winning, sometimes not,
    # which is why models under this package's own models/ folder (like
    # turtlebot3_burger_cam's ArUco texture) could silently fail to resolve
    # while models found via the system-wide turtlebot3_simulations resource
    # path (e.g. turtlebot3_common meshes, registered by that apt package's
    # own env-hook well before this launch file runs) always worked. Setting
    # it first removes the race entirely.
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_multi_dir, 'models'))
    )

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Load robot config
    robot_config_path = os.path.join(tb3_multi_dir, 'config', 'robots.yaml')
    with open(robot_config_path, 'r') as f:
        config = yaml.safe_load(f)

    robots = [r for r in config['robots'] if r.get('enabled', True)]
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_dir = f'turtlebot3_{tb3_model}'
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    urdf_file_name = 'turtlebot3_' + tb3_model + '.urdf'
    urdf_path = os.path.join(
        tb3_multi_dir,
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    for robot in robots:
        namespace = robot['name']

        sdf_path = os.path.join(tb3_multi_dir, 'models', model_dir, 'model.sdf')
        patched_sdf = load_sdf_with_namespace(sdf_path, namespace)

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='screen',
            # parameters=[{
            #          # FIX (Jazzy/Harmonic): the original PythonExpression(["'", frame_prefix,                 'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])

            # }])
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_desc, value_type=str),
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }])

        spawner_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-name', f'{namespace}_{tb3_model}',
                '-string', patched_sdf,
                '-x', str(robot['x_pose']),
                '-y', str(robot['y_pose']),
                '-z', '0.01',
            ],
            output='screen',
        )

        bridge_template = os.path.join(tb3_multi_dir, 'params', f'{tb3_model}_bridge.yaml')
        namespaced_bridge = create_namespaced_bridge_yaml(bridge_template, namespace)

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={namespaced_bridge}'],
            output='screen',
        )

        # Add image bridge if model has camera
        image_bridge = None
        if tb3_model != 'burger':
            image_bridge = Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                arguments=['/' + namespace + '/camera/image_raw'],
                output='screen',
            )

        # Add robot-related nodes
        ld.add_action(robot_state_publisher)
        ld.add_action(spawner_node)
        ld.add_action(bridge_node)
        if image_bridge:
            ld.add_action(image_bridge)

    # In a multi-robot setup using Gazebo Sim (Harmonic or later), each robot typically
    # requires a separate ROS-Gazebo bridge to relay topics such as sensor data, odometry,
    # and control commands between Gazebo and ROS 2.
    # However, some topics like `/clock` are *global* and should be published only once
    # to avoid conflicts or duplication. If multiple bridges publish `/clock`, it may lead
    # to inconsistent simulation time behavior across nodes or unnecessary topic traffic.
    # Therefore, the `/clock` topic is handled separately:
    # - It is excluded from the per-robot bridge configuration files (YAMLs).
    # - A dedicated, single bridge instance is launched to publish `/clock` from Gazebo to ROS 2.
    # This ensures consistent simulation time across the entire ROS 2 system while supporting
    # multiple robot instances with their own bridges.

    # Global clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    ld.add_action(clock_bridge)

    return ld
