"""Phase 0: bring up a TurtleBot3 in Gazebo Harmonic (ROS 2 Jazzy, ros_gz).

Thin wrapper around turtlebot3_gazebo's world launches so we don't re-implement
the ros_gz bridge/spawn ourselves. Pick the environment with `world:=`.

    ros2 launch omokai_bringup sim.launch.py
    ros2 launch omokai_bringup sim.launch.py world:=turtlebot3_house.launch.py
    ros2 launch omokai_bringup sim.launch.py world:=empty_world.launch.py

The robot model defaults to `burger` but respects an existing TURTLEBOT3_MODEL export.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Respect an existing export; otherwise default to burger.
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world.launch.py',
        description=(
            'turtlebot3_gazebo launch file to include: '
            'empty_world.launch.py | turtlebot3_world.launch.py | '
            'turtlebot3_house.launch.py'
        ),
    )
    # Deterministic spawn pose. Pinning these (and seeding AMCL at the same
    # values upstream) is what makes localization correct on the first try.
    declare_x = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='-0.5')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', model)

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb3_gazebo, 'launch', world])
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([
        declare_world,
        declare_x,
        declare_y,
        declare_yaw,
        set_model,
        world_launch,
    ])
