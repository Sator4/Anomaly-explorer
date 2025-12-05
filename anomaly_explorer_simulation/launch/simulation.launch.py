import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('nav2', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('world', default_value='labyrinth_small'),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]


def generate_launch_description():
    pkg_anomaly_explorer_sim = get_package_share_directory('anomaly_explorer_simulation')
    # pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')

    # ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    gazebo_launch = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'launch', 'gazebo_world.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'launch', 'turtlebot4_spawn.launch.py'])
    # robot_spawn_launch = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'launch', 'turtlebot3_spawn.launch.py'])


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('slam', LaunchConfiguration('slam')),
            ('nav2', LaunchConfiguration('nav2')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )


    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(gazebo)
    ld.add_action(robot_spawn)

    return ld


    