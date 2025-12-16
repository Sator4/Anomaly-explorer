import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('nav2', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('world', default_value='labyrinth_long_houses'),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]


def generate_launch_description():
    # Directories
    pkg_anomaly_explorer_sim = get_package_share_directory('anomaly_explorer_simulation')
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_gz_bringup')


    # Paths
    gazebo_launch = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'launch', 'gazebo_world.launch.py'])
    # robot_spawn_launch = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'launch', 'turtlebot4_spawn.launch.py']) # mine
    robot_spawn_launch = PathJoinSubstitution([pkg_turtlebot4_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    nav2_params = PathJoinSubstitution([pkg_anomaly_explorer_sim, 'config', 'nav2_params.yaml'])


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('emulate_tty', 'true'),
            # ('params_file', nav2_params)
        ],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('emulate_tty', 'true'),
            ('namespace', LaunchConfiguration('namespace')),
            ('model', 'lite'),
            ('sync', 'false'),
            ('rviz', LaunchConfiguration('rviz')),
            ('slam', LaunchConfiguration('slam')),
            ('nav2', LaunchConfiguration('nav2')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            # ('params_file', nav2_params)
        ]
    )


    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(gazebo)
    ld.add_action(robot_spawn)

    return ld


    