# Copyright 2021 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('model', default_value='lite', choices=['standard', 'lite']),
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('localization', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('nav2', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.001'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('turtlebot4_navigation'),
                              'config',
                              'nav2.yaml'
                              ]))
]



def generate_launch_description():

    # Directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')

    # Paths
    turtlebot4_ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_gz_bringup, 'launch', 'ros_gz_bridge.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_gz_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_gz_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_navigation.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    xacro_file = PathJoinSubstitution([pkg_turtlebot4_description, 'urdf', LaunchConfiguration('model'), 'turtlebot4.urdf.xacro'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    nav2_params = LaunchConfiguration('params_file')

    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description': ParameterValue(
                    Command([
                        'xacro', ' ', xacro_file, ' ',
                        'gazebo:=ignition', ' ',
                        'namespace:=', namespace]),
                    value_type=str)},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': ParameterValue(
                    Command([
                        'xacro', ' ', xacro_file, ' ',
                        'gazebo:=ignition', ' ',
                        'namespace:=', namespace,
                        # 'gazebo_controllers:=', PathJoinSubstitution(
                        #     [get_package_share_directory('irobot_create_control'), 'config', 'control.yaml'])
                    ]),
                    value_type=str
                )},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # Spawn TurtleBot 4
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # ROS IGN bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_ros_gz_bridge_launch]),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('model', LaunchConfiguration('model')),
                ('robot_name', robot_name),
                ('namespace', namespace)]
        ),

        # Create 3 nodes            отвечают за симуляцию сенсоров всяких всяких приколов реального мира, типа ограничений по скорости, если робот стоит перед стеной, реакция на то, что робота подняли и тд.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('namespace', namespace)
            ]
        ),

        # Create 3 Gazebo nodes     отвечают за то, чтобы обработать данные из газебы в понятные для create3 nodes, тк они могут от данных из реального мира, например, положение чего-то может браться не из датчиков, а из данных симуляции.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_gz_nodes_launch]),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('robot_name', robot_name),
            ]
        ),

        # RPLIDAR static transforms
        Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
    ])

    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params)
        ],
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )


    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(spawn_robot_group_action)
    ld.add_action(localization)
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(rviz)
    return ld
