# Copyright 2023 Clearpath Robotics, Inc.
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

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='labyrinth_small',
                          description='Simulation World'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('turtlebot4_navigation'),
                              'config',
                              'nav2.yaml'
                              ]))
]


def generate_launch_description():

    # Directories
    pkg_anomaly_explorer_sim = get_package_share_directory('anomaly_explorer_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_anomaly_explorer_sim, 'worlds'),
            os.path.join(pkg_anomaly_explorer_sim, 'models'),
            str(Path(pkg_anomaly_explorer_sim).parent.resolve()),
            str(Path(pkg_ros_gz_sim).parent.resolve()),
        ])
    )

    # Paths
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    nav2_params = LaunchConfiguration('params_file')

    # Gazebo harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 2',  # error, warning, info, debug
                ' --gui-config ',
                PathJoinSubstitution([
                    pkg_anomaly_explorer_sim,
                    'config',
                    'labyrinth_small.config'
                ])
            ])
        ]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
        ]
    )


    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )


    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)

    # ld.add_action(planner_cmd)
    # ld.add_action(controller_cmd)
    # ld.add_action(bt_navigator_cmd)

    return ld
