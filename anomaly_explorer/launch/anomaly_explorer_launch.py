# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Directories
    anomaly_explorer_dir = get_package_share_directory('anomaly_explorer')
    anomaly_explorer_simulation_dir = get_package_share_directory('anomaly_explorer_simulation')
    plansys2_bringup_dir = get_package_share_directory('plansys2_bringup')


    # Paths
    plansys2_path = PathJoinSubstitution(
        [plansys2_bringup_dir, 'launch', 'plansys2_bringup_launch_monolithic.py'])
    simulation_path = PathJoinSubstitution(
        [anomaly_explorer_simulation_dir, 'launch', 'simulation.launch.py']) 


    # Launch configurations
    plansys2_cmd = TimerAction(period=3.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(plansys2_path),
        launch_arguments=[
            ('model_file', anomaly_explorer_dir + '/pddl/domain.pddl'),
            ('emulate_tty', 'true')]
    )])
    
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_path)
    )

    explore_cmd = TimerAction(period=5.0, actions=[
        Node(
            package='anomaly_explorer_py',
            executable='explore_action_node.py',
            name='explore_action_node',
            emulate_tty='true',
            output='screen',
            parameters=[])
    ])

    investigate_cmd = TimerAction(period=5.0, actions=[
        Node(
            package='anomaly_explorer_py',
            executable='investigate_action_node.py',
            name='investigate_action_node',
            emulate_tty='true',
            output='screen',
            parameters=[])
    ])


    ld = LaunchDescription()

    ld.add_action(plansys2_cmd)
    ld.add_action(simulation_cmd)

    ld.add_action(explore_cmd)
    ld.add_action(investigate_cmd)
    
    return ld
