# Copyright 2020 ROS2-Control Development Team (2020)
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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('mz25_robot_description'),
        'description',
        'mz25_arm_vacuum_gripper.urdf.xacro')
    rviz2_config_path = os.path.join(
        get_package_share_directory('mz25_robot_description'),
        'rviz',
        'show.rviz')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    forward_controller = os.path.join(
        get_package_share_directory('mz25_robot_description'),
        'controllers',
        'mz25_controller_position.yaml'
        )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, forward_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            arguments=['--ros-args', '--log-level', 'info']
        ),
        ExecuteProcess(cmd=['ros2 control load_start_controller mz25_arm_controller'], shell=True, output='screen'),
        ExecuteProcess(cmd=['ros2 control load_start_controller joint_state_controller'], shell=True, output='screen'),
    ])
