import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

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

    return LaunchDescription([

      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': False}, robot_description]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[robot_description],
            arguments=['-d',rviz2_config_path]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui':True}],
            output='screen',
            ),
    ])