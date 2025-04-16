#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_name = 'urdf_export_assem_3_sldasm'
    package_dir = get_package_share_directory(pkg_name)

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(
        package_dir, 'urdf', 'robot.urdf'))
    rviz_config = LaunchConfiguration('rviz_config', default=os.path.join(
        package_dir, 'rviz', 'robot.rviz'))

    # Include the robot state publisher launch file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(os.path.join(
                package_dir, 'urdf', 'robot.urdf'), 'r').read()}
        ]
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create and return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(package_dir, 'urdf', 'robot.urdf'),
            description='URDF file'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_dir, 'rviz', 'robot.rviz'),
            description='RViz config file'),
        robot_state_publisher,
        rviz_node
    ]) 