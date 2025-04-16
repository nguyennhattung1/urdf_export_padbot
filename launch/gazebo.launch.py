#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_name = 'urdf_export_assem_3_sldasm'
    package_dir = get_package_share_directory(pkg_name)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(package_dir, 'worlds', 'empty.world'),
            'verbose': 'true',
        }.items(),
    )

    # Get URDF via xacro
    urdf_file = os.path.join(package_dir, 'urdf', 'robot.urdf')
    robot_description_content = open(urdf_file, 'r').read()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_dir, 'worlds', 'empty.world'),
            description='SDF world file'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ]) 