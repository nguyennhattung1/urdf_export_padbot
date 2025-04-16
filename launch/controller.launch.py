import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('urdf_export_assem_3_sldasm')
    
    # Launch configuration variables
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    
    # Declare the launch arguments
    declare_wheel_radius_cmd = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius in meters')
        
    declare_wheel_separation_cmd = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.42446',
        description='Distance between the wheels in meters')
        
    declare_base_frame_id_cmd = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Name of the base frame')
        
    declare_odom_frame_id_cmd = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Name of the odometry frame')
    
    # Start the differential drive controller
    diff_drive_controller_cmd = Node(
        package='urdf_export_assem_3_sldasm',
        executable='differential_drive_controller.py',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_separation': wheel_separation,
            'base_frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id
        }])
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_wheel_radius_cmd)
    ld.add_action(declare_wheel_separation_cmd)
    ld.add_action(declare_base_frame_id_cmd)
    ld.add_action(declare_odom_frame_id_cmd)
    
    # Add the nodes to launch
    ld.add_action(diff_drive_controller_cmd)
    
    return ld 