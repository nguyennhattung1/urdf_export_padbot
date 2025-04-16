import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('urdf_export_assem_3_sldasm')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_map_topic = LaunchConfiguration('use_map_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_a = LaunchConfiguration('initial_pose_a')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_map_topic_cmd = DeclareLaunchArgument(
        'use_map_topic',
        default_value='false',
        description='Use the map topic rather than requesting the map')
        
    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Topic that has the laser scan')
        
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose X coordinate')
        
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose Y coordinate')
        
    declare_initial_pose_a_cmd = DeclareLaunchArgument(
        'initial_pose_a',
        default_value='0.0',
        description='Initial pose yaw angle')
    
    # Start AMCL node
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_model_type': 'differential',
            'odom_model_type': 'diff-corrected',
            'odom_alpha5': 0.1,
            'gui_publish_rate': 10.0,
            'laser_max_beams': 60,
            'laser_max_range': 12.0,
            'min_particles': 500,
            'max_particles': 2000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'odom_alpha1': 0.2,
            'odom_alpha2': 0.2,
            'odom_alpha3': 0.2,
            'odom_alpha4': 0.2,
            'laser_z_hit': 0.5,
            'laser_z_short': 0.05,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.5,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'update_min_d': 0.25,
            'update_min_a': 0.2,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'resample_interval': 1,
            'transform_tolerance': 1.0,
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_a': initial_pose_a,
            'use_map_topic': use_map_topic
        }],
        remappings=[('scan', scan_topic)])
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_map_topic_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_a_cmd)
    
    # Add the nodes to launch
    ld.add_action(amcl_cmd)
    
    return ld 