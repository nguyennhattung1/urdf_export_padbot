import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('urdf_export_assem_3_sldasm')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_a = LaunchConfiguration('initial_pose_a')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless')
        
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(package_dir, 'worlds', 'empty.world'),
        description='Full path to world file to load')
        
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(package_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')
        
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
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_dir, 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time,
            'gui': gui
        }.items())
    
    # Load the robot description
    robot_description_content = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'robot.xacro')
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Start Map server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }])
    
    # Start robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])
    
    # Start differential drive controller
    diff_drive_controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_dir, 'launch', 'controller.launch.py')]))
    
    # Start AMCL localization
    amcl_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_dir, 'launch', 'includes', 'amcl.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_a': initial_pose_a
        }.items())
    
    # Start Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_dir, 'launch', 'includes', 'nav2.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items())
    
    # Start RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(gui))
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_a_cmd)
    
    # Add the commands to the launch description
    ld.add_action(gazebo_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(diff_drive_controller_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)
    
    return ld 