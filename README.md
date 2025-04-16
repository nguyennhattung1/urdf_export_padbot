# Robot Description Package

This package contains the XACRO-based robot description for a mobile robot with differential drive and mecanum wheels.

## Structure

The XACRO files are organized as follows:

- `robot.xacro` - Main robot description file that includes all other XACRO files
- `properties.xacro` - All robot parameters and constants
- `materials.xacro` - Material definitions for visualization
- `macros.xacro` - Reusable link and joint definitions

## Meshes

The robot uses the following mesh files for visualization and collision:

- `base_link.STL` - Robot base
- `Right_wheel_link.STL` - Right main wheel
- `Left_wheel_link.STL` - Left main wheel
- `Back_MDW_frame_link.STL` - Back mecanum drive wheel frame
- `Back_MDW_wheel_link.STL` - Back mecanum drive wheel
- `Front_MDW_frame_link.STL` - Front mecanum drive wheel frame
- `Front_MDW_wheel_link.STL` - Front mecanum drive wheel
- `IMU_link.STL` - IMU sensor
- `Lidar_link.STL` - Lidar sensor

## Usage

### Visualization in RViz

To visualize the robot in RViz:

```bash
roslaunch robot_description display.launch
```

### Simulation in Gazebo

To spawn the robot in Gazebo:

```bash
roslaunch robot_description gazebo.launch
```

## Customization

To customize the robot, edit the parameters in `properties.xacro` file. The most commonly adjusted parameters include:

- Wheel dimensions and positions
- Sensor positions
- Inertial properties

## Converting to URDF

If you need a pure URDF file (for tools that don't support XACRO), you can convert the XACRO to URDF using:

```bash
rosrun xacro xacro --inorder robot.xacro > robot.urdf
```

# ROS2 Navigation Setup

This package contains a setup for ROS2 Navigation2 (Nav2) for autonomous robot navigation.

## Prerequisites

- ROS2 (tested with Humble/Iron)
- Navigation2 packages installed
- A robot with odometry and laser scan data
- A map of the environment

## Usage

### 1. Launch the navigation stack

```bash
ros2 launch <your_package_name> navigation.launch.py
```

### 2. Set an initial pose

Use RViz2 to set the initial pose of the robot on the map using the "2D Pose Estimate" button.

### 3. Set a navigation goal

Use RViz2 to set a navigation goal using the "2D Nav Goal" button.

## Configuration

The navigation parameters are defined in `config/nav2_params.yaml`. You can modify these parameters to tune the navigation behavior for your specific robot.

Key areas to adjust:

- `controller_server`: Parameters for local planning and control
- `planner_server`: Parameters for global path planning
- `amcl`: Parameters for localization
- `local_costmap` and `global_costmap`: Parameters for obstacle avoidance

## Map Creation

If you need to create a map, you can use:

```bash
ros2 launch nav2_map_server map_saver_cli.launch.py save_map:=true map:=<map_name>
```

## Troubleshooting

- Ensure your robot's transform tree is properly set up (base_link, base_footprint, etc.)
- Verify that your laser scan data is being published on the correct topic
- Check that the map frame and odom frame are properly defined
- Make sure your navigation parameters match your robot's physical capabilities

# Robot Navigation Stack

This repository contains an optimized ROS 2 navigation package for robots with URDF models exported from SolidWorks. The code has been optimized to remove unnecessary components while maintaining full functionality.

## Optimizations Made

The codebase has been optimized in several ways:

1. **Removed Unnecessary Launch Files**:

   - Eliminated redundant simulation-specific launch files
   - Removed unused SLAM configuration files
   - Simplified launch structure to focus on essential components

2. **Streamlined Nav2 Parameters**:

   - Reduced parameters to only essential configurations
   - Removed redundant node configurations (e.g., `*_rclcpp_node` entries)
   - Optimized controller configurations for better performance
   - Eliminated duplicate and unused parameter sections

3. **Package Dependency Cleanup**:

   - Removed unnecessary dependencies to make the package lighter
   - Focused only on required navigation components

4. **Launch File Optimization**:
   - Removed unused imports in launch files
   - Simplified parameter passing between launch files
   - Improved readability and maintainability

## Package Structure

- **launch/** - Contains optimized launch files for running the robot navigation stack

  - **robot_state_publisher.launch.py** - Launches the robot state publisher with URDF model
  - **navigation.launch.py** - Launches the Navigation2 stack
  - **main.launch.py** - Main launch file that starts both robot state publisher and navigation

- **params/** - Contains optimized configuration parameters

  - **nav2_params.yaml** - Streamlined Navigation2 configuration parameters

- **urdf/** - Contains the robot URDF model

  - **robot.urdf** - Robot URDF model exported from SolidWorks

- **maps/** - Contains navigation maps
  - **map.yaml** - Map configuration
  - **map.pgm** - Map image file

## Running the Navigation Stack

To start the navigation stack:

```bash
ros2 launch urdf_export_assem_3_sldasm main.launch.py
```

## Launch Arguments

The launch files accept several arguments:

- `use_sim_time`: Set to 'true' to use simulation time (default: true)
- `map`: Path to the map YAML file (default: maps/map.yaml)
- `params_file`: Path to the navigation parameters file (default: params/nav2_params.yaml)
- `autostart`: Set to 'true' to automatically start the navigation stack (default: true)

Example with custom arguments:

```bash
ros2 launch urdf_export_assem_3_sldasm main.launch.py use_sim_time:=false map:=/path/to/custom/map.yaml
```

## Customization

To use your own URDF model:

1. Replace the `robot.urdf` file in the `urdf` directory with your model
2. Update the parameters in `nav2_params.yaml` to match your robot's configuration

## License

This project is licensed under the MIT License - see the LICENSE file for details.
