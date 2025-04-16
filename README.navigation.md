# Robot Navigation

This package includes support for navigating the robot in a known map using the ROS navigation stack.

## Prerequisites

Make sure you have the following ROS packages installed:

```bash
sudo apt-get install ros-<distro>-navigation ros-<distro>-map-server ros-<distro>-amcl
```

Replace `<distro>` with your ROS distribution (e.g., noetic, melodic).

## Map Setup

The package comes with a simple example map for testing purposes. You can create your own maps using:

1. **SLAM (Simultaneous Localization and Mapping):** Use gmapping or other SLAM algorithms
2. **Map_saver:** Save a map created during SLAM

```bash
# Example for saving a map
rosrun map_server map_saver -f my_map
```

Place your map files (PGM and YAML) in the `maps` directory.

## Navigation Launch Files

The package provides several launch files for navigation:

1. **navigation.launch** - Main navigation file that loads the map and starts the navigation stack

   ```bash
   roslaunch robot_description navigation.launch
   ```

   Parameters:

   - `map_file` - Path to the map YAML file (default: robot_description/maps/map.yaml)
   - `initial_pose_x`, `initial_pose_y`, `initial_pose_a` - Initial robot position and orientation
   - `use_sim_time` - Whether to use simulation time (default: true)
   - `gui` - Whether to launch RViz (default: true)

## Navigation Parameters

Navigation parameters are stored in the `param` directory:

- `costmap_common.yaml` - Common parameters for both global and local costmaps
- `costmap_global.yaml` - Global costmap parameters
- `costmap_global_static.yaml` - Static map layer parameters
- `costmap_local.yaml` - Local costmap parameters
- `planner.yaml` - Parameters for global and local planners

You can customize these files to tune the navigation for your robot's specific needs.

## Setting Navigation Goals

In RViz, you can set navigation goals for the robot using the "2D Nav Goal" button. Click on the map to set the goal position and drag to set the orientation.

## Troubleshooting

If the robot fails to navigate:

1. Check that the map is properly loaded
2. Verify that the laser scan data is being published on the `/scan` topic
3. Check that the TF tree is properly configured (robot → odom → map)
4. Adjust costmap parameters for your robot size and sensor range
5. Try tuning the local planner parameters for smoother navigation
