# Robot Navigation Stack - ROS 2 Humble

[![ROS 2](https://img.shields.io/badge/ROS-2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

A complete navigation stack for robots, fully converted from ROS 1 to ROS 2 Humble. This project includes a robot model from SolidWorks, differential drive control, and the Nav2 navigation system.

<p align="center">
  <img src="docs/robot_navigation.png" alt="Robot Navigation" width="600"/>
</p>

## Features

- **URDF Robot Model**: Complete URDF robot description from SolidWorks
- **Differential Drive Control**: Low-level robot control with odometry
- **Autonomous Navigation**: Full Nav2 integration with AMCL for localization
- **Simulation Ready**: Gazebo integration for virtual testing
- **Visualization**: RViz2 configuration for debugging and monitoring

## Installation

1. **Create a ROS 2 workspace**:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yourusername/urdf_export_assem_3_sldasm.git
```

2. **Install dependencies**:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package**:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch the full navigation stack

```bash
ros2 launch urdf_export_assem_3_sldasm main.launch.py
```

### Visualize the robot in RViz2

```bash
ros2 launch urdf_export_assem_3_sldasm display.launch.py
```

### Simulate in Gazebo

```bash
ros2 launch urdf_export_assem_3_sldasm gazebo.launch.py
```

## Structure

```
.
├── launch/                  # ROS 2 Python launch files
├── urdf_export_assem_3_sldasm/  # Python package
├── params/                  # Configuration parameters
├── urdf/                    # Robot URDF model
└── maps/                    # Navigation maps
```

## Documentation

For detailed documentation, please see:

- [ROS 2 Conversion](docs/ros2_conversion.md)
- [Navigation Setup](docs/navigation.md)
- [Customization Guide](docs/customization.md)

## Requirements

- ROS 2 Humble
- Nav2
- Gazebo 11 with ROS 2 plugins
- RViz2
- Python 3.8+

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
