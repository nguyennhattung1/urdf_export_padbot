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

# Robot Navigation Stack - ROS 2 Humble

Gói phần mềm này chứa đầy đủ các thành phần để điều khiển và điều hướng tự động cho robot, được chuyển đổi hoàn toàn từ ROS 1 sang ROS 2 Humble. Dự án bao gồm mô hình URDF từ SolidWorks, điều khiển differential drive, và hệ thống điều hướng Nav2.

## Chuyển đổi sang ROS 2

Tất cả các thành phần đã được chuyển đổi sang ROS 2:

1. **Launch Files**:

   - Từ định dạng XML (ROS 1) sang định dạng Python (ROS 2)
   - Thay thế tất cả các node ROS 1 bằng các node tương đương trong ROS 2

2. **Điều hướng**:

   - Thay thế move_base (ROS 1) bằng Nav2 (ROS 2)
   - Chuyển đổi AMCL để sử dụng tham số Nav2
   - Cập nhật global & local planners cho API của ROS 2

3. **Các Node Python**:
   - Cập nhật differential drive controller để sử dụng API node của ROS 2
   - Chuyển đổi TF sang TF2
   - Cập nhật cách publish các message theo mô hình ROS 2 với QoS

## Cấu trúc gói

- **launch/** - Chứa các file launch cho ROS 2

  - **robot_state_publisher.launch.py** - Khởi chạy robot state publisher
  - **navigation.launch.py** - Khởi chạy Nav2 stack
  - **main.launch.py** - File launch chính
  - **display.launch.py** - Launch chỉ để hiển thị
  - **gazebo.launch.py** - Launch mô phỏng Gazebo
  - **controller.launch.py** - Launch điều khiển differential drive
  - **includes/** - Các module launch phụ trợ
    - **amcl.launch.py** - Định vị
    - **nav2.launch.py** - Điều hướng

- **urdf_export_assem_3_sldasm/** - Gói Python

  - **differential_drive_controller.py** - Node điều khiển ROS 2

- **params/** - Chứa các tham số cấu hình

  - **nav2_params.yaml** - Tham số cấu hình Nav2

- **urdf/** - Chứa mô hình URDF của robot

  - **robot.urdf** - Mô hình URDF robot từ SolidWorks
  - **robot.xacro** - Mô hình XACRO robot
  - **properties.xacro** - Tham số robot
  - **materials.xacro** - Định nghĩa vật liệu
  - **macros.xacro** - Định nghĩa link và joint tái sử dụng

- **maps/** - Chứa bản đồ cho điều hướng
  - **map.yaml** - Cấu hình bản đồ
  - **map.pgm** - File hình ảnh bản đồ

## Cài đặt

```bash
# Tạo workspace ROS 2
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone <đường_dẫn_repository> urdf_export_assem_3_sldasm

# Cài đặt dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --symlink-install
source install/setup.bash
```

## Sử dụng

### Khởi động hệ thống điều hướng

```bash
ros2 launch urdf_export_assem_3_sldasm main.launch.py
```

### Chỉ hiển thị robot trong RViz2

```bash
ros2 launch urdf_export_assem_3_sldasm display.launch.py
```

### Mô phỏng trong Gazebo với điều hướng

```bash
ros2 launch urdf_export_assem_3_sldasm ros1_navigation.launch.py
```

## Tham số Launch

Các file launch chấp nhận các tham số sau:

- `use_sim_time`: Sử dụng thời gian mô phỏng (mặc định: true)
- `map`: Đường dẫn đến file bản đồ YAML (mặc định: maps/map.yaml)
- `params_file`: Đường dẫn đến file tham số điều hướng
- `autostart`: Tự động khởi động stack điều hướng

## Tùy chỉnh

### Sử dụng mô hình URDF riêng

1. Thay thế file `robot.urdf` trong thư mục `urdf/`
2. Cập nhật các tham số trong `params/nav2_params.yaml` để phù hợp với robot của bạn

### Điều chỉnh tham số điều hướng

Các tham số điều hướng trong `params/nav2_params.yaml` có thể được điều chỉnh:

- `controller_server`: Tham số cho điều khiển di chuyển
- `planner_server`: Tham số cho lập kế hoạch đường đi
- `amcl`: Tham số cho định vị
- `local_costmap` và `global_costmap`: Tham số cho tránh chướng ngại vật

## Tạo bản đồ mới

Nếu bạn cần tạo bản đồ mới, sử dụng:

```bash
ros2 launch nav2_map_server map_saver_cli.launch.py save_map:=true map:=<tên_bản_đồ>
```

## Dependencies

- ROS 2 Humble
- Nav2
- Gazebo 11 với plugin ROS 2
- RViz2
- Python 3.8+

## Xử lý sự cố

- Đảm bảo cây biến đổi của robot được thiết lập đúng (base_link, base_footprint, v.v.)
- Kiểm tra dữ liệu laser scan được xuất bản trên topic đúng
- Kiểm tra frame map và odom được định nghĩa đúng
- Đảm bảo các tham số điều hướng phù hợp với robot của bạn

## License

Dự án này được cấp phép theo giấy phép MIT - xem file LICENSE để biết chi tiết.
