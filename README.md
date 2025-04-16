# Robot Simulation Package - ROS 2 Humble

Gói phần mềm này mô phỏng robot di động với kiểu dẫn động differential drive và mecanum wheel sử dụng ROS 2 Humble. Package được xuất từ mô hình SolidWorks và bao gồm đầy đủ các thành phần để điều khiển, mô phỏng và điều hướng tự động.

## Cấu trúc gói

- **launch/** - Chứa các file launch cho ROS 2

  - **robot_state_publisher.launch.py** - Khởi chạy robot state publisher
  - **gazebo.launch.py** - Launch mô phỏng Gazebo
  - **rviz.launch.py** - Hiển thị robot trong RViz2
  - **simulation.launch.py** - Chạy đồng thời Gazebo và RViz2
  - **navigation.launch.py** - Khởi chạy Nav2 stack
  - **main.launch.py** - File launch tổng hợp (robot + navigation)
  - **controller.launch.py** - Điều khiển differential drive

- **urdf/** - Chứa mô hình URDF của robot

  - **robot.urdf** - Mô hình URDF robot từ SolidWorks

- **meshes/** - Chứa các file mesh STL (không được đưa vào Git)

  - Người dùng cần tải riêng thư mục này do kích thước lớn

- **worlds/** - Chứa các file world cho Gazebo

  - **empty.world** - Môi trường Gazebo trống

- **rviz/** - Chứa cấu hình RViz2

  - **robot.rviz** - Cấu hình RViz2 cho robot

- **config/** và **param/** - Chứa các tham số cấu hình

  - **nav2_params.yaml** - Tham số Nav2

- **maps/** - Chứa bản đồ cho điều hướng

## Cài đặt

### Yêu cầu

- Ubuntu 22.04 (hoặc phiên bản tương thích với ROS 2 Humble)
- ROS 2 Humble
- Gazebo 11
- Navigation2 packages

### Cài đặt ROS 2 Humble (nếu chưa có)

```bash
# Thêm repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

# Cài đặt ROS 2 Humble Desktop
sudo apt install ros-humble-desktop

# Cài đặt các gói cần thiết
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs
```

### Cài đặt package

```bash
# Tạo workspace ROS 2
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/nguyennhattung1/urdf_export_padbot.git urdf_export_assem_3_sldasm

# Tải thư mục meshes (liên hệ tác giả để lấy thư mục này)
# Đặt thư mục meshes vào trong thư mục gốc của package

# Cài đặt dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --symlink-install
source install/setup.bash
```

## Sử dụng

### 1. Chạy mô phỏng đơn giản

Để hiển thị robot trong RViz2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_export_assem_3_sldasm rviz.launch.py
```

### 2. Chạy mô phỏng với Gazebo

Để chạy robot trong môi trường Gazebo:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_export_assem_3_sldasm gazebo.launch.py
```

### 3. Chạy đồng thời Gazebo và RViz2

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_export_assem_3_sldasm simulation.launch.py
```

### 4. Chạy với Navigation2

Để chạy robot với hệ thống điều hướng Nav2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_export_assem_3_sldasm navigation.launch.py
```

### 5. Chạy toàn bộ hệ thống (robot + navigation)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_export_assem_3_sldasm main.launch.py
```

## Sử dụng điều hướng

1. Khởi động Navigation2 như hướng dẫn ở trên
2. Trong RViz2, sử dụng nút "2D Pose Estimate" để đặt vị trí ban đầu của robot trên bản đồ
3. Sử dụng nút "2D Nav Goal" để đặt điểm đích cho robot
4. Robot sẽ tự động di chuyển đến điểm đích

## Tạo bản đồ mới

Nếu bạn cần tạo bản đồ mới cho điều hướng:

```bash
# Chạy robot với SLAM
ros2 launch nav2_bringup slam_launch.py

# Lưu bản đồ
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/urdf_export_assem_3_sldasm/maps/my_map
```

## Tùy chỉnh

### Thay đổi tham số Navigation2

Các tham số điều hướng được lưu trong file `param/nav2_params.yaml`. Bạn có thể điều chỉnh:

- Tham số điều khiển (controller_server)
- Tham số lập kế hoạch đường đi (planner_server)
- Tham số định vị (amcl)
- Tham số bản đồ chi phí (costmap)

### Thay đổi môi trường Gazebo

Bạn có thể thay đổi môi trường Gazebo bằng cách:

1. Thêm file world mới vào thư mục `worlds/`
2. Chỉnh sửa file `gazebo.launch.py` để sử dụng file world mới

## Xử lý sự cố

### Không thấy robot trong Gazebo hoặc RViz2

- Kiểm tra xem đã có thư mục `meshes/` với đầy đủ các file STL chưa
- Đảm bảo đã source workspace: `source ~/ros2_ws/install/setup.bash`
- Kiểm tra không có lỗi trong log: `ros2 log info`

### Vấn đề với Navigation2

- Đảm bảo TF tree chính xác (kiểm tra trong RViz2)
- Kiểm tra topic laser scan đã được publish đúng cách
- Kiểm tra các tham số trong `param/nav2_params.yaml` phù hợp với robot

## Contributors

- Nguyễn Nhật Tùng (nguyennhattung1)

## License

MIT License

---

_Lưu ý: Do kích thước lớn của các file mesh, chúng không được đưa vào Git repository. Vui lòng liên hệ tác giả để lấy thư mục `meshes/` đầy đủ._
