cmake_minimum_required(VERSION 3.8)

project(urdf_export_assem_3_sldasm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(nav2_common REQUIRED)

# Install launch files
install(DIRECTORY
	launch
	params
	urdf
	maps
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
