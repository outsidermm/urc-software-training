cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robonav_training_common REQUIRED)

include_directories(include)

# A single composable component. The wheel-odometry math is short enough to live
# right in the node, so there is no separate algorithm-core library here.
# rclcpp_components_register_node ALSO emits a standalone `wheel_odometry`
# executable for `ros2 run wheel_odometry wheel_odometry`.
add_library(wheel_odometry_component SHARED src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry_component
  rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs robonav_training_common)
rclcpp_components_register_node(wheel_odometry_component
  PLUGIN "robonav_training::WheelOdometry"
  EXECUTABLE wheel_odometry)

install(TARGETS wheel_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()