cmake_minimum_required(VERSION 3.8)
project(ros_cpp_practice)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(number_scaler_component SHARED src/number_scaler.cpp)
target_include_directories(number_scaler_component PUBLIC include)
ament_target_dependencies(number_scaler_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(number_scaler_component
  PLUGIN "robonav_training::NumberScaler"
  EXECUTABLE number_scaler)

install(TARGETS number_scaler_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()