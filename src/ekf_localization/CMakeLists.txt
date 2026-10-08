cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

find_package(ament_cmake REQUIRED)

# This package contributes no compiled code: it configures and launches the EKF
# from the robot_localization package. We only install the config and launch.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
