cmake_minimum_required(VERSION 3.8)
project(arctos_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)

# Add the hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/single_joint_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_msgs
  ros2_socketcan
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface single_actuator_interface.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_msgs
  ros2_socketcan
)

ament_package()