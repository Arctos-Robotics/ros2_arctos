cmake_minimum_required(VERSION 3.5)
project(arctos_hardware_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arctos_motor_driver REQUIRED)  # Ensure this line is present

# Add library for hardware interface
add_library(arctos_interface SHARED
  src/arctos_interface.cpp
)

# Link the hardware interface libraries and the motor driver
target_link_libraries(arctos_interface
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES} 
)

# Include directories
target_include_directories(arctos_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Specify dependencies
ament_target_dependencies(arctos_interface
  rclcpp
  can_msgs
  sensor_msgs
  arctos_motor_driver  
  hardware_interface
  trajectory_msgs
  realtime_tools
)

# Install targets
install(TARGETS arctos_interface
  EXPORT export_arctos_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ros2_control
  DESTINATION share/${PROJECT_NAME}/ros2_control
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(FILES arctos_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_targets(export_arctos_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp can_msgs sensor_msgs hardware_interface)

ament_package()
