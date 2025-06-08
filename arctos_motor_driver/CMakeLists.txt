cmake_minimum_required(VERSION 3.5)
project(arctos_motor_driver)

# Set compiler options with C++17 support
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ============================================================can protocol====================================================
# Create library for CAN protocol
add_library(can_protocol SHARED
  src/can_protocol.cpp
)

target_include_directories(can_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(can_protocol PUBLIC cxx_std_17)  # Explicitly set C++17

target_link_libraries(can_protocol
  Threads::Threads
)

ament_target_dependencies(can_protocol
  rclcpp
  can_msgs
)

# Create the main motor driver library
add_library(${PROJECT_NAME} SHARED
  src/motor_driver.cpp
)

# Link against CAN protocol library and Threads
target_link_libraries(${PROJECT_NAME}
  can_protocol
  Threads::Threads
)

# ============================================================motor driver====================================================

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Explicitly set C++17

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  can_msgs
  sensor_msgs
)

# Install targets
install(TARGETS 
    can_protocol
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html
install(DIRECTORY include/
  DESTINATION include
)

# Install config files if needed
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/
#   OPTIONAL
# )

# Export dependencies and targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  can_msgs
  sensor_msgs
  Threads
)

ament_package()