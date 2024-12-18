cmake_minimum_required(VERSION 3.5)
project(arctos_motor_driver_test)

# Set compiler options (optional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arctos_motor_driver REQUIRED)

add_executable(arctos_motor_driver_test src/arctos_motor_driver_test.cpp)


# Specify dependencies
ament_target_dependencies(arctos_motor_driver_test
  rclcpp
  can_msgs
  sensor_msgs
  arctos_motor_driver
)
install(TARGETS
  arctos_motor_driver_test
  DESTINATION lib/${PROJECT_NAME}
)
# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export package information
ament_package()
