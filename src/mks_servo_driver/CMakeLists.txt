cmake_minimum_required(VERSION 3.8)
project(mks_servo_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

add_executable(mks_servo_driver src/mks_servo_driver.cpp)
ament_target_dependencies(mks_servo_driver rclcpp can_msgs)

install(TARGETS mks_servo_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
