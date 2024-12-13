cmake_minimum_required(VERSION 3.16)
project(arctos_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  arctos_motor_driver
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
  arctos_motor_driver
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
  arctos_motor_driver
)

# Find packages
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Include directories
include_directories(${AMENT_PREFIX_PATH}/include/arctos_motor_driver)

## COMPILE

# Add executable for the trajectory generator
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

# Add library for the hardware interface
add_library(
  arctos_hardware_interface
  SHARED
  hardware/arctos_hardware.cpp
  controller/arctos_controller.cpp
)

# Set C++17 as the required standard
target_compile_features(arctos_hardware_interface PUBLIC cxx_std_17)

# Set include directories for the hardware interface library
target_include_directories(arctos_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/arctos_hardware_interface>
  $<INSTALL_INTERFACE:include/arctos_motor_driver/include>
)

# Link dependencies using ament_target_dependencies
ament_target_dependencies(
  arctos_hardware_interface PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
  arctos_motor_driver
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arctos_hardware_interface.xml)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface arctos_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/arctos_hardware_interface
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/arctos_hardware_interface
)

install(
  TARGETS send_trajectory
  RUNTIME DESTINATION lib/arctos_hardware_interface
)

install(TARGETS arctos_hardware_interface
  EXPORT export_arctos_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_arctos_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})

ament_package()
