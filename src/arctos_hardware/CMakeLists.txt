cmake_minimum_required(VERSION 3.16)
project(arctos_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  arctos_hardware
  SHARED
  hardware/rrbot_actuator.cpp
)
target_compile_features(arctos_hardware PUBLIC cxx_std_17)
target_include_directories(arctos_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/arctos_hardware>
)
ament_target_dependencies(
  arctos_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arctos_hardware_plugin.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/arctos_hardware
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/arctos_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/arctos_hardware
)
install(TARGETS arctos_hardware
  EXPORT export_arctos_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(arctos_hardware_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_arctos_hardware_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_arctos_hardware_launch test/test_rrbot_modular_actuators_launch.py)
endif()

## EXPORTS
ament_export_targets(export_arctos_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
