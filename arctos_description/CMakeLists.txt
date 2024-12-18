cmake_minimum_required(VERSION 3.5)
project(arctos_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
