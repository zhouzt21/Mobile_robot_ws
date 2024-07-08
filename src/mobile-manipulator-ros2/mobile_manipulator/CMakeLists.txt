cmake_minimum_required(VERSION 3.5)

project(mobile_manipulator)


find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
