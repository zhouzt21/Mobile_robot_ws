cmake_minimum_required(VERSION 3.5)
project(tracer_slam)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# cmake_policy(SET CMP0077 NEW)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}/
)
# if(BUILD_TESTING)
#   find_package(ament_link_auto REQUIRED)
#   ament_link_auto_find_test_dependencies()
# endif()

ament_package()
