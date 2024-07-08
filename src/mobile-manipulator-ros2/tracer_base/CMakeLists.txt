cmake_minimum_required(VERSION 3.8)
project(tracer_base)
# Add flags to support compiling on windows, since ROS2 supports it
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
endif()
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# cmake module path
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
#    roslaunch
#    roslint
find_package(rclcpp REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tracer_msgs REQUIRED)
   #tracer_sdk
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ugv_sdk REQUIRED)
    
include_directories(include)

add_library(tracer_messenger STATIC src/tracer_messenger.cpp)
target_include_directories(tracer_messenger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tracer_messenger
  rclcpp
  std_msgs
  tracer_msgs
  ugv_sdk
  tf2_ros
  nav_msgs
)
set_property(TARGET tracer_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)
add_executable(tracer_base_node src/tracer_base_node.cpp)
target_include_directories(tracer_base_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# target_link_libraries(tracer_base_node tracer_messenger ${catkin_LIBRARIES})
ament_target_dependencies(tracer_base_node
rclcpp
tracer_msgs
ugv_sdk
)
target_link_libraries(tracer_base_node tracer_messenger)
install(TARGETS
tracer_base_node
tracer_messenger
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(tracer_messenger)
ament_package()
