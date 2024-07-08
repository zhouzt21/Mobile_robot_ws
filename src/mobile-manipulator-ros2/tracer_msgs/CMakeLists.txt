cmake_minimum_required(VERSION 3.8)
project(tracer_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
  # std_msgs
  # message_generation
# )

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(message_generation REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TracerMotorState.msg"
  "msg/UartTracerMotorState.msg"
  "msg/TracerLightState.msg"
  "msg/TracerLightCmd.msg"
  "msg/TracerStatus.msg"
  "msg/UartTracerStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
 )
 ament_export_dependencies(rosidl_default_runtime)
 ament_package()