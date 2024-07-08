cmake_minimum_required(VERSION 3.8.10)

project(tracer_with_lidar)


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
#add_subdirectory(catkin_simple)

#find_package(ros2launch)

#foreach(dir config launch meshes urdf)
# install(TARGETS 
#   config
# 	DESTINATION share/${PROJECT_NAME}/)

# install(TARGETS 
#   launch
# 	DESTINATION share/${PROJECT_NAME}/)

# install(TARGETS 
#   meshes
# 	DESTINATION share/${PROJECT_NAME}/)

# install(TARGETS 
#   urdf
# 	DESTINATION share/${PROJECT_NAME}/)

# foreach(dir config launch meshes urdf)
#   install(DIRECTORY ${dir}/
# 		DESTINATION lib/${dir})
# endforeach(dir)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
meshes 
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
urdf
DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY
# rviz2 
# DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY
# param
# DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include)
#endforeach(dir)
ament_package()