<launch>
   <arg name="model" />
   <param name="robot_description" textfile="$(find tracer_with_lidar)/urdf/tracer_with_lidar.urdf" />
   <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" />
   <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher" />
   <node name="rviz2" pkg="rviz2" exec="rviz2" args="-d $(find tracer_with_lidar)/urdf.rviz" />
</launch>