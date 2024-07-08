<launch>
  <include
    file="$(find-pkg-share gazebo_ros)/launch/empty_world.launch" />
  <node
    name="tf_footprint_base"
    pkg="tf"
    exec="static_transform_publisher"
    args="0 0 0 0 0 0 base_link base_footprint 40" />
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    exec="spawn_model"
    args="-file $(find-pkg-share tracer_with_lidar)/urdf/tracer_with_lidar.urdf -urdf -model tracer_with_lidar"
    output="screen" />
  <node
    name="fake_joint_calibration"
    pkg="rostopic"
    exec="rostopic"
    args="pub /calibrated std_msgs/Bool true" />
</launch>