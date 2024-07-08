import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share=launch_ros.substitutions.FindPackageShare(package='tracer_with_lidar').find('tracer_with_lidar')
    urdf_path =os.path.join(pkg_share, 'urdf/tracer_with_lidar.urdf')
    rviz_path =os.path.join(pkg_share, 'urdf/default.rviz')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}
    
    
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )

    static_tf_node = launch_ros.actions.Node(
        name='odom_2_base',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0.0', '0.0', '0.0', 'odom', 'base_link'],   
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )
 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_path,
                                             description='Absolute path to robot urdf file'),
        
        robot_state_publisher_node,
        static_tf_node,
        rviz_node
    ])