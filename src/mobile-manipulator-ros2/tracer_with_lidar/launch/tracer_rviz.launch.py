import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = get_package_share_directory('tracer_with_lidar')
    urdf_path = os.path.join(pkg_share, 'urdf/tracer_with_lidar.urdf')
    rviz_path = os.path.join(pkg_share, 'urdf/default.rviz')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      parameters=[{
                                          'use_sim_time':
                                          use_sim_time,
                                          'robot_description':
                                          robot_desc
                                      }],
                                      arguments=[urdf_path])
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument('model',
                              default_value=urdf_path,
                              description='Absolute path to robot urdf file'),
        robot_state_publisher_node, 
        rviz_node, 
    ])