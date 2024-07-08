import os
import os.path as osp
from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    tracer_bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            osp.join(get_package_share_directory('tracer_base'), 'launch',
                     'tracer_base.launch.xml')))
    tracer_rviz_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            osp.join(get_package_share_directory('tracer_with_lidar'),
                     'launch', 'tracer_rviz.launch.py')))

    velodyne_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            osp.join(get_package_share_directory('velodyne'), 'launch',
                     'velodyne-all-nodes-VLP16-launch.py')))
    velodyne_scan_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            osp.join(get_package_share_directory('velodyne_laserscan'),
                     'launch', 'velodyne_laserscan_node-launch.py')))

    slam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            osp.join(get_package_share_directory('tracer_slam'), 'launch',
                     'online_async_launch.py')))

    ld.add_action(tracer_bringup_launch)
    ld.add_action(tracer_rviz_launch)
    ld.add_action(velodyne_launch)
    ld.add_action(velodyne_scan_launch)
    ld.add_action(slam_launch)

    return ld