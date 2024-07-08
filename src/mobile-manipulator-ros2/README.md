# ROS2 Packages for Tracer Mobile Base

## Packages

* tracer_base: a ROS wrapper around tracer SDK to monitor and control the robot
* tracer_bringup: launch and configuration files to start ROS nodes
* tracer_msgs: tracer related message definitions
* ugv_sdk: communication interface. [README](https://github.com/agilexrobotics/ugv_sdk#hardware-interface) 


## Basic usage of the ROS2 package

1. Install dependent packages [needs check]

    ```
    $ sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
    $ sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt install ros-$ROS_DISTRO-ros-controllers
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/ros2_tracer_ws/src
    $ git clone https://gitee.com/tsinghua_ral/tracer-ros2.git
    $ cd ..
    $ colcon build
    ```

3. Setup CAN-To-USB adapter
* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
* first time set up can2usb
    ```
    $ bash src/tracer_ros2/tracer_bringup/scripts/setup_can2usb.bash
    ```
* If not the first time bring up can2usb(Run this command every time you turn on the power)
    ```
    $ bash src/tracer_ros2/tracer_bringup/scripts/bring_can2usb.bash
    ```
4. Launch ROS nodes

* Start the base node for the real robot whith can

    ```
    $ ros2 launch tracer_base tracer_base.launch.xml
    ```
* Start the keyboard tele-op node

    ```
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
## Slam_toolbox
* Bring up tracer, lidar, and run slam_toolbox
    ```
    $ ros2 launch tracer_with_lidar slam_with_lidar.launch.py
    ```

## Mobile Manipulator URDF
* 
    ```
    $ ros2 launch mobile_manipulator display.launch.py
    ```
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 

