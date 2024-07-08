#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include "tracer_base/tracer_messenger.hpp"
#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

using namespace westonrobot;

std::unique_ptr<TracerRobot> robot;

int main(int argc, char **argv)
{
    // setup ROS node
    // ros::init(argc, argv, "tracer_base");
    // ros::NodeHandle node(""), private_node("~");
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("n");
    // rclcpp::Node::SharedPtr private_node = rclcpp::Node::make_shared("pn");

    robot = std::unique_ptr<TracerRobot>(new TracerRobot());
    if (robot == nullptr)
        std::cout << "Failed to create robot object" << std::endl;

    TracerROSMessenger messenger(robot.get(), node);
    //TracerROSMessenger messenger(node);
    // fetch parameters before connecting to robot
    std::string port_name;
    node->get_parameter_or("port_name", port_name, std::string("can0"));
    node->get_parameter_or("odom_frame", messenger.odom_frame_, std::string("odom"));
    node->get_parameter_or("base_frame", messenger.base_frame_, std::string("base_link"));
    node->get_parameter_or("simulated_robot", messenger.simulated_robot_, false);

    RCLCPP_INFO(node->get_logger(), "odom_frame_: %s, base_frame_: %s", messenger.odom_frame_.c_str(), messenger.base_frame_.c_str());
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
        try
        {
            robot->Connect(port_name);
        }
        catch (std::exception error)
        {
            RCLCPP_ERROR(node->get_logger(),"please bringup up can or make sure can port exist");
            rclcpp::shutdown();
        }
        robot->EnableCommandedMode();
        RCLCPP_INFO(node->get_logger(), "Using CAN bus to talk with the robot");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Using UART to talk with the robot");
    }
    messenger.SetupSubscription();
    RCLCPP_INFO(node->get_logger(), "######## Start ########");
    // publish robot state at 50Hz while listening to twist commands
    // ros::Rate rate_50hz(50);  // 50Hz
    rclcpp::Rate rate_50hz(50);
    //int cnt = 0;
    while (rclcpp::ok()) {
      if (port_name.find("can") != std::string::npos)
          messenger.PublishStateToROS();
     // else  messenger.PublishUartStateToROS();
    //   ros::spinOnce();
      rclcpp::spin_some(node);
    //   rclcpp::spin_some(private_node);
      rate_50hz.sleep();
    }
    return 0;
}
