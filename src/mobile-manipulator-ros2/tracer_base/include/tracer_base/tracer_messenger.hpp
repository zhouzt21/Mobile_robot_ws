/* 
 * Tracer_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "tracer_msgs/msg/tracer_light_cmd.hpp"
#include "tracer_msgs/msg/tracer_status.hpp"
#include "tracer_msgs/msg/uart_tracer_status.hpp"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
namespace westonrobot
{

class TracerROSMessenger: protected TracerRobot
{
public:
    TracerROSMessenger(std::shared_ptr<rclcpp::Node> nh);
    TracerROSMessenger(TracerRobot *Tracer, std::shared_ptr<rclcpp::Node> nh);

    std::string odom_frame_;
    std::string base_frame_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();
    void PublishUartStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);
    void DetachRobot();

private:
    TracerRobot *tracer_;
    std::shared_ptr<rclcpp::Node> nh_;

    std::mutex twist_mutex_;
    geometry_msgs::msg::Twist current_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<tracer_msgs::msg::TracerStatus>::SharedPtr status_publisher_;
    rclcpp::Publisher<tracer_msgs::msg::UartTracerStatus>::SharedPtr status_uart_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
    rclcpp::Subscription<tracer_msgs::msg::TracerLightCmd>::SharedPtr light_cmd_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Clock _ros_clock;
    rclcpp::Time last_time_;
    rclcpp::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void LightCmdCallback(const tracer_msgs::msg::TracerLightCmd::ConstSharedPtr msg);
    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace wescore

#endif /* TRACER_MESSENGER_HPP */
