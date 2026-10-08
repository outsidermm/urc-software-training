#pragma once

#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robonav_training
{
class WheelOdometry : public rclcpp::Node {
    public:
        explicit WheelOdometry(const rclcpp::NodeOptions & options);

    private:
        void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
        void publishOdometry(const rclcpp::Time & stamps, double vx, double wz);
        double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
        double wheel_radius_, wheel_separation_;
        double left_wheel_pos_, right_wheel_pos_;
        bool has_previous_ = false;
        std::string left_joint_name_, right_joint_name_;
        rclcpp::Time stamp_prev_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

}