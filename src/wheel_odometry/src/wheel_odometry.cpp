#include "wheel_odometry/wheel_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robonav_training_common/angles.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace robonav_training {
    WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options) : Node("wheel_odometry", options) {
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.34);
        left_joint_name_ = declare_parameter<std::string>("left_joint_name", "left_wheel_joint");
        right_joint_name_ = declare_parameter<std::string>("right_joint_name", "right_wheel_joint");

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&WheelOdometry::onJointState, this, std::placeholders::_1));
    }

    static int findJointIndex(
    const sensor_msgs::msg::JointState & msg, const std::string & name)
    {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == name) {
        return static_cast<int>(i);
        }
    }
    return -1;
    }

    void WheelOdometry::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
        const int left_idx = findJointIndex(*msg, left_joint_name_);
        const int right_idx = findJointIndex(*msg, right_joint_name_);
        const int pos_cnt = static_cast<int>(msg->position.size());
        if (left_idx < 0 || right_idx < 0 || left_idx >= pos_cnt || right_idx >= pos_cnt) return;
        
        const rclcpp::Time stamp_cur(msg->header.stamp);
        
        double l_now = msg->position[left_idx];
        double r_now = msg->position[right_idx];

        if(!has_previous_) {
            left_wheel_pos_ = l_now;
            right_wheel_pos_ = r_now;
            stamp_prev_ = stamp_cur;
            has_previous_ = true;
            publishOdometry(stamp_cur, 0.0, 0.0);
            return ;
        }

        const double dt = (stamp_cur - stamp_prev_).seconds();
        if (dt <= 0) return ;

        double l_dis = (l_now - left_wheel_pos_) * wheel_radius_;
        double r_dis = (r_now - right_wheel_pos_) * wheel_radius_;
        double c_dis = (l_dis + r_dis) / 2;
        double theta_dis = (r_dis - l_dis) / wheel_separation_;

        double mid_theta = theta_ + theta_dis / 2;
        x_ += c_dis * cos(mid_theta);
        y_ += c_dis * sin(mid_theta);
        theta_ = normalizeAngle(theta_ + theta_dis);

        double vx = c_dis / dt;
        double wz = theta_dis / dt;

        publishOdometry(stamp_cur, vx, wz);

        stamp_prev_ = stamp_cur;
        left_wheel_pos_ = l_now;
        right_wheel_pos_ = r_now;
    }

    void WheelOdometry::publishOdometry(const rclcpp::Time & stamp, double vx, double wz) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = wz;

        odom.pose.covariance[0] = 1e-3;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[35] = 1e-2;
        odom.twist.covariance[0] = 1e-3;
        odom.twist.covariance[35] = 1e-2;

        odom_pub_->publish(odom);
    }


}

RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::WheelOdometry);