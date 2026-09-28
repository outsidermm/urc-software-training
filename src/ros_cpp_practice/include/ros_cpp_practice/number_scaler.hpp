#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace robonav_training
{

class NumberScaler : public rclcpp::Node
{
public:
  explicit NumberScaler(const rclcpp::NodeOptions & options);

private:
  void onNumber(const std_msgs::msg::Float64::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_sub_;
  double scale_;
};

}  // namespace robonav_training