#include "ros_cpp_practice/number_scaler.hpp"

#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace robonav_training
{

NumberScaler::NumberScaler(const rclcpp::NodeOptions & options)
: Node("number_scaler", options)
{
  scale_ = declare_parameter<double>("scale", 2.0);
  output_pub_ = create_publisher<std_msgs::msg::Float64>("/practice/output", 10);
  input_sub_ = create_subscription<std_msgs::msg::Float64>("/practice/input", 10, std::bind(&NumberScaler::onNumber, this, std::placeholders::_1));
}

void NumberScaler::onNumber(const std_msgs::msg::Float64::SharedPtr msg) {
  std_msgs::msg::Float64 output;
  output.data = msg->data * scale_;
  output_pub_->publish(output);
}

}  // namespace robonav_training

RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::NumberScaler)