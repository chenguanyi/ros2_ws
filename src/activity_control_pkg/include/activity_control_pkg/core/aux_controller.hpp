#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace activity_control_pkg
{

class AuxController
{
public:
  explicit AuxController(rclcpp::Node & node);

  void setArm(bool enabled);
  void setMagnet(bool enabled);
  void setSignal(bool enabled);
  void setAll(std::uint8_t arm_state, std::uint8_t magnet_state, std::uint8_t signal_state);

private:
  void publish(const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & publisher, std::uint8_t value);

  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr magnet_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr signal_pub_;
  std::string arm_topic_;
  std::string magnet_topic_;
  std::string signal_topic_;
};

}  // namespace activity_control_pkg
