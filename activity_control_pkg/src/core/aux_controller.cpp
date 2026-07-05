#include "activity_control_pkg/core/aux_controller.hpp"

namespace activity_control_pkg
{

AuxController::AuxController(rclcpp::Node & node)
: node_(node)
{
  arm_topic_ = node_.declare_parameter("arm_command_topic", std::string("/arm_command"));
  magnet_topic_ = node_.declare_parameter("magnet_command_topic", std::string("/magnet_command"));
  signal_topic_ = node_.declare_parameter("signal_command_topic", std::string("/signal_command"));

  arm_pub_ = node_.create_publisher<std_msgs::msg::UInt8>(arm_topic_, rclcpp::QoS(10).reliable());
  magnet_pub_ = node_.create_publisher<std_msgs::msg::UInt8>(magnet_topic_, rclcpp::QoS(10).reliable());
  signal_pub_ = node_.create_publisher<std_msgs::msg::UInt8>(signal_topic_, rclcpp::QoS(10).reliable());
}

void AuxController::setArm(bool enabled)
{
  publish(arm_pub_, enabled ? 1U : 0U);
}

void AuxController::setMagnet(bool enabled)
{
  publish(magnet_pub_, enabled ? 1U : 0U);
}

void AuxController::setSignal(bool enabled)
{
  publish(signal_pub_, enabled ? 1U : 0U);
}

void AuxController::setAll(
  std::uint8_t arm_state,
  std::uint8_t magnet_state,
  std::uint8_t signal_state)
{
  publish(arm_pub_, arm_state);
  publish(magnet_pub_, magnet_state);
  publish(signal_pub_, signal_state);
  RCLCPP_DEBUG(
    node_.get_logger(),
    "Published aux states: arm=%u magnet=%u signal=%u",
    static_cast<unsigned>(arm_state),
    static_cast<unsigned>(magnet_state),
    static_cast<unsigned>(signal_state));
}

void AuxController::publish(
  const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & publisher,
  std::uint8_t value)
{
  std_msgs::msg::UInt8 message;
  message.data = value == 0U ? 0U : 1U;
  publisher->publish(message);
}

}  // namespace activity_control_pkg
