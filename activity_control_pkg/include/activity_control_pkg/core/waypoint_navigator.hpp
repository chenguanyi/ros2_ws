#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct WaypointTarget
{
  double x_cm{0.0};
  double y_cm{0.0};
  double z_cm{0.0};
  double yaw_deg{0.0};
};

struct WaypointPolicy
{
  std::optional<double> position_tolerance_cm;
  std::optional<double> height_tolerance_cm;
  std::optional<double> yaw_tolerance_deg;
  bool enable_controller{true};
};

class WaypointNavigator
{
public:
  explicit WaypointNavigator(rclcpp::Node & node);

  void goTo(
    double x_cm,
    double y_cm,
    double z_cm,
    double yaw_deg,
    const WaypointPolicy & policy = WaypointPolicy{});
  void goTo(const WaypointTarget & target, const WaypointPolicy & policy = WaypointPolicy{});
  bool isReached();
  bool isReached(const WaypointTarget & target, const WaypointPolicy & policy = WaypointPolicy{});
  bool hold(double seconds);
  void cancel();
  void publishActiveController(std::uint8_t controller_id);

  bool hasState();
  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  std::optional<WaypointTarget> currentTarget() const;

private:
  double normalizeAngleDeg(double angle_deg) const;
  bool isReached(
    const WaypointTarget & target,
    const WaypointPolicy & policy,
    double x_cm,
    double y_cm,
    double z_cm,
    double yaw_deg) const;
  double positionTolerance(const WaypointPolicy & policy) const;
  double heightTolerance(const WaypointPolicy & policy) const;
  double yawTolerance(const WaypointPolicy & policy) const;
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void publishTarget(const WaypointTarget & target, const WaypointPolicy & policy);

  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;
  std::string height_topic_;
  double default_position_tolerance_cm_;
  double default_height_tolerance_cm_;
  double default_yaw_tolerance_deg_;
  bool log_waypoint_targets_{true};
  double current_height_cm_{0.0};
  bool has_height_{false};
  std::optional<WaypointTarget> current_target_;
  WaypointPolicy current_policy_;
  std::optional<rclcpp::Time> hold_until_;
};

}  // namespace activity_control_pkg
