#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <angles/angles.h>

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

WaypointNavigator::WaypointNavigator(rclcpp::Node & node)
: node_(node)
{
  map_frame_ = node_.declare_parameter("map_frame", std::string("map"));
  laser_link_frame_ = node_.declare_parameter("laser_link_frame", std::string("laser_link"));
  output_topic_ = node_.declare_parameter("output_topic", std::string("/target_position"));
  height_topic_ = node_.declare_parameter("height_topic", std::string("/height"));
  default_position_tolerance_cm_ = node_.declare_parameter("position_tolerance_cm", 8.0);
  default_height_tolerance_cm_ = node_.declare_parameter("height_tolerance_cm", 6.0);
  default_yaw_tolerance_deg_ = node_.declare_parameter("yaw_tolerance_deg", 5.0);
  log_waypoint_targets_ = node_.declare_parameter("log_waypoint_targets", true);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = node_.create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = node_.create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  height_sub_ = node_.create_subscription<std_msgs::msg::Int16>(
    height_topic_,
    rclcpp::QoS(10),
    std::bind(&WaypointNavigator::heightCallback, this, std::placeholders::_1));
}

void WaypointNavigator::goTo(
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg,
  const WaypointPolicy & policy)
{
  goTo(WaypointTarget{x_cm, y_cm, z_cm, yaw_deg}, policy);
}

void WaypointNavigator::goTo(const WaypointTarget & target, const WaypointPolicy & policy)
{
  current_target_ = target;
  current_policy_ = policy;
  hold_until_.reset();
  publishTarget(target, policy);
}

bool WaypointNavigator::isReached()
{
  if (!current_target_.has_value()) {
    return false;
  }
  return isReached(*current_target_, current_policy_);
}

bool WaypointNavigator::isReached(const WaypointTarget & target, const WaypointPolicy & policy)
{
  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return false;
  }
  return isReached(target, policy, x_cm, y_cm, z_cm, yaw_deg);
}

bool WaypointNavigator::hold(double seconds)
{
  if (seconds <= 0.0) {
    hold_until_.reset();
    return true;
  }

  const auto now = node_.now();
  if (!hold_until_.has_value()) {
    hold_until_ = now + rclcpp::Duration::from_seconds(seconds);
    RCLCPP_INFO(node_.get_logger(), "Holding current waypoint for %.2fs.", seconds);
    return false;
  }

  if (now < *hold_until_) {
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      1000,
      "Holding current waypoint for %.2fs more.",
      (*hold_until_ - now).seconds());
    return false;
  }

  hold_until_.reset();
  return true;
}

void WaypointNavigator::cancel()
{
  current_target_.reset();
  hold_until_.reset();
  publishActiveController(3);
}

void WaypointNavigator::publishActiveController(std::uint8_t controller_id)
{
  std_msgs::msg::UInt8 message;
  message.data = controller_id;
  active_controller_pub_->publish(message);
}

bool WaypointNavigator::hasState()
{
  if (!has_height_) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      2000,
      "Waiting for %s before starting mission task.",
      height_topic_.c_str());
    return false;
  }

  if (!tf_buffer_->canTransform(map_frame_, laser_link_frame_, tf2::TimePointZero)) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      2000,
      "Waiting for TF %s -> %s before starting mission task.",
      map_frame_.c_str(),
      laser_link_frame_.c_str());
    return false;
  }

  return true;
}

bool WaypointNavigator::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  if (!has_height_) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      2000,
      "Waiting for %s before evaluating waypoint progress.",
      height_topic_.c_str());
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);

    x_cm = transform.transform.translation.x * 100.0;
    y_cm = transform.transform.translation.y * 100.0;
    z_cm = current_height_cm_;

    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.transform.rotation, quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    yaw_deg = yaw * 180.0 / M_PI;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

std::optional<WaypointTarget> WaypointNavigator::currentTarget() const
{
  return current_target_;
}

double WaypointNavigator::normalizeAngleDeg(double angle_deg) const
{
  return angles::to_degrees(angles::normalize_angle(angles::from_degrees(angle_deg)));
}

bool WaypointNavigator::isReached(
  const WaypointTarget & target,
  const WaypointPolicy & policy,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dz = target.z_cm - z_cm;
  const double dxy = std::hypot(dx, dy);
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  return dxy <= positionTolerance(policy) &&
         std::fabs(dz) <= heightTolerance(policy) &&
         std::fabs(dyaw) <= yawTolerance(policy);
}

double WaypointNavigator::positionTolerance(const WaypointPolicy & policy) const
{
  return policy.position_tolerance_cm.value_or(default_position_tolerance_cm_);
}

double WaypointNavigator::heightTolerance(const WaypointPolicy & policy) const
{
  return policy.height_tolerance_cm.value_or(default_height_tolerance_cm_);
}

double WaypointNavigator::yawTolerance(const WaypointPolicy & policy) const
{
  return policy.yaw_tolerance_deg.value_or(default_yaw_tolerance_deg_);
}

void WaypointNavigator::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void WaypointNavigator::publishTarget(const WaypointTarget & target, const WaypointPolicy & policy)
{
  std_msgs::msg::Float32MultiArray message;
  message.data = {
    static_cast<float>(target.x_cm),
    static_cast<float>(target.y_cm),
    static_cast<float>(target.z_cm),
    static_cast<float>(target.yaw_deg),
  };
  target_pub_->publish(message);

  if (policy.enable_controller) {
    publishActiveController(2);
  }

  if (log_waypoint_targets_) {
    RCLCPP_INFO(
      node_.get_logger(),
      "Published waypoint target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg);
  } else {
    RCLCPP_DEBUG(
      node_.get_logger(),
      "Published waypoint target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg);
  }
}

}  // namespace activity_control_pkg
