#include "activity_control_pkg/core/sensor_cache.hpp"

namespace activity_control_pkg
{

SensorCache::SensorCache(rclcpp::Node & node)
: node_(node)
{
  const auto height_topic = node_.declare_parameter("sensor_height_topic", std::string("/height"));
  const auto height_raw_topic = node_.declare_parameter("sensor_height_raw_topic", std::string("/height_raw"));
  const auto laser_ground_topic =
    node_.declare_parameter("sensor_laser_ground_topic", std::string("/laser_array/ground_height"));
  const auto laser_obstacle_topic =
    node_.declare_parameter("sensor_laser_obstacle_topic", std::string("/laser_array/obstacle_height"));

  height_sub_ = node_.create_subscription<std_msgs::msg::Int16>(
    height_topic,
    rclcpp::QoS(10),
    std::bind(&SensorCache::heightCallback, this, std::placeholders::_1));
  height_raw_sub_ = node_.create_subscription<std_msgs::msg::Int16>(
    height_raw_topic,
    rclcpp::QoS(10),
    std::bind(&SensorCache::heightRawCallback, this, std::placeholders::_1));
  laser_ground_sub_ = node_.create_subscription<std_msgs::msg::Float32>(
    laser_ground_topic,
    rclcpp::QoS(10),
    std::bind(&SensorCache::laserGroundCallback, this, std::placeholders::_1));
  laser_obstacle_sub_ = node_.create_subscription<std_msgs::msg::Float32>(
    laser_obstacle_topic,
    rclcpp::QoS(10),
    std::bind(&SensorCache::laserObstacleCallback, this, std::placeholders::_1));
}

SensorSnapshot SensorCache::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void SensorCache::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.height_cm = static_cast<double>(msg->data);
  snapshot_.has_height = true;
}

void SensorCache::heightRawCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.height_raw_cm = static_cast<double>(msg->data);
  snapshot_.has_height_raw = true;
}

void SensorCache::laserGroundCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.laser_ground_height_m = static_cast<double>(msg->data);
  snapshot_.has_laser_ground_height = true;
}

void SensorCache::laserObstacleCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.laser_obstacle_height_m = static_cast<double>(msg->data);
  snapshot_.has_laser_obstacle_height = true;
}

}  // namespace activity_control_pkg
