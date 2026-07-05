#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>

namespace activity_control_pkg
{

struct SensorSnapshot
{
  bool has_height{false};
  double height_cm{0.0};
  bool has_height_raw{false};
  double height_raw_cm{0.0};
  bool has_laser_ground_height{false};
  double laser_ground_height_m{0.0};
  bool has_laser_obstacle_height{false};
  double laser_obstacle_height_m{0.0};
};

class SensorCache
{
public:
  explicit SensorCache(rclcpp::Node & node);
  SensorSnapshot snapshot() const;

private:
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void heightRawCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void laserGroundCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void laserObstacleCallback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_raw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr laser_ground_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr laser_obstacle_sub_;
  mutable std::mutex mutex_;
  SensorSnapshot snapshot_;
};

}  // namespace activity_control_pkg
