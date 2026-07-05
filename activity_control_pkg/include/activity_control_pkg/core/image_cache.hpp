#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace activity_control_pkg
{

class ImageCache
{
public:
  explicit ImageCache(rclcpp::Node & node);
  sensor_msgs::msg::Image::ConstSharedPtr latest() const;
  bool hasImage() const;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  mutable std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
};

}  // namespace activity_control_pkg
