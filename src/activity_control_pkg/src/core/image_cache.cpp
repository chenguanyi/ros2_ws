#include "activity_control_pkg/core/image_cache.hpp"

namespace activity_control_pkg
{

ImageCache::ImageCache(rclcpp::Node & node)
: node_(node)
{
  const auto image_topic = node_.declare_parameter("image_topic", std::string("/camera/image_raw"));
  image_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
    image_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ImageCache::imageCallback, this, std::placeholders::_1));
}

sensor_msgs::msg::Image::ConstSharedPtr ImageCache::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_image_;
}

bool ImageCache::hasImage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_image_ != nullptr;
}

void ImageCache::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_image_ = msg;
}

}  // namespace activity_control_pkg
