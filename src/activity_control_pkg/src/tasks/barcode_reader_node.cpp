#include <algorithm>
#include <cctype>
#include <functional>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <zbar.h>

namespace activity_control_pkg
{
namespace tasks
{

class BarcodeReaderNode : public rclcpp::Node
{
public:
  BarcodeReaderNode()
  : rclcpp::Node("barcode_reader_node")
  {
    const auto image_topic =
      declare_parameter("image_topic", std::string("/side_camera/image_raw"));
    const auto barcode_topic =
      declare_parameter("barcode_topic", std::string("/plant_protection/barcode_value"));
    stable_count_required_ = declare_parameter("stable_count", 3);

    if (stable_count_required_ < 1) {
      throw std::runtime_error("stable_count must be >= 1.");
    }

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ENABLE, 1);

    const auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    barcode_pub_ = create_publisher<std_msgs::msg::Int32>(barcode_topic, pub_qos);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&BarcodeReaderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Barcode reader ready: image_topic=%s barcode_topic=%s stable_count=%d.",
      image_topic.c_str(),
      barcode_topic.c_str(),
      stable_count_required_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat gray;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
      } else {
        const auto bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      }
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Barcode image conversion failed: %s", ex.what());
      return;
    }

    if (gray.empty() || gray.type() != CV_8UC1) {
      return;
    }
    if (!gray.isContinuous()) {
      gray = gray.clone();
    }

    zbar::Image image(
      static_cast<unsigned>(gray.cols),
      static_cast<unsigned>(gray.rows),
      "Y800",
      gray.data,
      gray.total());
    scanner_.scan(image);

    for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      const auto value = parseBarcodeValue(symbol->get_data());
      if (value > 0) {
        observeValue(value);
        image.set_data(nullptr, 0);
        return;
      }
    }

    image.set_data(nullptr, 0);
  }

  static int parseBarcodeValue(std::string data)
  {
    data.erase(
      std::remove_if(data.begin(), data.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
      }),
      data.end());
    if (data.empty()) {
      return 0;
    }
    const bool all_digits = std::all_of(data.begin(), data.end(), [](unsigned char ch) {
      return std::isdigit(ch) != 0;
    });
    if (!all_digits) {
      return 0;
    }
    try {
      return std::stoi(data);
    } catch (const std::exception &) {
      return 0;
    }
  }

  void observeValue(int value)
  {
    if (value == pending_value_) {
      ++pending_count_;
    } else {
      pending_value_ = value;
      pending_count_ = 1;
    }

    if (pending_count_ < stable_count_required_ || value == published_value_) {
      return;
    }

    published_value_ = value;
    std_msgs::msg::Int32 msg;
    msg.data = value;
    barcode_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Stable barcode detected: %d.", value);
  }

  zbar::ImageScanner scanner_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr barcode_pub_;
  int stable_count_required_{3};
  int pending_value_{0};
  int pending_count_{0};
  int published_value_{0};
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::BarcodeReaderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
