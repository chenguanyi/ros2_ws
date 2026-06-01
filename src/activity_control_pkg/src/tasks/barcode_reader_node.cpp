#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
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
    const auto candidate_topic =
      declare_parameter("candidate_topic", std::string("/plant_protection/barcode_candidate"));
    const auto overlay_topic =
      declare_parameter("overlay_topic", std::string("/side_camera/barcode_overlay"));
    const auto fine_data_topic =
      declare_parameter("fine_data_topic", std::string("/fine_data"));
    stable_count_required_ = declare_parameter("stable_count", 3);
    republish_period_sec_ = declare_parameter("republish_period_sec", 1.0);
    publish_fine_data_ = declare_parameter("publish_fine_data", false);

    if (stable_count_required_ < 1) {
      throw std::runtime_error("stable_count must be >= 1.");
    }
    if (republish_period_sec_ < 0.0) {
      throw std::runtime_error("republish_period_sec must be >= 0.");
    }

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ENABLE, 1);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    const auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    barcode_pub_ = create_publisher<std_msgs::msg::Int32>(barcode_topic, pub_qos);
    candidate_pub_ = create_publisher<std_msgs::msg::Int32>(candidate_topic, pub_qos);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(overlay_topic, rclcpp::SensorDataQoS());
    fine_data_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic, rclcpp::QoS(10));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&BarcodeReaderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Barcode reader ready: image_topic=%s barcode_topic=%s candidate_topic=%s overlay_topic=%s fine_data_topic=%s publish_fine_data=%d stable_count=%d republish_period=%.2fs.",
      image_topic.c_str(),
      barcode_topic.c_str(),
      candidate_topic.c_str(),
      overlay_topic.c_str(),
      fine_data_topic.c_str(),
      publish_fine_data_ ? 1 : 0,
      stable_count_required_,
      republish_period_sec_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    cv::Mat gray;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      } else {
        bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      }
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Barcode image conversion failed: %s", ex.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    if (gray.empty() || gray.type() != CV_8UC1) {
      publishOverlay(*msg, bgr);
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
        publishCandidate(value);
        observeValue(value);
        publishFineDataIfEnabled(bgr, *symbol);
        drawBarcodeOverlay(bgr, *symbol, value);
        publishOverlay(*msg, bgr);
        image.set_data(nullptr, 0);
        return;
      }
    }

    publishOverlay(*msg, bgr);
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

    if (pending_count_ < stable_count_required_) {
      return;
    }

    const auto stamp = now();
    const bool changed = value != published_value_;
    const bool first_publish = last_publish_time_.nanoseconds() == 0;
    bool republish_due = false;
    if (!first_publish) {
      republish_due =
        republish_period_sec_ <= 0.0 ||
        (stamp - last_publish_time_).seconds() >= republish_period_sec_;
    }
    if (!changed && !first_publish && !republish_due) {
      return;
    }

    published_value_ = value;
    last_publish_time_ = stamp;
    std_msgs::msg::Int32 msg;
    msg.data = value;
    barcode_pub_->publish(msg);
    if (changed || first_publish) {
      RCLCPP_INFO(get_logger(), "Stable barcode detected: %d.", value);
    }
  }

  void publishCandidate(int value)
  {
    std_msgs::msg::Int32 msg;
    msg.data = value;
    candidate_pub_->publish(msg);
  }

  void drawBarcodeOverlay(cv::Mat & image, const zbar::Symbol & symbol, int value) const
  {
    std::vector<cv::Point> points;
    const int point_count = symbol.get_location_size();
    points.reserve(std::max(0, point_count));
    for (int i = 0; i < point_count; ++i) {
      points.emplace_back(symbol.get_location_x(i), symbol.get_location_y(i));
    }

    cv::Rect box;
    if (points.size() >= 2U) {
      box = cv::boundingRect(points);
    } else {
      box = cv::Rect(0, 0, image.cols, image.rows);
    }

    box &= cv::Rect(0, 0, image.cols, image.rows);
    if (box.empty()) {
      return;
    }

    cv::rectangle(image, box, cv::Scalar(0, 255, 0), 3);
    if (points.size() >= 4U) {
      cv::polylines(image, points, true, cv::Scalar(0, 180, 255), 2);
    }

    const auto text = std::to_string(value);
    int baseline = 0;
    const auto text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline);
    const int text_x = std::min(std::max(box.x + box.width + 8, 0), image.cols - text_size.width - 8);
    const int text_y = std::max(box.y + text_size.height + 4, text_size.height + 8);
    const cv::Rect text_bg(
      std::max(0, text_x - 4),
      std::max(0, text_y - text_size.height - 6),
      std::min(text_size.width + 8, image.cols - std::max(0, text_x - 4)),
      text_size.height + baseline + 10);
    cv::rectangle(image, text_bg, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
      image,
      text,
      cv::Point(text_x, text_y),
      cv::FONT_HERSHEY_SIMPLEX,
      1.0,
      cv::Scalar(0, 255, 0),
      2);
  }

  void publishFineDataIfEnabled(const cv::Mat & image, const zbar::Symbol & symbol)
  {
    if (!publish_fine_data_ || image.empty()) {
      return;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    const int point_count = symbol.get_location_size();
    if (point_count > 0) {
      for (int i = 0; i < point_count; ++i) {
        sum_x += static_cast<double>(symbol.get_location_x(i));
        sum_y += static_cast<double>(symbol.get_location_y(i));
      }
      sum_x /= static_cast<double>(point_count);
      sum_y /= static_cast<double>(point_count);
    } else {
      sum_x = static_cast<double>(image.cols) * 0.5;
      sum_y = static_cast<double>(image.rows) * 0.5;
    }

    std_msgs::msg::Int32MultiArray msg;
    msg.data = {
      static_cast<int>(std::lround(sum_x - static_cast<double>(image.cols) * 0.5)),
      static_cast<int>(std::lround(sum_y - static_cast<double>(image.rows) * 0.5)),
    };
    fine_data_pub_->publish(msg);
  }

  void publishOverlay(const sensor_msgs::msg::Image & source_msg, const cv::Mat & image)
  {
    auto overlay_msg = cv_bridge::CvImage(
      source_msg.header,
      sensor_msgs::image_encodings::BGR8,
      image).toImageMsg();
    overlay_pub_->publish(*overlay_msg);
  }

  zbar::ImageScanner scanner_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr barcode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr candidate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  int stable_count_required_{3};
  double republish_period_sec_{1.0};
  bool publish_fine_data_{false};
  int pending_value_{0};
  int pending_count_{0};
  int published_value_{0};
  rclcpp::Time last_publish_time_;
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
