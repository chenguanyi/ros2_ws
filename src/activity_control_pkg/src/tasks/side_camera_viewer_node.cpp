#include <chrono>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

namespace activity_control_pkg
{
namespace tasks
{

class SideCameraViewerNode : public rclcpp::Node
{
public:
  SideCameraViewerNode()
  : rclcpp::Node("side_camera_viewer_node")
  {
    const auto image_topic =
      declare_parameter("image_topic", std::string("/side_camera/barcode_overlay"));
    const auto candidate_topic =
      declare_parameter("candidate_topic", std::string("/plant_protection/barcode_candidate"));
    const auto barcode_topic =
      declare_parameter("barcode_topic", std::string("/plant_protection/barcode_value"));
    window_name_ = declare_parameter("window_name", std::string("Side camera barcode"));
    display_width_ = declare_parameter("display_width", 960);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        imageCallback(msg);
      });
    candidate_sub_ = create_subscription<std_msgs::msg::Int32>(
      candidate_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        candidate_value_ = msg->data;
        candidate_stamp_ = now();
      });
    barcode_sub_ = create_subscription<std_msgs::msg::Int32>(
      barcode_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        stable_value_ = msg->data;
        stable_stamp_ = now();
      });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    RCLCPP_INFO(
      get_logger(),
      "Side camera viewer ready: image_topic=%s candidate_topic=%s barcode_topic=%s.",
      image_topic.c_str(),
      candidate_topic.c_str(),
      barcode_topic.c_str());
  }

  ~SideCameraViewerNode() override
  {
    cv::destroyWindow(window_name_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        const auto rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        const auto gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);
      } else {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      }
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Viewer image conversion failed: %s", ex.what());
      return;
    }

    if (frame.empty()) {
      return;
    }

    drawStatus(frame);
    resizeForDisplay(frame);
    cv::imshow(window_name_, frame);
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      RCLCPP_INFO(get_logger(), "Viewer close requested from keyboard.");
      rclcpp::shutdown();
    }
  }

  void drawStatus(cv::Mat & frame) const
  {
    const auto stamp = now();
    const bool candidate_recent =
      candidate_stamp_.nanoseconds() > 0 && (stamp - candidate_stamp_).seconds() < 1.0;
    const bool stable_seen = stable_stamp_.nanoseconds() > 0;

    const std::string candidate_text =
      candidate_recent ? "candidate: " + std::to_string(candidate_value_) : "candidate: none";
    const std::string stable_text =
      stable_seen ? "stable: " + std::to_string(stable_value_) : "stable: none";
    const std::string status_text = candidate_recent ? "recognizing" : "searching";

    const int margin = 12;
    const int line_height = 28;
    const cv::Rect bg(0, 0, std::min(frame.cols, 360), 104);
    cv::rectangle(frame, bg, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
      frame,
      status_text,
      cv::Point(margin, margin + line_height - 6),
      cv::FONT_HERSHEY_SIMPLEX,
      0.75,
      candidate_recent ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 220, 255),
      2);
    cv::putText(
      frame,
      candidate_text,
      cv::Point(margin, margin + 2 * line_height - 4),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(255, 255, 255),
      2);
    cv::putText(
      frame,
      stable_text,
      cv::Point(margin, margin + 3 * line_height - 2),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(255, 255, 255),
      2);
  }

  void resizeForDisplay(cv::Mat & frame) const
  {
    if (display_width_ <= 0 || frame.cols <= display_width_) {
      return;
    }
    const double scale = static_cast<double>(display_width_) / static_cast<double>(frame.cols);
    cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_AREA);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr candidate_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr barcode_sub_;
  std::string window_name_;
  int display_width_{960};
  int candidate_value_{0};
  int stable_value_{0};
  rclcpp::Time candidate_stamp_;
  rclcpp::Time stable_stamp_;
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::SideCameraViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
