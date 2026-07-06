#include "activity_control_pkg/core/aux_controller.hpp"
#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace activity_control_pkg
{
namespace tasks
{
namespace
{

struct RedCircleDetection
{
  bool found{false};
  cv::Point2d center;
  double area{0.0};
  double circularity{0.0};
};

struct TimedAction
{
  bool active{false};
  rclcpp::Time until;
};

std::string formatSigned(double value)
{
  std::ostringstream stream;
  if (value >= 0.0) {
    stream << '+';
  }
  stream << std::fixed << std::setprecision(1) << value;
  return stream.str();
}

void drawTextWithBackground(
  cv::Mat & image,
  const std::string & text,
  cv::Point origin,
  double scale,
  const cv::Scalar & color)
{
  int baseline = 0;
  const int thickness = 2;
  const auto text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  const int max_x = std::max(4, image.cols - text_size.width - 8);
  const int max_y = std::max(text_size.height + 8, image.rows - baseline - 8);
  origin.x = std::clamp(origin.x, 4, max_x);
  origin.y = std::clamp(origin.y, text_size.height + 8, max_y);

  const cv::Rect bg(
    std::max(0, origin.x - 4),
    std::max(0, origin.y - text_size.height - 6),
    std::min(text_size.width + 8, image.cols - std::max(0, origin.x - 4)),
    std::min(text_size.height + baseline + 10, image.rows - std::max(0, origin.y - text_size.height - 6)));
  if (!bg.empty()) {
    cv::rectangle(image, bg, cv::Scalar(0, 0, 0), cv::FILLED);
  }
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
}

void drawCross(cv::Mat & image, const cv::Point & center, int radius, const cv::Scalar & color, int thickness)
{
  if (center.x < 0 || center.x >= image.cols || center.y < 0 || center.y >= image.rows) {
    return;
  }
  cv::line(
    image,
    cv::Point(std::max(0, center.x - radius), center.y),
    cv::Point(std::min(image.cols - 1, center.x + radius), center.y),
    color,
    thickness);
  cv::line(
    image,
    cv::Point(center.x, std::max(0, center.y - radius)),
    cv::Point(center.x, std::min(image.rows - 1, center.y + radius)),
    color,
    thickness);
}

}  // namespace

class DiansaiFirstTaskNode : public rclcpp::Node
{
public:
  DiansaiFirstTaskNode()
  : rclcpp::Node("diansai_first_task_node"),
    flight_(*this),
    aux_(*this)
  {
    loadParameters();

    const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    mission_complete_pub_ = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
    visual_takeover_pub_ = create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
    fine_data_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic_, rclcpp::QoS(10));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DiansaiFirstTaskNode::imageCallback, this, std::placeholders::_1));

    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);

    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&DiansaiFirstTaskNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "电赛第一题任务节点已启动：alignment_check_mode=%d image_topic=%s fine_data_topic=%s debug_view=%d。",
      alignment_check_mode_ ? 1 : 0,
      image_topic_.c_str(),
      fine_data_topic_.c_str(),
      show_debug_view_ ? 1 : 0);
  }

private:
  enum class Phase
  {
    WAIT_STATE,
    TAKEOFF,
    GOTO_PICKUP_FIXED,
    ALIGN_PICKUP,
    DESCEND_PICKUP,
    PICKUP_HOLD,
    CLIMB_CHECK,
    CHECK_MARKER,
    GOTO_ENDPOINT,
    DESCEND_DROPOFF,
    DROPOFF_ARM_DOWN,
    DROPOFF_MAGNET_OFF,
    DROPOFF_ARM_UP,
    COMPLETE_DESCEND,
    COMPLETE,
    STOPPED,
  };

  void loadParameters()
  {
    timer_period_sec_ = declare_parameter("timer_period_sec", 0.05);
    alignment_check_mode_ = declare_parameter("alignment_check_mode", false);
    image_topic_ = declare_parameter("image_topic", std::string("/camera/image_raw"));
    fine_data_topic_ = declare_parameter("fine_data_topic", std::string("/fine_data"));
    show_debug_view_ = declare_parameter("show_debug_view", true);
    debug_window_name_ = declare_parameter("debug_window_name", std::string("diansai_first_pickup_debug"));

    takeoff_ = loadTarget("takeoff", WaypointTarget{0.0, 0.0, 80.0, 0.0});
    pickup_ = loadTarget("pickup", WaypointTarget{60.0, -60.0, 80.0, 0.0});
    endpoint_ = loadTarget("endpoint", WaypointTarget{60.0, -120.0, 80.0, 0.0});
    action_height_cm_ = declare_parameter("action_height_cm", 20.0);
    complete_height_cm_ = declare_parameter("complete_height_cm", 10.0);

    pickup_hold_sec_ = declare_parameter("pickup_hold_sec", 5.0);
    dropoff_hold_sec_ = declare_parameter("dropoff_hold_sec", 1.0);
    max_pickup_attempts_ = declare_parameter("max_pickup_attempts", 3);
    align_timeout_sec_ = declare_parameter("align_timeout_sec", 8.0);
    align_deadzone_px_ = declare_parameter("align_deadzone_px", 30);
    align_target_offset_x_px_ = declare_parameter("align_target_offset_x_px", 0.0);
    align_target_offset_y_px_ = declare_parameter("align_target_offset_y_px", 0.0);
    align_stable_sec_ = declare_parameter("align_stable_sec", 0.5);
    post_pickup_check_sec_ = declare_parameter("post_pickup_check_sec", 1.0);
    post_pickup_visible_stable_sec_ = declare_parameter("post_pickup_visible_stable_sec", 0.3);
    detection_fresh_sec_ = declare_parameter("detection_fresh_sec", 0.4);

    h1_min_ = declare_parameter("red_h1_min", 0);
    h1_max_ = declare_parameter("red_h1_max", 12);
    h2_min_ = declare_parameter("red_h2_min", 139);
    h2_max_ = declare_parameter("red_h2_max", 179);
    s_min_ = declare_parameter("red_s_min", 28);
    s_max_ = declare_parameter("red_s_max", 255);
    v_min_ = declare_parameter("red_v_min", 70);
    v_max_ = declare_parameter("red_v_max", 255);
    blur_kernel_ = declare_parameter("red_blur", 5);
    erode_iterations_ = declare_parameter("red_erode", 1);
    dilate_iterations_ = declare_parameter("red_dilate", 2);
    min_area_ = declare_parameter("red_min_area", 300.0);
    min_circularity_ = declare_parameter("red_min_circularity", 0.55);

    if (timer_period_sec_ <= 0.0) {
      throw std::runtime_error("timer_period_sec must be > 0.");
    }
    if (max_pickup_attempts_ < 1) {
      throw std::runtime_error("max_pickup_attempts must be >= 1.");
    }
    if (align_timeout_sec_ <= 0.0 || align_stable_sec_ < 0.0) {
      throw std::runtime_error("align_timeout_sec must be > 0 and align_stable_sec must be >= 0.");
    }
  }

  WaypointTarget loadTarget(const std::string & prefix, const WaypointTarget & fallback)
  {
    return WaypointTarget{
      declare_parameter(prefix + "_x_cm", fallback.x_cm),
      declare_parameter(prefix + "_y_cm", fallback.y_cm),
      declare_parameter(prefix + "_z_cm", fallback.z_cm),
      declare_parameter(prefix + "_yaw_deg", fallback.yaw_deg),
    };
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        const auto gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        const auto rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else {
        bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      }
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "底部相机图像转换失败：%s", ex.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    const auto detection = detectRedCircle(bgr);
    const rclcpp::Time stamp = now();
    last_detection_found_ = detection.found;
    last_detection_stamp_ = stamp;
    last_detection_area_ = detection.area;
    last_detection_circularity_ = detection.circularity;

    if (detection.found) {
      const double target_x = static_cast<double>(bgr.cols) * 0.5 + align_target_offset_x_px_;
      const double target_y = static_cast<double>(bgr.rows) * 0.5 + align_target_offset_y_px_;
      last_error_x_px_ = detection.center.x - target_x;
      last_error_y_px_ = target_y - detection.center.y;
      last_center_x_px_ = detection.center.x;
      last_center_y_px_ = detection.center.y;
      publishFineData(last_error_x_px_, last_error_y_px_);
    }

    if (show_debug_view_) {
      drawDebugView(bgr, detection);
      try {
        cv::imshow(debug_window_name_, bgr);
        cv::waitKey(1);
      } catch (const cv::Exception & ex) {
        show_debug_view_ = false;
        RCLCPP_WARN(
          get_logger(),
          "OpenCV 调试窗口不可用，已自动关闭 show_debug_view：%s",
          ex.what());
      }
    }
  }

  RedCircleDetection detectRedCircle(const cv::Mat & bgr) const
  {
    cv::Mat blurred;
    int kernel = std::max(1, blur_kernel_);
    if (kernel % 2 == 0) {
      ++kernel;
    }
    if (kernel > 1) {
      cv::GaussianBlur(bgr, blurred, cv::Size(kernel, kernel), 0.0);
    } else {
      blurred = bgr;
    }

    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(
      hsv,
      cv::Scalar(h1_min_, s_min_, v_min_),
      cv::Scalar(h1_max_, s_max_, v_max_),
      mask1);
    cv::inRange(
      hsv,
      cv::Scalar(h2_min_, s_min_, v_min_),
      cv::Scalar(h2_max_, s_max_, v_max_),
      mask2);
    cv::Mat mask = mask1 | mask2;

    if (erode_iterations_ > 0) {
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), erode_iterations_);
    }
    if (dilate_iterations_ > 0) {
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), dilate_iterations_);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    RedCircleDetection best;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < min_area_) {
        continue;
      }
      const double perimeter = cv::arcLength(contour, true);
      if (perimeter <= 1e-6) {
        continue;
      }
      const double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
      if (circularity < min_circularity_) {
        continue;
      }
      const auto moments = cv::moments(contour);
      if (std::fabs(moments.m00) <= 1e-6) {
        continue;
      }
      if (!best.found || area > best.area) {
        best.found = true;
        best.center = cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00);
        best.area = area;
        best.circularity = circularity;
      }
    }
    return best;
  }

  void publishFineData(double error_x_px, double error_y_px)
  {
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {
      static_cast<int>(std::lround(error_x_px)),
      static_cast<int>(std::lround(error_y_px)),
    };
    fine_data_pub_->publish(msg);
  }

  void drawDebugView(cv::Mat & image, const RedCircleDetection & detection) const
  {
    const cv::Point target_point(
      static_cast<int>(std::lround(static_cast<double>(image.cols) * 0.5 + align_target_offset_x_px_)),
      static_cast<int>(std::lround(static_cast<double>(image.rows) * 0.5 + align_target_offset_y_px_)));
    const int deadzone = std::max(0, align_deadzone_px_);
    cv::Rect target_range(
      target_point.x - deadzone,
      target_point.y - deadzone,
      deadzone * 2 + 1,
      deadzone * 2 + 1);
    target_range &= cv::Rect(0, 0, image.cols, image.rows);
    if (!target_range.empty()) {
      cv::rectangle(image, target_range, cv::Scalar(255, 255, 0), 2);
    }
    drawCross(image, target_point, 14, cv::Scalar(255, 255, 0), 2);

    std::string status = "LOST";
    cv::Scalar status_color(0, 80, 255);
    if (detection.found) {
      const cv::Point marker_point(
        static_cast<int>(std::lround(detection.center.x)),
        static_cast<int>(std::lround(detection.center.y)));
      const double error_x = detection.center.x - static_cast<double>(target_point.x);
      const double error_y = static_cast<double>(target_point.y) - detection.center.y;
      const bool in_range = std::abs(error_x) <= deadzone && std::abs(error_y) <= deadzone;
      status = in_range ? "LOCK RANGE" : "ALIGN";
      status_color = in_range ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 220, 255);
      cv::circle(image, marker_point, 12, status_color, 2);
      drawCross(image, marker_point, 10, status_color, 2);
      cv::line(image, target_point, marker_point, cv::Scalar(255, 255, 255), 1);
      drawTextWithBackground(
        image,
        "error: " + formatSigned(error_x) + ", " + formatSigned(error_y) + " px",
        cv::Point(12, image.rows - 62),
        0.6,
        status_color);
      drawTextWithBackground(
        image,
        "area: " + std::to_string(static_cast<int>(std::lround(detection.area))) +
        " circ: " + std::to_string(static_cast<int>(std::lround(detection.circularity * 100.0))) + "%",
        cv::Point(12, image.rows - 34),
        0.55,
        cv::Scalar(255, 255, 255));
    }

    drawTextWithBackground(image, status, cv::Point(12, 30), 0.7, status_color);
    drawTextWithBackground(image, phaseText(), cv::Point(12, 62), 0.6, cv::Scalar(255, 255, 255));
  }

  std::string phaseText() const
  {
    switch (phase_) {
      case Phase::WAIT_STATE:
        return "WAIT_STATE";
      case Phase::TAKEOFF:
        return "TAKEOFF";
      case Phase::GOTO_PICKUP_FIXED:
        return "GOTO_PICKUP_FIXED";
      case Phase::ALIGN_PICKUP:
        return "ALIGN_PICKUP";
      case Phase::DESCEND_PICKUP:
        return "DESCEND_PICKUP";
      case Phase::PICKUP_HOLD:
        return "PICKUP_HOLD";
      case Phase::CLIMB_CHECK:
        return "CLIMB_CHECK";
      case Phase::CHECK_MARKER:
        return "CHECK_MARKER";
      case Phase::GOTO_ENDPOINT:
        return "GOTO_ENDPOINT";
      case Phase::DESCEND_DROPOFF:
        return "DESCEND_DROPOFF";
      case Phase::DROPOFF_ARM_DOWN:
        return "DROPOFF_ARM_DOWN";
      case Phase::DROPOFF_MAGNET_OFF:
        return "DROPOFF_MAGNET_OFF";
      case Phase::DROPOFF_ARM_UP:
        return "DROPOFF_ARM_UP";
      case Phase::COMPLETE_DESCEND:
        return "COMPLETE_DESCEND";
      case Phase::COMPLETE:
        return "COMPLETE";
      case Phase::STOPPED:
        return "STOPPED";
    }
    return "UNKNOWN";
  }

  bool hasFreshDetection() const
  {
    return last_detection_found_ && last_detection_stamp_.nanoseconds() != 0 &&
           (now() - last_detection_stamp_).seconds() <= detection_fresh_sec_;
  }

  bool markerCentered() const
  {
    if (!hasFreshDetection()) {
      return false;
    }
    return std::abs(last_error_x_px_) <= static_cast<double>(align_deadzone_px_) &&
           std::abs(last_error_y_px_) <= static_cast<double>(align_deadzone_px_);
  }

  void setVisualTakeover(bool enabled)
  {
    if (visual_takeover_enabled_ == enabled) {
      return;
    }
    visual_takeover_enabled_ = enabled;
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    visual_takeover_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "视觉接管%s。", enabled ? "开启" : "关闭");
  }

  void goToPhase(Phase phase, const std::string & log_message)
  {
    phase_ = phase;
    timed_action_.active = false;
    if (!log_message.empty()) {
      RCLCPP_INFO(get_logger(), "%s", log_message.c_str());
    }
  }

  bool waitTimed(double seconds)
  {
    if (seconds <= 0.0) {
      timed_action_.active = false;
      return true;
    }
    const auto stamp = now();
    if (!timed_action_.active) {
      timed_action_.active = true;
      timed_action_.until = stamp + rclcpp::Duration::from_seconds(seconds);
      return false;
    }
    if (stamp < timed_action_.until) {
      return false;
    }
    timed_action_.active = false;
    return true;
  }

  WaypointTarget withHeight(const WaypointTarget & target, double height_cm) const
  {
    return WaypointTarget{target.x_cm, target.y_cm, height_cm, target.yaw_deg};
  }

  void startTask()
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    pickup_attempt_ = 0;
    descent_target_ = withHeight(pickup_, action_height_cm_);
    flight_.goTo(takeoff_);
    goToPhase(Phase::TAKEOFF, "任务开始：已发布安全执行器状态，前往起飞点。");
  }

  void startPickupAttempt()
  {
    ++pickup_attempt_;
    stable_align_start_ = rclcpp::Time();
    align_start_ = now();
    descent_target_ = withHeight(pickup_, action_height_cm_);
    setVisualTakeover(true);
    goToPhase(
      Phase::ALIGN_PICKUP,
      "开始第 " + std::to_string(pickup_attempt_) + " 次取物视觉对准。");
  }

  void lockDescentTargetFromCurrentPoseOrFallback(bool aligned)
  {
    double x_cm = 0.0;
    double y_cm = 0.0;
    double z_cm = 0.0;
    double yaw_deg = 0.0;
    if (aligned && flight_.getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
      descent_target_ = WaypointTarget{x_cm, y_cm, action_height_cm_, pickup_.yaw_deg};
      RCLCPP_INFO(
        get_logger(),
        "取物对准成功，锁定下降坐标 x=%.1fcm y=%.1fcm。",
        descent_target_.x_cm,
        descent_target_.y_cm);
    } else if (aligned) {
      descent_target_ = withHeight(pickup_, action_height_cm_);
      RCLCPP_WARN(get_logger(), "取物已对准但读取当前 TF 失败，使用固定取物点下降。");
    } else {
      descent_target_ = withHeight(pickup_, action_height_cm_);
      RCLCPP_WARN(get_logger(), "取物对准超时，使用固定取物点下降。");
    }
  }

  void completeTask()
  {
    if (mission_complete_sent_) {
      return;
    }
    setVisualTakeover(false);
    flight_.publishActiveController(3);
    aux_.setAll(0, 0, 0);
    std_msgs::msg::Empty message;
    mission_complete_pub_->publish(message);
    mission_complete_sent_ = true;
    goToPhase(Phase::STOPPED, "电赛第一题任务完成，已发布 /mission_complete。");
  }

  void timerCallback()
  {
    if (alignment_check_mode_) {
      return;
    }
    if (mission_complete_sent_) {
      return;
    }

    switch (phase_) {
      case Phase::WAIT_STATE:
        if (!flight_.hasState()) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "等待定位和高度数据...");
          return;
        }
        startTask();
        return;

      case Phase::TAKEOFF:
        if (!flight_.isReached()) {
          return;
        }
        flight_.goTo(pickup_);
        goToPhase(Phase::GOTO_PICKUP_FIXED, "已到达起飞点，前往固定取物点。");
        return;

      case Phase::GOTO_PICKUP_FIXED:
        if (!flight_.isReached()) {
          return;
        }
        startPickupAttempt();
        return;

      case Phase::ALIGN_PICKUP:
        processAlignment();
        return;

      case Phase::DESCEND_PICKUP:
        if (!flight_.isReached()) {
          return;
        }
        goToPhase(Phase::PICKUP_HOLD, "已下降到取物高度，保持机械臂和磁铁吸附。");
        return;

      case Phase::PICKUP_HOLD:
        if (!waitTimed(pickup_hold_sec_)) {
          return;
        }
        aux_.setAll(0, 1, 0);
        flight_.goTo(WaypointTarget{descent_target_.x_cm, descent_target_.y_cm, pickup_.z_cm, pickup_.yaw_deg});
        goToPhase(Phase::CLIMB_CHECK, "取物保持完成，收回机械臂并爬升到检查高度。");
        return;

      case Phase::CLIMB_CHECK:
        if (!flight_.isReached()) {
          return;
        }
        check_start_ = now();
        visible_stable_start_ = rclcpp::Time();
        goToPhase(Phase::CHECK_MARKER, "开始检查红色圆片是否仍可见。");
        return;

      case Phase::CHECK_MARKER:
        processPickupCheck();
        return;

      case Phase::GOTO_ENDPOINT:
        if (!flight_.isReached()) {
          return;
        }
        flight_.goTo(withHeight(endpoint_, action_height_cm_));
        goToPhase(Phase::DESCEND_DROPOFF, "已到达终点上方，下降到放置高度。");
        return;

      case Phase::DESCEND_DROPOFF:
        if (!flight_.isReached()) {
          return;
        }
        aux_.setAll(1, 1, 0);
        goToPhase(Phase::DROPOFF_ARM_DOWN, "已到达放置高度，放下机械臂并保持磁铁开启。");
        return;

      case Phase::DROPOFF_ARM_DOWN:
        if (!waitTimed(dropoff_hold_sec_)) {
          return;
        }
        aux_.setAll(1, 0, 0);
        goToPhase(Phase::DROPOFF_MAGNET_OFF, "放置位置保持完成，关闭磁铁释放铁块。");
        return;

      case Phase::DROPOFF_MAGNET_OFF:
        if (!waitTimed(dropoff_hold_sec_)) {
          return;
        }
        aux_.setAll(0, 0, 0);
        goToPhase(Phase::DROPOFF_ARM_UP, "铁块释放完成，收回机械臂。");
        return;

      case Phase::DROPOFF_ARM_UP:
        if (!waitTimed(dropoff_hold_sec_)) {
          return;
        }
        flight_.goTo(withHeight(endpoint_, complete_height_cm_));
        goToPhase(Phase::COMPLETE_DESCEND, "机械臂已收回，下降到任务完成高度。");
        return;

      case Phase::COMPLETE_DESCEND:
        if (!flight_.isReached()) {
          return;
        }
        goToPhase(Phase::COMPLETE, "已到达完成高度，准备发布任务完成信号。");
        return;

      case Phase::COMPLETE:
        completeTask();
        return;

      case Phase::STOPPED:
        return;
    }
  }

  void processAlignment()
  {
    const auto stamp = now();
    const bool centered = markerCentered();
    if (centered) {
      if (stable_align_start_.nanoseconds() == 0) {
        stable_align_start_ = stamp;
      }
      if ((stamp - stable_align_start_).seconds() >= align_stable_sec_) {
        setVisualTakeover(false);
        lockDescentTargetFromCurrentPoseOrFallback(true);
        RCLCPP_INFO(get_logger(), "视觉对准稳定，发送 arm=1 magnet=1 signal=0 后下降取物。");
        aux_.setAll(1, 1, 0);
        flight_.goTo(descent_target_);
        goToPhase(Phase::DESCEND_PICKUP, "视觉对准稳定，放下机械臂并开启磁铁，下降取物。");
        return;
      }
    } else {
      stable_align_start_ = rclcpp::Time();
    }

    if (align_start_.nanoseconds() != 0 && (stamp - align_start_).seconds() >= align_timeout_sec_) {
      setVisualTakeover(false);
      lockDescentTargetFromCurrentPoseOrFallback(false);
      RCLCPP_INFO(get_logger(), "视觉对准超时，发送 arm=1 magnet=1 signal=0 后下降取物。");
      aux_.setAll(1, 1, 0);
      flight_.goTo(descent_target_);
      goToPhase(Phase::DESCEND_PICKUP, "视觉对准超时，放下机械臂并开启磁铁，继续下降取物。");
      return;
    }

    if (hasFreshDetection()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "取物对准中：误差 x=%.1fpx y=%.1fpx。",
        last_error_x_px_,
        last_error_y_px_);
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "取物对准中：暂未识别到红色圆片。");
    }
  }

  void processPickupCheck()
  {
    const auto stamp = now();
    if (hasFreshDetection()) {
      if (visible_stable_start_.nanoseconds() == 0) {
        visible_stable_start_ = stamp;
      }
    } else {
      visible_stable_start_ = rclcpp::Time();
    }

    const bool visible_stable = visible_stable_start_.nanoseconds() != 0 &&
      (stamp - visible_stable_start_).seconds() >= post_pickup_visible_stable_sec_;
    const bool check_done = check_start_.nanoseconds() != 0 &&
      (stamp - check_start_).seconds() >= post_pickup_check_sec_;
    if (!visible_stable && !check_done) {
      return;
    }

    if (visible_stable) {
      RCLCPP_WARN(get_logger(), "红色圆片仍可见，判断本次取物可能失败。");
      if (pickup_attempt_ < max_pickup_attempts_) {
        flight_.goTo(pickup_);
        goToPhase(Phase::GOTO_PICKUP_FIXED, "准备返回固定取物点重新对准取物。");
        return;
      }
      RCLCPP_WARN(get_logger(), "已达到最大取物次数，继续前往终点。");
    } else {
      RCLCPP_INFO(get_logger(), "检查期间红色圆片未稳定可见，判断取物成功。");
    }

    aux_.setAll(0, 1, 0);
    flight_.goTo(endpoint_);
    goToPhase(Phase::GOTO_ENDPOINT, "保持磁铁开启，前往固定终点。");
  }

  WaypointNavigator flight_;
  AuxController aux_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  double timer_period_sec_{0.05};
  bool alignment_check_mode_{false};
  bool show_debug_view_{true};
  std::string image_topic_{"/camera/image_raw"};
  std::string fine_data_topic_{"/fine_data"};
  std::string debug_window_name_{"diansai_first_pickup_debug"};

  WaypointTarget takeoff_;
  WaypointTarget pickup_;
  WaypointTarget endpoint_;
  double action_height_cm_{20.0};
  double complete_height_cm_{10.0};
  double pickup_hold_sec_{5.0};
  double dropoff_hold_sec_{1.0};
  int max_pickup_attempts_{3};
  double align_timeout_sec_{8.0};
  int align_deadzone_px_{30};
  double align_target_offset_x_px_{0.0};
  double align_target_offset_y_px_{0.0};
  double align_stable_sec_{0.5};
  double post_pickup_check_sec_{1.0};
  double post_pickup_visible_stable_sec_{0.3};
  double detection_fresh_sec_{0.4};

  int h1_min_{0};
  int h1_max_{12};
  int h2_min_{139};
  int h2_max_{179};
  int s_min_{28};
  int s_max_{255};
  int v_min_{70};
  int v_max_{255};
  int blur_kernel_{5};
  int erode_iterations_{1};
  int dilate_iterations_{2};
  double min_area_{300.0};
  double min_circularity_{0.55};

  Phase phase_{Phase::WAIT_STATE};
  bool mission_complete_sent_{false};
  bool visual_takeover_enabled_{false};
  int pickup_attempt_{0};
  WaypointTarget descent_target_;
  TimedAction timed_action_;

  bool last_detection_found_{false};
  rclcpp::Time last_detection_stamp_;
  double last_error_x_px_{0.0};
  double last_error_y_px_{0.0};
  double last_center_x_px_{0.0};
  double last_center_y_px_{0.0};
  double last_detection_area_{0.0};
  double last_detection_circularity_{0.0};
  rclcpp::Time align_start_;
  rclcpp::Time stable_align_start_;
  rclcpp::Time check_start_;
  rclcpp::Time visible_stable_start_;
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::DiansaiFirstTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
