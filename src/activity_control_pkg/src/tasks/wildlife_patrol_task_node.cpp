#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <chrono>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace activity_control_pkg
{
namespace tasks
{

class WildlifePatrolTaskNode : public rclcpp::Node
{
public:
  WildlifePatrolTaskNode()
  : rclcpp::Node("wildlife_patrol_task_node"), flight_(*this)
  {
    timer_period_sec_ = declare_parameter("timer_period_sec", 0.05);
    waypoints_topic_ = declare_parameter("waypoints_topic", std::string("/wildlife/waypoints"));
    detections_topic_ = declare_parameter("detections_topic", std::string("/yolo/detections"));
    animal_topic_ = declare_parameter("animal_topic", std::string("/animal"));
    status_topic_ = declare_parameter("status_topic", std::string("/wildlife/status"));
    animal_confidence_threshold_ = declare_parameter("animal_confidence_threshold", 0.70);
    animal_confirm_frames_ = declare_parameter("animal_confirm_frames", 3);
    takeoff_x_cm_ = declare_parameter("takeoff_x_cm", 0.0);
    takeoff_y_cm_ = declare_parameter("takeoff_y_cm", 0.0);
    takeoff_z_cm_ = declare_parameter("takeoff_z_cm", 120.0);
    takeoff_yaw_deg_ = declare_parameter("takeoff_yaw_deg", 0.0);
    route_height_cm_ = declare_parameter("route_height_cm", 120.0);
    route_yaw_deg_ = declare_parameter("route_yaw_deg", 0.0);

    waypoints_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      waypoints_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WildlifePatrolTaskNode::waypointsCallback, this, std::placeholders::_1));
    detections_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      detections_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WildlifePatrolTaskNode::detectionsCallback, this, std::placeholders::_1));
    animal_pub_ = create_publisher<std_msgs::msg::Int32>(animal_topic_, rclcpp::QoS(10).reliable());
    mission_complete_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/mission_complete", rclcpp::QoS(10).reliable());
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, status_qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&WildlifePatrolTaskNode::timerCallback, this));
    status_heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WildlifePatrolTaskNode::publishStatusHeartbeat, this));

    flight_.publishActiveController(3);
    publishStatus("ready_waiting_route");
    RCLCPP_INFO(
      get_logger(),
      "Wildlife patrol ready. Waiting for route on %s, flight state, and YOLO detections on %s.",
      waypoints_topic_.c_str(), detections_topic_.c_str());
  }

private:
  static constexpr std::size_t kWaypointWidth = 4;
  static constexpr std::size_t kDetectionWidth = 8;

  static std::string jsonEscape(const std::string & value)
  {
    std::ostringstream escaped;
    for (const char character : value) {
      switch (character) {
        case '\\': escaped << "\\\\"; break;
        case '"': escaped << "\\\""; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default: escaped << character; break;
      }
    }
    return escaped.str();
  }

  const char * phaseName() const
  {
    switch (phase_) {
      case Phase::WAIT_ROUTE: return "WAIT_ROUTE";
      case Phase::WAIT_FLIGHT_STATE: return "WAIT_FLIGHT_STATE";
      case Phase::FLYING: return "FLYING";
      case Phase::LANDING_TO_10CM: return "LANDING_TO_10CM";
      case Phase::HOLDING: return "HOLDING";
    }
    return "UNKNOWN";
  }

  std::size_t currentWaypointIndex() const
  {
    if (phase_ == Phase::WAIT_ROUTE || phase_ == Phase::WAIT_FLIGHT_STATE || route_.empty()) {
      return 0;
    }
    return std::min(current_index_ + 1, route_.size());
  }

  void publishStatus(const std::string & note)
  {
    last_status_note_ = note;
    std_msgs::msg::String message;
    std::ostringstream out;
    out << "{"
        << "\"phase\":\"" << phaseName() << "\","
        << "\"ready\":" << (flight_ready_ ? "true" : "false") << ','
        << "\"route_valid\":" << (route_valid_ ? "true" : "false") << ','
        << "\"route_accepted\":" << (route_accepted_ ? "true" : "false") << ','
        << "\"waypoint_count\":" << route_.size() << ','
        << "\"current_waypoint_index\":" << currentWaypointIndex() << ','
        << "\"note\":\"" << jsonEscape(last_status_note_) << "\","
        << "\"error\":\"" << jsonEscape(last_status_error_) << "\"}";
    message.data = out.str();
    status_pub_->publish(message);
  }

  void publishStatusHeartbeat()
  {
    publishStatus(last_status_note_);
  }

  void publishMissionComplete()
  {
    if (mission_complete_sent_) {
      return;
    }
    flight_.publishActiveController(3);
    std_msgs::msg::Empty message;
    mission_complete_pub_->publish(message);
    mission_complete_sent_ = true;
    phase_ = Phase::HOLDING;
    publishStatus("task_complete");
    RCLCPP_INFO(get_logger(), "Wildlife patrol task complete; published /mission_complete.");
  }

  bool isExpected(double actual, double expected) const
  {
    return std::fabs(actual - expected) <= 0.01;
  }

  bool parseRoute(
    const std_msgs::msg::Float32MultiArray & message,
    std::vector<WaypointTarget> & route,
    std::string & error) const
  {
    if (message.data.empty()) {
      error = "route is empty";
      return false;
    }
    if (message.data.size() % kWaypointWidth != 0) {
      error = "route length must be a multiple of 4";
      return false;
    }

    route.clear();
    route.reserve(message.data.size() / kWaypointWidth);
    for (std::size_t index = 0; index < message.data.size(); index += kWaypointWidth) {
      const auto x_cm = static_cast<double>(message.data[index]);
      const auto y_cm = static_cast<double>(message.data[index + 1]);
      const auto z_cm = static_cast<double>(message.data[index + 2]);
      const auto yaw_deg = static_cast<double>(message.data[index + 3]);
      if (!std::isfinite(x_cm) || !std::isfinite(y_cm) || !std::isfinite(z_cm) ||
        !std::isfinite(yaw_deg))
      {
        error = "route contains a non-finite value";
        return false;
      }
      // Cruise and landing steps may use different heights (for example z=0 for landing).
      // Only the fixed first takeoff waypoint is required to have takeoff_z_cm_.
      if (!isExpected(yaw_deg, route_yaw_deg_)) {
        error = "route yaw does not match the wildlife ground-station contract";
        return false;
      }
      route.push_back(WaypointTarget{x_cm, y_cm, z_cm, yaw_deg});
    }

    const auto & first = route.front();
    if (!isExpected(first.x_cm, takeoff_x_cm_) || !isExpected(first.y_cm, takeoff_y_cm_) ||
      !isExpected(first.z_cm, takeoff_z_cm_) || !isExpected(first.yaw_deg, takeoff_yaw_deg_))
    {
      error = "first route waypoint must be the fixed takeoff point";
      return false;
    }
    return true;
  }

  void waypointsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (phase_ != Phase::WAIT_ROUTE) {
      RCLCPP_WARN(get_logger(), "Ignoring route: patrol has already started or is holding at its final waypoint.");
      last_status_error_ = "route ignored because patrol has already started or completed";
      publishStatus("route_ignored");
      return;
    }

    std::vector<WaypointTarget> parsed_route;
    std::string error;
    if (!parseRoute(*message, parsed_route, error)) {
      RCLCPP_WARN(get_logger(), "Rejected wildlife route: %s.", error.c_str());
      route_valid_ = false;
      route_accepted_ = false;
      last_status_error_ = error;
      publishStatus("route_rejected");
      return;
    }
    route_ = std::move(parsed_route);
    route_valid_ = true;
    route_accepted_ = true;
    last_status_error_.clear();
    phase_ = Phase::WAIT_FLIGHT_STATE;
    publishStatus("route_accepted");
    RCLCPP_INFO(get_logger(), "Accepted wildlife route with %zu waypoints.", route_.size());
  }

  void detectionsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    std::array<bool, 5> seen_classes{};
    if (message->data.size() % kDetectionWidth != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring malformed YOLO detections: expected groups of %zu floats, got %zu.",
        kDetectionWidth, message->data.size());
      updateAnimalConfirmation(seen_classes);
      return;
    }
    for (std::size_t index = 0; index < message->data.size(); index += kDetectionWidth) {
      const double raw_class_id = static_cast<double>(message->data[index]);
      const double confidence = static_cast<double>(message->data[index + 1]);
      if (!std::isfinite(raw_class_id)) {
        continue;
      }
      if (!std::isfinite(confidence) || confidence < animal_confidence_threshold_) {
        continue;
      }
      const int class_id = static_cast<int>(raw_class_id);
      if (raw_class_id != static_cast<double>(class_id) || class_id < 0 || class_id > 4) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring unsupported animal class id %.3f.", raw_class_id);
        continue;
      }
      seen_classes[static_cast<std::size_t>(class_id)] = true;
    }
    updateAnimalConfirmation(seen_classes);
  }

  void updateAnimalConfirmation(const std::array<bool, 5> & seen_classes)
  {
    const int required_frames = std::max(1, animal_confirm_frames_);
    for (std::size_t class_id = 0; class_id < seen_classes.size(); ++class_id) {
      if (!seen_classes[class_id]) {
        animal_streaks_[class_id] = 0;
        animal_latched_[class_id] = false;
        continue;
      }

      animal_streaks_[class_id] = std::min(animal_streaks_[class_id] + 1, required_frames);
      if (animal_streaks_[class_id] >= required_frames && !animal_latched_[class_id]) {
        std_msgs::msg::Int32 animal;
        animal.data = static_cast<std::int32_t>(class_id);
        animal_pub_->publish(animal);
        animal_latched_[class_id] = true;
        RCLCPP_INFO(
          get_logger(), "Animal confirmed: class=%zu confidence>=%.2f over %d frames.",
          class_id, animal_confidence_threshold_, required_frames);
      }
    }
  }

  void timerCallback()
  {
    if (phase_ == Phase::WAIT_ROUTE) {
      const bool was_ready = flight_ready_;
      flight_ready_ = flight_.hasState();
      if (flight_ready_ != was_ready) {
        publishStatus(flight_ready_ ? "flight_state_ready" : "waiting_flight_state");
      }
      return;
    }

    if (phase_ == Phase::WAIT_FLIGHT_STATE) {
      if (!flight_.hasState()) {
        return;
      }
      flight_ready_ = true;
      current_index_ = 0;
      phase_ = Phase::FLYING;
      flight_.goTo(route_.front());
      publishStatus("patrol_started");
      RCLCPP_INFO(get_logger(), "Wildlife patrol started from the fixed takeoff waypoint.");
      return;
    }

    if (phase_ == Phase::LANDING_TO_10CM) {
      if (flight_.isReached()) {
        publishMissionComplete();
      }
      return;
    }

    if (phase_ != Phase::FLYING || !flight_.isReached()) {
      return;
    }

    ++current_index_;
    if (current_index_ >= route_.size()) {
      const auto & final_waypoint = route_.back();
      if (final_waypoint.z_cm <= 10.0) {
        publishMissionComplete();
      } else {
        landing_target_ = final_waypoint;
        landing_target_.z_cm = 10.0;
        phase_ = Phase::LANDING_TO_10CM;
        flight_.goTo(landing_target_);
        publishStatus("landing_to_10cm");
        RCLCPP_INFO(
          get_logger(), "Final waypoint is %.1fcm high; descending to 10.0cm before completion.",
          final_waypoint.z_cm);
      }
      return;
    }
    flight_.goTo(route_[current_index_]);
    publishStatus("waypoint_advanced");
  }

  enum class Phase
  {
    WAIT_ROUTE,
    WAIT_FLIGHT_STATE,
    FLYING,
    LANDING_TO_10CM,
    HOLDING,
  };

  WaypointNavigator flight_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr animal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_heartbeat_timer_;
  std::string waypoints_topic_;
  std::string detections_topic_;
  std::string animal_topic_;
  std::string status_topic_;
  double timer_period_sec_{0.05};
  double takeoff_x_cm_{0.0};
  double takeoff_y_cm_{0.0};
  double takeoff_z_cm_{120.0};
  double takeoff_yaw_deg_{0.0};
  double route_height_cm_{120.0};
  double route_yaw_deg_{0.0};
  double animal_confidence_threshold_{0.70};
  int animal_confirm_frames_{3};
  std::vector<WaypointTarget> route_;
  std::array<int, 5> animal_streaks_{};
  std::array<bool, 5> animal_latched_{};
  WaypointTarget landing_target_{};
  std::size_t current_index_{0};
  bool flight_ready_{false};
  bool route_valid_{false};
  bool route_accepted_{false};
  std::string last_status_note_{"ready_waiting_route"};
  std::string last_status_error_;
  bool mission_complete_sent_{false};
  Phase phase_{Phase::WAIT_ROUTE};
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::WildlifePatrolTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
