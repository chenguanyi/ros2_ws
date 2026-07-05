#include "activity_control_pkg/core/aux_controller.hpp"
#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace activity_control_pkg
{
namespace tasks
{

struct FixedWaypoint
{
  WaypointTarget target;
  std::uint8_t arm_state{0};
  std::uint8_t magnet_state{0};
  std::uint8_t signal_state{0};
  double hold_sec{0.0};
};

class FixedWaypointTaskNode : public rclcpp::Node
{
public:
  FixedWaypointTaskNode()
  : rclcpp::Node("fixed_waypoint_task_node"),
    flight_(*this),
    aux_(*this)
  {
    timer_period_sec_ = declare_parameter("timer_period_sec", 0.05);
    configured_waypoints_ = declare_parameter("waypoints", std::vector<double>{});
    configured_arm_states_ = declare_parameter("arm_states", std::vector<int64_t>{});
    configured_magnet_states_ = declare_parameter("magnet_states", std::vector<int64_t>{});
    configured_signal_states_ = declare_parameter("signal_states", std::vector<int64_t>{});
    configured_hold_seconds_ = declare_parameter("hold_seconds", std::vector<double>{});

    loadWaypoints();

    mission_complete_pub_ =
      create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&FixedWaypointTaskNode::timerCallback, this));

    flight_.publishActiveController(3);
    RCLCPP_INFO(
      get_logger(),
      "Fixed waypoint task ready. Waiting for height and TF before starting.");
  }

private:
  void loadBinaryStates(
    const std::string & name,
    const std::vector<int64_t> & configured_values,
    std::size_t expected_size,
    std::vector<std::uint8_t> & parsed_values)
  {
    if (configured_values.empty()) {
      parsed_values.assign(expected_size, 0U);
      return;
    }

    if (configured_values.size() != expected_size) {
      throw std::runtime_error(
        name + " size must equal waypoint count " + std::to_string(expected_size) +
        ", got " + std::to_string(configured_values.size()) + ".");
    }

    parsed_values.clear();
    parsed_values.reserve(expected_size);
    for (std::size_t index = 0; index < configured_values.size(); ++index) {
      const auto value = configured_values[index];
      if (value != 0 && value != 1) {
        throw std::runtime_error(
          name + "[" + std::to_string(index) + "] must be 0 or 1, got " +
          std::to_string(value) + ".");
      }
      parsed_values.push_back(static_cast<std::uint8_t>(value));
    }
  }

  void loadHoldSeconds(std::size_t expected_size, std::vector<double> & parsed_values)
  {
    if (configured_hold_seconds_.empty()) {
      parsed_values.assign(expected_size, 0.0);
      return;
    }

    if (configured_hold_seconds_.size() != expected_size) {
      throw std::runtime_error(
        "hold_seconds size must equal waypoint count " + std::to_string(expected_size) +
        ", got " + std::to_string(configured_hold_seconds_.size()) + ".");
    }

    parsed_values.clear();
    parsed_values.reserve(expected_size);
    for (std::size_t index = 0; index < configured_hold_seconds_.size(); ++index) {
      const double value = configured_hold_seconds_[index];
      if (value < 0.0) {
        throw std::runtime_error(
          "hold_seconds[" + std::to_string(index) + "] must be >= 0.0, got " +
          std::to_string(value) + ".");
      }
      parsed_values.push_back(value);
    }
  }

  void loadWaypoints()
  {
    if (configured_waypoints_.empty()) {
      throw std::runtime_error(
        "No waypoints configured. Use 'waypoints: [x_cm, y_cm, z_cm, yaw_deg, ...]'.");
    }

    if (configured_waypoints_.size() % 4 != 0) {
      throw std::runtime_error(
        "Waypoint array size must be a multiple of 4, got " +
        std::to_string(configured_waypoints_.size()) + ".");
    }

    const std::size_t waypoint_count = configured_waypoints_.size() / 4;
    std::vector<std::uint8_t> arm_states;
    std::vector<std::uint8_t> magnet_states;
    std::vector<std::uint8_t> signal_states;
    std::vector<double> hold_seconds;

    loadBinaryStates("arm_states", configured_arm_states_, waypoint_count, arm_states);
    loadBinaryStates("magnet_states", configured_magnet_states_, waypoint_count, magnet_states);
    loadBinaryStates("signal_states", configured_signal_states_, waypoint_count, signal_states);
    loadHoldSeconds(waypoint_count, hold_seconds);

    waypoints_.clear();
    waypoints_.reserve(waypoint_count);
    for (std::size_t index = 0; index < configured_waypoints_.size(); index += 4) {
      const std::size_t waypoint_index = index / 4;
      waypoints_.push_back(FixedWaypoint{
        WaypointTarget{
          configured_waypoints_[index],
          configured_waypoints_[index + 1],
          configured_waypoints_[index + 2],
          configured_waypoints_[index + 3],
        },
        arm_states[waypoint_index],
        magnet_states[waypoint_index],
        signal_states[waypoint_index],
        hold_seconds[waypoint_index],
      });
    }

    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      const auto & waypoint = waypoints_[index];
      RCLCPP_INFO(
        get_logger(),
        "Loaded waypoint %zu: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg arm=%u magnet=%u signal=%u hold=%.2fs",
        index + 1,
        waypoint.target.x_cm,
        waypoint.target.y_cm,
        waypoint.target.z_cm,
        waypoint.target.yaw_deg,
        static_cast<unsigned>(waypoint.arm_state),
        static_cast<unsigned>(waypoint.magnet_state),
        static_cast<unsigned>(waypoint.signal_state),
        waypoint.hold_sec);
    }
  }

  void startTask()
  {
    current_index_ = 0;
    task_started_ = true;
    action_sent_ = false;
    flight_.goTo(waypoints_.front().target);
    RCLCPP_INFO(get_logger(), "Fixed waypoint task started.");
  }

  void completeTask()
  {
    if (mission_complete_sent_) {
      return;
    }

    flight_.publishActiveController(3);
    std_msgs::msg::Empty message;
    mission_complete_pub_->publish(message);
    mission_complete_sent_ = true;
    RCLCPP_INFO(get_logger(), "Fixed waypoint task completed.");
  }

  void timerCallback()
  {
    if (mission_complete_sent_) {
      return;
    }

    if (!task_started_) {
      if (!flight_.hasState()) {
        return;
      }
      startTask();
      return;
    }

    if (current_index_ >= waypoints_.size()) {
      completeTask();
      return;
    }

    const auto & waypoint = waypoints_[current_index_];
    if (!flight_.isReached()) {
      return;
    }

    if (!action_sent_) {
      aux_.setAll(waypoint.arm_state, waypoint.magnet_state, waypoint.signal_state);
      action_sent_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Reached waypoint %zu/%zu and published aux actions.",
        current_index_ + 1,
        waypoints_.size());
    }

    if (!flight_.hold(waypoint.hold_sec)) {
      return;
    }

    ++current_index_;
    action_sent_ = false;
    if (current_index_ >= waypoints_.size()) {
      completeTask();
      return;
    }

    flight_.goTo(waypoints_[current_index_].target);
  }

  WaypointNavigator flight_;
  AuxController aux_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  double timer_period_sec_{0.05};
  std::vector<double> configured_waypoints_;
  std::vector<int64_t> configured_arm_states_;
  std::vector<int64_t> configured_magnet_states_;
  std::vector<int64_t> configured_signal_states_;
  std::vector<double> configured_hold_seconds_;
  std::vector<FixedWaypoint> waypoints_;
  std::size_t current_index_{0};
  bool task_started_{false};
  bool action_sent_{false};
  bool mission_complete_sent_{false};
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::FixedWaypointTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
