#include "activity_control_pkg/core/aux_controller.hpp"
#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace activity_control_pkg
{
namespace tasks
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kCruiseHeightCm = 150.0;
constexpr double kHomeZCm = 10.0;
constexpr double kScanClearanceCm = 60.0;
constexpr double kRackHalfThicknessCm = 0.0;
constexpr double kRackYMinFieldCm = 100.0;
constexpr double kRackYMaxFieldCm = 300.0;
constexpr double kTakeoffFieldXCm = 75.0;
constexpr double kTakeoffFieldYCm = 75.0;
constexpr double kLandingFieldXCm = 425.0;
constexpr double kLandingFieldYCm = 325.0;
constexpr double kUpperSlotZCm = 140.0;
constexpr double kLowerSlotZCm = 100.0;
constexpr double kLaserOnSec = 0.5;
constexpr double kLedOnSec = 1.0;
constexpr double kScanTimeoutSec = 4.0;
constexpr double kVisualLockTimeoutSec = 1.6;
constexpr double kBarcodeFreshSec = 0.6;
constexpr double kFineDataFreshSec = 0.4;
constexpr int kFineDataDeadzonePx = 28;
constexpr double kRouteXySpeedCmS = 36.0;
constexpr double kRouteZSpeedCmS = 30.0;
constexpr double kRouteYawSpeedDegS = 25.0;
constexpr double kTurnYawThresholdDeg = 2.0;
constexpr int kInventoryCount = 24;

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct SafetyRect
{
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};
};

struct RackSpec
{
  int index{0};
  double x_field_cm{0.0};
};

struct FaceSpec
{
  char face{'A'};
  int rack_index{0};
  int side_sign{-1};
  double yaw_deg{0.0};
};

struct SlotTarget
{
  std::string coord;
  char face{'A'};
  int slot{0};
  WaypointTarget target;
};

struct RouteStep
{
  WaypointTarget target;
  bool scan{false};
  std::string coord;
  std::string kind;
};

struct InventoryRecord
{
  int id{0};
  std::string coord;
  WaypointTarget target;
  double time_sec{0.0};
};

enum class MissionMode
{
  INVENTORY,
  TARGET,
};

enum class MissionPhase
{
  WAIT_MODE,
  WAIT_TARGET_BARCODE,
  WAIT_GROUND_ROUTE,
  WAIT_STATE,
  TAKEOFF,
  TRANSIT,
  GO_SCAN,
  SCAN,
  LASER_ON,
  LED_ON,
  RETURN_TRANSIT,
  DESCEND,
  COMPLETE,
  STOPPED,
};

const char * phaseName(MissionPhase phase)
{
  switch (phase) {
    case MissionPhase::WAIT_MODE:
      return "WAIT_MODE";
    case MissionPhase::WAIT_TARGET_BARCODE:
      return "WAIT_TARGET_BARCODE";
    case MissionPhase::WAIT_GROUND_ROUTE:
      return "WAIT_GROUND_ROUTE";
    case MissionPhase::WAIT_STATE:
      return "WAIT_STATE";
    case MissionPhase::TAKEOFF:
      return "TAKEOFF";
    case MissionPhase::TRANSIT:
      return "TRANSIT";
    case MissionPhase::GO_SCAN:
      return "GO_SCAN";
    case MissionPhase::SCAN:
      return "SCAN";
    case MissionPhase::LASER_ON:
      return "LASER_ON";
    case MissionPhase::LED_ON:
      return "LED_ON";
    case MissionPhase::RETURN_TRANSIT:
      return "RETURN_TRANSIT";
    case MissionPhase::DESCEND:
      return "DESCEND";
    case MissionPhase::COMPLETE:
      return "COMPLETE";
    case MissionPhase::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

double normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double fieldToMapX(double x_field_cm)
{
  return x_field_cm - kTakeoffFieldXCm;
}

double fieldToMapY(double y_field_cm)
{
  return y_field_cm - kTakeoffFieldYCm;
}

double correctedMapYFromOldMapX(double old_x_cm)
{
  constexpr double kFirstRackNearScanLineOldMapXCm = 15.0;
  if (std::fabs(old_x_cm - kFirstRackNearScanLineOldMapXCm) < 1e-6) {
    return 0.0;
  }
  return -old_x_cm;
}

Point2 oldMapToFlightMap(double old_x_cm, double old_y_cm)
{
  return Point2{old_y_cm, correctedMapYFromOldMapX(old_x_cm)};
}

WaypointTarget oldMapToFlightTarget(
  double old_x_cm,
  double old_y_cm,
  double z_cm,
  double yaw_deg)
{
  const auto point = oldMapToFlightMap(old_x_cm, old_y_cm);
  return WaypointTarget{point.x, point.y, z_cm, yaw_deg};
}

SafetyRect oldMapRectToFlightMap(
  double old_x_min,
  double old_y_min,
  double old_x_max,
  double old_y_max)
{
  const std::array<Point2, 4> corners{
    oldMapToFlightMap(old_x_min, old_y_min),
    oldMapToFlightMap(old_x_min, old_y_max),
    oldMapToFlightMap(old_x_max, old_y_min),
    oldMapToFlightMap(old_x_max, old_y_max),
  };

  SafetyRect rect{
    corners.front().x,
    corners.front().y,
    corners.front().x,
    corners.front().y,
  };
  for (const auto & corner : corners) {
    rect.x_min = std::min(rect.x_min, corner.x);
    rect.y_min = std::min(rect.y_min, corner.y);
    rect.x_max = std::max(rect.x_max, corner.x);
    rect.y_max = std::max(rect.y_max, corner.y);
  }
  return rect;
}

double distance2d(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point2 pointOf(const WaypointTarget & target)
{
  return Point2{target.x_cm, target.y_cm};
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream out;
  for (const char ch : input) {
    if (ch == '\\' || ch == '"') {
      out << '\\' << ch;
    } else if (ch == '\n' || ch == '\r') {
      out << ' ';
    } else {
      out << ch;
    }
  }
  return out.str();
}

bool pointInsideOpenRect(const Point2 & point, const SafetyRect & rect)
{
  constexpr double kEpsilon = 1e-6;
  return point.x > rect.x_min + kEpsilon &&
         point.x < rect.x_max - kEpsilon &&
         point.y > rect.y_min + kEpsilon &&
         point.y < rect.y_max - kEpsilon;
}

bool segmentIntersectsOpenRect(const Point2 & start, const Point2 & end, const SafetyRect & rect)
{
  if (pointInsideOpenRect(start, rect) || pointInsideOpenRect(end, rect)) {
    return true;
  }

  constexpr double kEpsilon = 1e-6;
  const double x_min = rect.x_min + kEpsilon;
  const double x_max = rect.x_max - kEpsilon;
  const double y_min = rect.y_min + kEpsilon;
  const double y_max = rect.y_max - kEpsilon;
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  double t0 = 0.0;
  double t1 = 1.0;

  auto clip = [&t0, &t1](double p, double q) {
      if (std::fabs(p) < 1e-12) {
        return q >= 0.0;
      }
      const double r = q / p;
      if (p < 0.0) {
        if (r > t1) {
          return false;
        }
        if (r > t0) {
          t0 = r;
        }
      } else {
        if (r < t0) {
          return false;
        }
        if (r < t1) {
          t1 = r;
        }
      }
      return true;
    };

  if (!clip(-dx, start.x - x_min)) {
    return false;
  }
  if (!clip(dx, x_max - start.x)) {
    return false;
  }
  if (!clip(-dy, start.y - y_min)) {
    return false;
  }
  if (!clip(dy, y_max - start.y)) {
    return false;
  }

  return t0 <= t1 && t1 >= 0.0 && t0 <= 1.0;
}

double segmentTimeCost(const WaypointTarget & start, const WaypointTarget & end)
{
  const double xy_time =
    std::hypot(end.x_cm - start.x_cm, end.y_cm - start.y_cm) / kRouteXySpeedCmS;
  const double z_time = std::fabs(end.z_cm - start.z_cm) / kRouteZSpeedCmS;
  const double yaw_time =
    std::fabs(normalizeAngleDeg(end.yaw_deg - start.yaw_deg)) / kRouteYawSpeedDegS;
  return std::max({xy_time, z_time, yaw_time});
}

double yawTurnTimeCost(double start_yaw_deg, double end_yaw_deg)
{
  return std::fabs(normalizeAngleDeg(end_yaw_deg - start_yaw_deg)) / kRouteYawSpeedDegS;
}

}  // namespace

class WarehouseInventoryTaskNode : public rclcpp::Node
{
public:
  WarehouseInventoryTaskNode()
  : rclcpp::Node("warehouse_inventory_task_node"),
    flight_(*this),
    aux_(*this)
  {
    const auto mode_text = declare_parameter("mission_mode", std::string("inventory"));
    mission_mode_ = parseMissionMode(mode_text);
    timer_period_sec_ = declare_parameter("timer_period_sec", 0.05);
    mode_select_grace_sec_ = declare_parameter("mode_select_grace_sec", 0.5);
    barcode_topic_ =
      declare_parameter("barcode_topic", std::string("/warehouse_inventory/barcode_value"));
    fine_data_topic_ = declare_parameter("fine_data_topic", std::string("/fine_data"));
    ground_mode_topic_ = declare_parameter("ground_mode_topic", std::string("/d_task/mode"));
    ground_route_topic_ = declare_parameter("ground_route_topic", std::string("/d_task/route"));
    d_task_qr_id_topic_ = declare_parameter("d_task_qr_id_topic", std::string("/d_task/qr_id"));
    d_task_status_topic_ =
      declare_parameter("d_task_status_topic", std::string("/d_task/status"));

    if (timer_period_sec_ <= 0.0) {
      throw std::runtime_error("timer_period_sec must be > 0.");
    }
    if (mode_select_grace_sec_ < 0.0) {
      throw std::runtime_error("mode_select_grace_sec must be >= 0.");
    }

    log_dir_ = findLogDir();
    std::filesystem::create_directories(log_dir_);
    openEventCsv();

    buildFieldModel();
    active_steps_ = planBestInventoryRoute();
    publishable_route_ = routeWithLanding(active_steps_);

    const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    mission_complete_pub_ =
      create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
    visual_takeover_pub_ =
      create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
    route_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/route", durable_qos);
    scan_result_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/scan_result", durable_qos);
    all_results_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/all_results", durable_qos);
    target_id_pub_ =
      create_publisher<std_msgs::msg::Int32>("/warehouse_inventory/target_id", durable_qos);
    qr_id_pub_ =
      create_publisher<std_msgs::msg::Int32>(d_task_qr_id_topic_, durable_qos);
    query_result_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/query_result", durable_qos);
    d_task_status_pub_ =
      create_publisher<std_msgs::msg::String>(d_task_status_topic_, durable_qos);

    const auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    barcode_sub_ = create_subscription<std_msgs::msg::Int32>(
      barcode_topic_,
      barcode_qos,
      std::bind(&WarehouseInventoryTaskNode::barcodeCallback, this, std::placeholders::_1));
    fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      fine_data_topic_,
      rclcpp::QoS(10),
      std::bind(&WarehouseInventoryTaskNode::fineDataCallback, this, std::placeholders::_1));
    query_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/warehouse_inventory/query",
      rclcpp::QoS(10),
      std::bind(&WarehouseInventoryTaskNode::queryCallback, this, std::placeholders::_1));
    ground_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      ground_mode_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(&WarehouseInventoryTaskNode::groundModeCallback, this, std::placeholders::_1));
    ground_route_sub_ = create_subscription<std_msgs::msg::String>(
      ground_route_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(&WarehouseInventoryTaskNode::groundRouteCallback, this, std::placeholders::_1));

    publishRoute(publishable_route_);

    phase_ = mission_mode_ == MissionMode::TARGET ?
      MissionPhase::WAIT_TARGET_BARCODE : MissionPhase::WAIT_MODE;
    mode_select_deadline_ = now() + rclcpp::Duration::from_seconds(mode_select_grace_sec_);
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);
    publishDTaskStatus("ready");

    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&WarehouseInventoryTaskNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Warehouse inventory task ready: mode=%s, route_steps=%zu, safety_clearance=%.1fcm, "
      "mode_topic=%s route_topic=%s grace=%.2fs.",
      mission_mode_ == MissionMode::TARGET ? "target" : "inventory",
      active_steps_.size(),
      kScanClearanceCm,
      ground_mode_topic_.c_str(),
      ground_route_topic_.c_str(),
      mode_select_grace_sec_);
  }

private:
  MissionMode parseMissionMode(const std::string & mode_text) const
  {
    if (mode_text == "inventory") {
      return MissionMode::INVENTORY;
    }
    if (mode_text == "target") {
      return MissionMode::TARGET;
    }
    throw std::runtime_error("mission_mode must be 'inventory' or 'target'.");
  }

  std::string findLogDir() const
  {
    const char * ros_log_dir = std::getenv("ROS_LOG_DIR");
    if (ros_log_dir != nullptr && ros_log_dir[0] != '\0') {
      return ros_log_dir;
    }
    return "./mylog/warehouse_inventory";
  }

  void buildFieldModel()
  {
    racks_ = {
      RackSpec{0, 150.0},
      RackSpec{1, 350.0},
    };
    faces_ = {
      FaceSpec{'A', 0, -1, 0.0},
      FaceSpec{'B', 0, 1, 180.0},
      FaceSpec{'C', 1, -1, 0.0},
      FaceSpec{'D', 1, 1, 180.0},
    };

    safety_rects_.clear();
    for (const auto & rack : racks_) {
      const double rack_old_x = fieldToMapX(rack.x_field_cm);
      safety_rects_.push_back(oldMapRectToFlightMap(
        rack_old_x - kRackHalfThicknessCm - kScanClearanceCm,
        fieldToMapY(kRackYMinFieldCm - kScanClearanceCm),
        rack_old_x + kRackHalfThicknessCm + kScanClearanceCm,
        fieldToMapY(kRackYMaxFieldCm + kScanClearanceCm)));
    }

    slots_by_face_.clear();
    const std::array<double, 3> slot_y_field{150.0, 200.0, 250.0};
    for (const auto & face : faces_) {
      const auto rack_iter = std::find_if(
        racks_.begin(),
        racks_.end(),
        [&face](const RackSpec & rack) {
          return rack.index == face.rack_index;
        });
      if (rack_iter == racks_.end()) {
        throw std::runtime_error("Face references unknown rack.");
      }

      std::vector<SlotTarget> slots;
      slots.reserve(6);
      for (int col = 0; col < 3; ++col) {
        slots.push_back(makeSlotTarget(face, col + 1, slot_y_field[col], kUpperSlotZCm));
      }
      for (int col = 0; col < 3; ++col) {
        slots.push_back(makeSlotTarget(face, col + 4, slot_y_field[col], kLowerSlotZCm));
      }
      slots_by_face_[face.face] = slots;
    }
  }

  SlotTarget makeSlotTarget(
    const FaceSpec & face,
    int slot,
    double slot_y_field_cm,
    double slot_z_cm) const
  {
    const auto rack_iter = std::find_if(
      racks_.begin(),
      racks_.end(),
      [&face](const RackSpec & rack) {
        return rack.index == face.rack_index;
      });
    const double rack_x_field = rack_iter->x_field_cm;
    const double scan_x_field =
      rack_x_field + static_cast<double>(face.side_sign) * kScanClearanceCm;
    std::ostringstream coord;
    coord << face.face << slot;
    return SlotTarget{
      coord.str(),
      face.face,
      slot,
      oldMapToFlightTarget(
        fieldToMapX(scan_x_field),
        fieldToMapY(slot_y_field_cm),
        slot_z_cm,
        face.yaw_deg),
    };
  }

  std::vector<std::vector<SlotTarget>> faceVariants(char face) const
  {
    const auto iter = slots_by_face_.find(face);
    if (iter == slots_by_face_.end()) {
      throw std::runtime_error("Unknown face in route planning.");
    }
    const auto & slots = iter->second;
    auto by_slot = [&slots](int slot) -> SlotTarget {
        const auto found = std::find_if(
          slots.begin(),
          slots.end(),
          [slot](const SlotTarget & target) {
            return target.slot == slot;
          });
        if (found == slots.end()) {
          throw std::runtime_error("Missing face slot.");
        }
        return *found;
      };

    std::vector<std::vector<int>> orders{
      {1, 2, 3, 6, 5, 4},
      {3, 2, 1, 4, 5, 6},
    };

    std::vector<std::vector<SlotTarget>> variants;
    for (const auto & order : orders) {
      std::vector<SlotTarget> variant;
      variant.reserve(order.size());
      for (const int slot : order) {
        variant.push_back(by_slot(slot));
      }
      variants.push_back(variant);
    }
    return variants;
  }

  bool segmentBlocked(const Point2 & start, const Point2 & end) const
  {
    for (const auto & rect : safety_rects_) {
      if (segmentIntersectsOpenRect(start, end, rect)) {
        return true;
      }
    }
    return false;
  }

  std::vector<Point2> safePath2d(const Point2 & start, const Point2 & end) const
  {
    std::vector<Point2> nodes{start, end};
    for (const auto & rect : safety_rects_) {
      nodes.push_back(Point2{rect.x_min, rect.y_min});
      nodes.push_back(Point2{rect.x_min, rect.y_max});
      nodes.push_back(Point2{rect.x_max, rect.y_min});
      nodes.push_back(Point2{rect.x_max, rect.y_max});
    }

    const std::size_t n = nodes.size();
    std::vector<std::vector<std::pair<std::size_t, double>>> graph(n);
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        if (segmentBlocked(nodes[i], nodes[j])) {
          continue;
        }
        const double cost = distance2d(nodes[i], nodes[j]);
        graph[i].push_back({j, cost});
        graph[j].push_back({i, cost});
      }
    }

    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n, kInf);
    std::vector<std::size_t> prev(n, n);
    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    dist[0] = 0.0;
    queue.push({0.0, 0});
    while (!queue.empty()) {
      const auto [cost, node] = queue.top();
      queue.pop();
      if (cost > dist[node]) {
        continue;
      }
      if (node == 1) {
        break;
      }
      for (const auto & edge : graph[node]) {
        const double next_cost = cost + edge.second;
        if (next_cost < dist[edge.first]) {
          dist[edge.first] = next_cost;
          prev[edge.first] = node;
          queue.push({next_cost, edge.first});
        }
      }
    }

    if (!std::isfinite(dist[1])) {
      return {start, end};
    }

    std::vector<Point2> path;
    for (std::size_t at = 1; at != n; at = prev[at]) {
      path.push_back(nodes[at]);
      if (at == 0) {
        break;
      }
    }
    std::reverse(path.begin(), path.end());
    if (path.empty() || distance2d(path.front(), start) > 1e-6) {
      path.insert(path.begin(), start);
    }
    return path;
  }

  std::vector<WaypointTarget> safeTransitWaypoints(
    const WaypointTarget & start,
    const WaypointTarget & end) const
  {
    const auto path = safePath2d(pointOf(start), pointOf(end));
    std::vector<WaypointTarget> result;
    if (path.size() <= 2) {
      return result;
    }

    const double transit_z = start.z_cm;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      if (distance2d(path[i], path[i - 1]) < 0.5) {
        continue;
      }
      result.push_back(WaypointTarget{path[i].x, path[i].y, transit_z, end.yaw_deg});
    }
    return result;
  }

  void appendTurnInPlace(
    WaypointTarget & current,
    double target_yaw_deg,
    std::vector<RouteStep> & steps) const
  {
    const double dyaw = normalizeAngleDeg(target_yaw_deg - current.yaw_deg);
    if (std::fabs(dyaw) <= kTurnYawThresholdDeg) {
      current.yaw_deg = target_yaw_deg;
      return;
    }

    current.yaw_deg = target_yaw_deg;
    steps.push_back(RouteStep{current, false, "", "turn"});
  }

  void appendTransit(
    const WaypointTarget & start,
    const WaypointTarget & end,
    std::vector<RouteStep> & steps) const
  {
    WaypointTarget current = start;
    appendTurnInPlace(current, end.yaw_deg, steps);
    const auto transit = safeTransitWaypoints(current, end);
    for (const auto & target : transit) {
      steps.push_back(RouteStep{target, false, "", "transit"});
    }
  }

  double safeCostBetween(const WaypointTarget & start, const WaypointTarget & end) const
  {
    double cost = 0.0;
    WaypointTarget current = start;
    cost += yawTurnTimeCost(current.yaw_deg, end.yaw_deg);
    current.yaw_deg = end.yaw_deg;
    auto transit = safeTransitWaypoints(current, end);
    for (const auto & waypoint : transit) {
      cost += segmentTimeCost(current, waypoint);
      current = waypoint;
    }
    cost += segmentTimeCost(current, end);
    return cost;
  }

  std::vector<RouteStep> expandSequenceToSteps(const std::vector<SlotTarget> & sequence) const
  {
    std::vector<RouteStep> steps;
    WaypointTarget current = homeAtCruise();
    for (const auto & slot : sequence) {
      appendTransit(current, slot.target, steps);
      steps.push_back(RouteStep{slot.target, true, slot.coord, "scan"});
      current = slot.target;
    }
    return steps;
  }

  std::vector<RouteStep> planBestInventoryRoute() const
  {
    const std::vector<std::pair<char, std::vector<int>>> fixed_route{
      {'A', {1, 2, 3, 6, 5, 4}},
      {'C', {4, 5, 6, 3, 2, 1}},
      {'B', {1, 2, 3, 6, 5, 4}},
      {'D', {4, 5, 6, 3, 2, 1}},
    };

    auto slot_by_number = [this](char face, int slot_number) -> SlotTarget {
        const auto iter = slots_by_face_.find(face);
        if (iter == slots_by_face_.end()) {
          throw std::runtime_error("Missing face slots during planning.");
        }

        const auto found = std::find_if(
          iter->second.begin(),
          iter->second.end(),
          [slot_number](const SlotTarget & target) {
            return target.slot == slot_number;
          });
        if (found == iter->second.end()) {
          throw std::runtime_error("Missing face slot during planning.");
        }
        return *found;
      };

    std::vector<SlotTarget> sequence;
    sequence.reserve(kInventoryCount);
    for (const auto & [face, slot_numbers] : fixed_route) {
      for (const int slot_number : slot_numbers) {
        sequence.push_back(slot_by_number(face, slot_number));
      }
    }

    if (sequence.size() != kInventoryCount) {
      throw std::runtime_error("Failed to plan a complete 24-slot warehouse route.");
    }

    double route_cost = 0.0;
    WaypointTarget current = homeAtCruise();
    for (const auto & slot : sequence) {
      route_cost += safeCostBetween(current, slot.target);
      current = slot.target;
    }
    route_cost += safeCostBetween(current, landingAtCruise(current.yaw_deg));

    auto steps = expandSequenceToSteps(sequence);
    RCLCPP_INFO(
      get_logger(),
      "Planned warehouse route: scan_points=%zu expanded_steps=%zu score=%.3f.",
      sequence.size(),
      steps.size(),
      route_cost);
    logRouteSafety(steps);
    return steps;
  }

  void logRouteSafety(const std::vector<RouteStep> & steps) const
  {
    for (const auto & step : steps) {
      if (!step.scan) {
        continue;
      }
      RCLCPP_INFO(
        get_logger(),
        "Route scan %-2s x=%.1f y=%.1f z=%.1f yaw=%.1f clearance=%.1fcm.",
        step.coord.c_str(),
        step.target.x_cm,
        step.target.y_cm,
        step.target.z_cm,
        step.target.yaw_deg,
        kScanClearanceCm);
    }
  }

  WaypointTarget homeAtCruise() const
  {
    return WaypointTarget{0.0, 0.0, kCruiseHeightCm, 0.0};
  }

  WaypointTarget landingAtCruise(double yaw_deg = 0.0) const
  {
    return oldMapToFlightTarget(
      fieldToMapX(kLandingFieldXCm),
      fieldToMapY(kLandingFieldYCm),
      kCruiseHeightCm,
      yaw_deg);
  }

  WaypointTarget landingFinal(double yaw_deg = 0.0) const
  {
    return oldMapToFlightTarget(
      fieldToMapX(kLandingFieldXCm),
      fieldToMapY(kLandingFieldYCm),
      kHomeZCm,
      yaw_deg);
  }

  void openEventCsv()
  {
    std::ostringstream path;
    path << log_dir_ << "/events_" << now().nanoseconds() << ".csv";
    event_csv_path_ = path.str();
    event_csv_.open(event_csv_path_, std::ios::out | std::ios::trunc);
    if (!event_csv_.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open warehouse event CSV: %s", event_csv_path_.c_str());
      return;
    }
    event_csv_ <<
      "time_sec,event,phase,step_index,coord,barcode,target_x_cm,target_y_cm,target_z_cm,target_yaw_deg,note\n";
    event_csv_.flush();
    RCLCPP_INFO(get_logger(), "Warehouse event CSV: %s", event_csv_path_.c_str());
  }

  void logEvent(const std::string & event, const std::string & coord, int barcode, const std::string & note)
  {
    if (!event_csv_.is_open()) {
      return;
    }
    const auto target = flight_.currentTarget();
    event_csv_ << std::fixed << std::setprecision(6)
               << now().seconds() << ','
               << event << ','
               << phaseName(phase_) << ','
               << current_step_index_ << ','
               << coord << ','
               << barcode << ',';
    if (target) {
      event_csv_ << target->x_cm << ','
                 << target->y_cm << ','
                 << target->z_cm << ','
                 << target->yaw_deg << ',';
    } else {
      event_csv_ << ",,,,";
    }
    event_csv_ << jsonEscape(note) << '\n';
    event_csv_.flush();
  }

  std::string routeToJson(const std::vector<RouteStep> & steps) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"safety_clearance_cm\":" << kScanClearanceCm << ",\"steps\":[";
    for (std::size_t i = 0; i < steps.size(); ++i) {
      if (i > 0) {
        out << ',';
      }
      const auto & step = steps[i];
      out << "{\"kind\":\"" << step.kind << "\","
          << "\"scan\":" << (step.scan ? "true" : "false") << ','
          << "\"coord\":\"" << jsonEscape(step.coord) << "\","
          << "\"x_cm\":" << step.target.x_cm << ','
          << "\"y_cm\":" << step.target.y_cm << ','
          << "\"z_cm\":" << step.target.z_cm << ','
          << "\"yaw_deg\":" << step.target.yaw_deg << '}';
    }
    out << "]}";
    return out.str();
  }

  void publishRoute(const std::vector<RouteStep> & steps)
  {
    std_msgs::msg::String msg;
    msg.data = routeToJson(steps);
    route_pub_->publish(msg);
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
  }

  void publishDTaskStatus(const std::string & note)
  {
    if (!d_task_status_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{"
        << "\"mode\":\"" << (mission_mode_ == MissionMode::TARGET ? "target" : "inventory") << "\","
        << "\"mode_value\":" << (mission_mode_ == MissionMode::TARGET ? 1 : 0) << ','
        << "\"phase\":\"" << phaseName(phase_) << "\","
        << "\"locked\":" << (mission_locked_ ? "true" : "false") << ','
        << "\"target_id\":" << target_id_ << ','
        << "\"route_valid\":" << (ground_route_valid_ ? "true" : "false") << ','
        << "\"route_loaded\":" << (ground_route_loaded_ ? "true" : "false") << ','
        << "\"active_steps\":" << active_steps_.size() << ','
        << "\"current_step_index\":" << current_step_index_ << ','
        << "\"note\":\"" << jsonEscape(note) << "\"}";
    msg.data = out.str();
    d_task_status_pub_->publish(msg);
    last_status_pub_stamp_ = now();
  }

  void publishDTaskStatusThrottled(const std::string & note, double period_sec = 1.0)
  {
    if (last_status_pub_stamp_.nanoseconds() != 0 &&
      (now() - last_status_pub_stamp_).seconds() < period_sec)
    {
      return;
    }
    publishDTaskStatus(note);
  }

  bool modeSwitchAllowed() const
  {
    return !mission_locked_ &&
           (phase_ == MissionPhase::WAIT_MODE ||
            phase_ == MissionPhase::WAIT_STATE ||
            phase_ == MissionPhase::WAIT_TARGET_BARCODE ||
            phase_ == MissionPhase::WAIT_GROUND_ROUTE);
  }

  void selectInventoryMode(const std::string & reason)
  {
    mission_mode_ = MissionMode::INVENTORY;
    target_id_ = 0;
    accepted_barcode_ = 0;
    target_slot_.reset();
    ground_route_valid_ = false;
    ground_route_loaded_ = false;
    ground_route_steps_.clear();
    active_steps_ = planBestInventoryRoute();
    publishable_route_ = routeWithLanding(active_steps_);
    publishRoute(publishable_route_);
    phase_ = MissionPhase::WAIT_STATE;
    RCLCPP_INFO(get_logger(), "D task mode set to inventory by %s.", reason.c_str());
    logEvent("mode_inventory", "", 0, reason);
    publishDTaskStatus("mode_inventory_" + reason);
  }

  void selectGroundTargetMode(const std::string & reason)
  {
    mission_mode_ = MissionMode::TARGET;
    target_id_ = 0;
    accepted_barcode_ = 0;
    target_slot_.reset();
    ground_route_valid_ = false;
    ground_route_loaded_ = false;
    ground_route_steps_.clear();
    active_steps_.clear();
    publishable_route_.clear();
    phase_ = MissionPhase::WAIT_TARGET_BARCODE;
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);
    RCLCPP_INFO(get_logger(), "D task mode set to ground target by %s.", reason.c_str());
    logEvent("mode_ground_target", "", 0, reason);
    publishDTaskStatus("mode_ground_target_" + reason);
  }

  void groundModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!modeSwitchAllowed()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignored /d_task/mode=%u because mission is locked or not selectable.",
        static_cast<unsigned>(msg->data));
      publishDTaskStatus("mode_ignored_locked");
      return;
    }

    if (msg->data == 0U) {
      selectInventoryMode("ground_mode_0");
      return;
    }

    if (msg->data == 1U) {
      selectGroundTargetMode("ground_mode_1");
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Ignored invalid /d_task/mode=%u. Expected 0 or 1.",
      static_cast<unsigned>(msg->data));
    publishDTaskStatus("invalid_mode_value");
  }

  bool readRequiredNumber(
    const Json::Value & value,
    const std::string & key,
    std::size_t step_index,
    double & output,
    std::string & error) const
  {
    if (!value.isMember(key) || !value[key].isNumeric()) {
      error = "steps[" + std::to_string(step_index) + "]." + key + " must be a number";
      return false;
    }

    output = value[key].asDouble();
    if (!std::isfinite(output)) {
      error = "steps[" + std::to_string(step_index) + "]." + key + " must be finite";
      return false;
    }
    return true;
  }

  bool parseGroundRouteJson(
    const std::string & text,
    std::vector<RouteStep> & steps,
    std::string & error) const
  {
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    Json::Value root;
    std::string parse_errors;
    std::istringstream input(text);
    if (!Json::parseFromStream(builder, input, &root, &parse_errors)) {
      error = "invalid JSON: " + parse_errors;
      return false;
    }

    if (!root.isObject()) {
      error = "route root must be an object";
      return false;
    }

    const Json::Value & json_steps = root["steps"];
    if (!json_steps.isArray() || json_steps.empty()) {
      error = "route.steps must be a non-empty array";
      return false;
    }

    steps.clear();
    steps.reserve(json_steps.size());
    bool has_scan_step = false;
    for (Json::ArrayIndex i = 0; i < json_steps.size(); ++i) {
      const Json::Value & step = json_steps[i];
      if (!step.isObject()) {
        error = "steps[" + std::to_string(i) + "] must be an object";
        return false;
      }

      double x_cm = 0.0;
      double y_cm = 0.0;
      double z_cm = 0.0;
      double yaw_deg = 0.0;
      if (!readRequiredNumber(step, "x_cm", i, x_cm, error) ||
        !readRequiredNumber(step, "y_cm", i, y_cm, error) ||
        !readRequiredNumber(step, "z_cm", i, z_cm, error) ||
        !readRequiredNumber(step, "yaw_deg", i, yaw_deg, error))
      {
        return false;
      }

      bool scan = false;
      if (step.isMember("scan")) {
        if (!step["scan"].isBool()) {
          error = "steps[" + std::to_string(i) + "].scan must be a boolean";
          return false;
        }
        scan = step["scan"].asBool();
      }
      has_scan_step = has_scan_step || scan;

      std::string kind = scan ? "scan" : "transit";
      if (step.isMember("kind")) {
        if (!step["kind"].isString()) {
          error = "steps[" + std::to_string(i) + "].kind must be a string";
          return false;
        }
        kind = step["kind"].asString();
      }

      std::string coord;
      if (step.isMember("coord")) {
        if (!step["coord"].isString()) {
          error = "steps[" + std::to_string(i) + "].coord must be a string";
          return false;
        }
        coord = step["coord"].asString();
      }

      steps.push_back(RouteStep{
        WaypointTarget{x_cm, y_cm, z_cm, yaw_deg},
        scan,
        coord,
        kind,
      });
    }

    if (!has_scan_step) {
      error = "route.steps must contain at least one scan=true step";
      return false;
    }

    return true;
  }

  std::optional<SlotTarget> firstScanSlotFromRoute(const std::vector<RouteStep> & steps) const
  {
    for (const auto & step : steps) {
      if (!step.scan) {
        continue;
      }
      int slot = 0;
      if (step.coord.size() > 1) {
        try {
          slot = std::stoi(step.coord.substr(1));
        } catch (const std::exception &) {
          slot = 0;
        }
      }
      return SlotTarget{
        step.coord,
        step.coord.empty() ? '?' : step.coord.front(),
        slot,
        step.target,
      };
    }
    return std::nullopt;
  }

  void activateGroundRoute(const std::string & reason)
  {
    if (!ground_route_valid_ || ground_route_steps_.empty()) {
      publishDTaskStatus("ground_route_not_ready");
      return;
    }

    active_steps_ = ground_route_steps_;
    publishable_route_ = routeWithLanding(active_steps_);
    target_slot_ = firstScanSlotFromRoute(active_steps_);
    ground_route_loaded_ = true;
    publishRoute(publishable_route_);
    phase_ = MissionPhase::WAIT_STATE;
    RCLCPP_INFO(
      get_logger(),
      "Ground route accepted: steps=%zu target_id=%d.",
      active_steps_.size(),
      target_id_);
    logEvent(
      "ground_route_loaded",
      target_slot_ ? target_slot_->coord : "",
      target_id_,
      reason);
    publishDTaskStatus("ground_route_loaded_" + reason);
  }

  void groundRouteCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!modeSwitchAllowed()) {
      RCLCPP_WARN(get_logger(), "Ignored /d_task/route because mission is locked.");
      publishDTaskStatus("route_ignored_locked");
      return;
    }

    std::vector<RouteStep> parsed_steps;
    std::string error;
    if (!parseGroundRouteJson(msg->data, parsed_steps, error)) {
      ground_route_valid_ = false;
      ground_route_loaded_ = false;
      ground_route_steps_.clear();
      RCLCPP_ERROR(get_logger(), "Rejected /d_task/route: %s.", error.c_str());
      publishDTaskStatus("route_rejected_" + error);
      return;
    }

    ground_route_steps_ = parsed_steps;
    ground_route_valid_ = true;
    ground_route_loaded_ = false;
    RCLCPP_INFO(get_logger(), "Received valid /d_task/route with %zu steps.", ground_route_steps_.size());
    publishDTaskStatus("ground_route_valid");

    if (mission_mode_ == MissionMode::TARGET && target_id_ > 0) {
      activateGroundRoute("route_callback");
    }
  }

  void barcodeCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data <= 0) {
      return;
    }
    last_barcode_value_ = msg->data;
    last_barcode_stamp_ = now();
    if (mission_mode_ == MissionMode::TARGET && target_id_ <= 0) {
      target_id_ = msg->data;
      std_msgs::msg::Int32 target_msg;
      target_msg.data = target_id_;
      target_id_pub_->publish(target_msg);
      qr_id_pub_->publish(target_msg);
      RCLCPP_INFO(
        get_logger(),
        "Target mode accepted preflight QR id: %d. Published to %s.",
        target_id_,
        d_task_qr_id_topic_.c_str());
      logEvent("target_id_detected", "", target_id_, "preflight_ground_station");
      if (phase_ == MissionPhase::WAIT_TARGET_BARCODE) {
        phase_ = MissionPhase::WAIT_GROUND_ROUTE;
      }
      publishDTaskStatus("qr_id_published");
      if (ground_route_valid_) {
        activateGroundRoute("barcode_callback");
      }
    }
  }

  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }
    last_fine_x_px_ = msg->data[0];
    last_fine_y_px_ = msg->data[1];
    last_fine_stamp_ = now();
  }

  bool freshBarcode(int & value) const
  {
    if (last_barcode_value_ <= 0 || last_barcode_stamp_.nanoseconds() == 0) {
      return false;
    }
    if ((now() - last_barcode_stamp_).seconds() > kBarcodeFreshSec) {
      return false;
    }
    value = last_barcode_value_;
    return true;
  }

  bool fineDataCentered() const
  {
    if (last_fine_stamp_.nanoseconds() == 0) {
      return false;
    }
    if ((now() - last_fine_stamp_).seconds() > kFineDataFreshSec) {
      return false;
    }
    return std::abs(last_fine_x_px_) <= kFineDataDeadzonePx &&
           std::abs(last_fine_y_px_) <= kFineDataDeadzonePx;
  }

  void queryCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const auto iter = inventory_results_.find(msg->data);
    std_msgs::msg::String result;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    if (iter == inventory_results_.end()) {
      out << "{\"id\":" << msg->data << ",\"found\":false}";
    } else {
      const auto & record = iter->second;
      out << "{\"id\":" << record.id << ",\"found\":true,"
          << "\"coord\":\"" << jsonEscape(record.coord) << "\","
          << "\"x_cm\":" << record.target.x_cm << ','
          << "\"y_cm\":" << record.target.y_cm << ','
          << "\"z_cm\":" << record.target.z_cm << ','
          << "\"yaw_deg\":" << record.target.yaw_deg << '}';
    }
    result.data = out.str();
    query_result_pub_->publish(result);
  }

  void timerCallback()
  {
    switch (phase_) {
      case MissionPhase::WAIT_MODE:
        processWaitMode();
        break;
      case MissionPhase::WAIT_TARGET_BARCODE:
        processWaitTargetBarcode();
        break;
      case MissionPhase::WAIT_GROUND_ROUTE:
        processWaitGroundRoute();
        break;
      case MissionPhase::WAIT_STATE:
        if (flight_.hasState()) {
          startMission();
        }
        break;
      case MissionPhase::TAKEOFF:
        if (flight_.isReached()) {
          logEvent("takeoff_reached", "", 0, "start_route");
          startNextStep();
        }
        break;
      case MissionPhase::TRANSIT:
      case MissionPhase::GO_SCAN:
        processTravel();
        break;
      case MissionPhase::SCAN:
        processScan();
        break;
      case MissionPhase::LASER_ON:
        processLaser();
        break;
      case MissionPhase::LED_ON:
        processLed();
        break;
      case MissionPhase::RETURN_TRANSIT:
        processReturnTransit();
        break;
      case MissionPhase::DESCEND:
        processDescend();
        break;
      case MissionPhase::COMPLETE:
      case MissionPhase::STOPPED:
        break;
    }
  }

  void processWaitMode()
  {
    if (mission_mode_ == MissionMode::TARGET) {
      phase_ = MissionPhase::WAIT_TARGET_BARCODE;
      publishDTaskStatus("initial_target_mode");
      return;
    }

    const auto now_time = now();
    if (now_time >= mode_select_deadline_) {
      phase_ = MissionPhase::WAIT_STATE;
      RCLCPP_INFO(get_logger(), "No /d_task/mode=1 received; starting default inventory task.");
      logEvent("mode_default_inventory", "", 0, "grace_timeout");
      publishDTaskStatus("default_inventory_grace_timeout");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Waiting %.2fs for optional /d_task/mode=1 before default inventory.",
      (mode_select_deadline_ - now_time).seconds());
    publishDTaskStatusThrottled("waiting_mode_grace");
  }

  void processWaitTargetBarcode()
  {
    if (target_id_ <= 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Target mode waiting for preflight QR code on %s.",
        barcode_topic_.c_str());
      publishDTaskStatusThrottled("waiting_qr_id");
      return;
    }

    phase_ = MissionPhase::WAIT_GROUND_ROUTE;
    publishDTaskStatus("qr_id_ready_waiting_route");
  }

  void processWaitGroundRoute()
  {
    if (target_id_ <= 0) {
      phase_ = MissionPhase::WAIT_TARGET_BARCODE;
      publishDTaskStatus("route_wait_without_qr");
      return;
    }

    if (ground_route_valid_) {
      activateGroundRoute("wait_ground_route");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Target mode waiting for ground station route on %s after QR id %d.",
      ground_route_topic_.c_str(),
      target_id_);
    publishDTaskStatusThrottled("waiting_ground_route");
  }

  std::vector<RouteStep> planDirectTargetRoute(const SlotTarget & target_slot) const
  {
    std::vector<RouteStep> steps;
    appendTransit(homeAtCruise(), target_slot.target, steps);
    steps.push_back(RouteStep{target_slot.target, true, target_slot.coord, "scan"});
    return steps;
  }

  std::vector<RouteStep> routeWithLanding(const std::vector<RouteStep> & steps) const
  {
    std::vector<RouteStep> display_steps = steps;
    WaypointTarget current = homeAtCruise();
    if (!steps.empty()) {
      current = steps.back().target;
    }
    const auto landing_cruise = landingAtCruise(current.yaw_deg);
    const auto landing_final = landingFinal(current.yaw_deg);
    appendTransit(current, landing_cruise, display_steps);
    display_steps.push_back(RouteStep{landing_cruise, false, "", "landing_cruise"});
    display_steps.push_back(RouteStep{landing_final, false, "", "landing_final"});
    return display_steps;
  }

  void stopWithoutTakeoff(const std::string & reason)
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);
    phase_ = MissionPhase::STOPPED;
    RCLCPP_ERROR(get_logger(), "Warehouse target task stopped before takeoff: %s.", reason.c_str());
    logEvent("stopped_before_takeoff", "", target_id_, reason);
  }

  void startMission()
  {
    mission_locked_ = true;
    current_step_index_ = 0;
    return_step_index_ = 0;
    accepted_barcode_ = 0;
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.goTo(homeAtCruise());
    phase_ = MissionPhase::TAKEOFF;
    RCLCPP_INFO(get_logger(), "Warehouse mission takeoff to %.1fcm.", kCruiseHeightCm);
    logEvent("takeoff", "", 0, "start_mission");
    publishDTaskStatus("mission_locked_takeoff");
  }

  void startNextStep()
  {
    aux_.setMagnet(false);
    setVisualTakeover(false);

    if (current_step_index_ >= active_steps_.size()) {
      startReturn();
      return;
    }

    const auto & step = active_steps_[current_step_index_];
    flight_.goTo(step.target);
    phase_ = step.scan ? MissionPhase::GO_SCAN : MissionPhase::TRANSIT;
    RCLCPP_INFO(
      get_logger(),
      "Route step %zu/%zu: %s %s x=%.1f y=%.1f z=%.1f yaw=%.1f.",
      current_step_index_ + 1,
      active_steps_.size(),
      step.kind.c_str(),
      step.coord.c_str(),
      step.target.x_cm,
      step.target.y_cm,
      step.target.z_cm,
      step.target.yaw_deg);
    logEvent("go_to_step", step.coord, 0, step.kind);
  }

  void processTravel()
  {
    if (!flight_.isReached()) {
      return;
    }

    const auto & step = active_steps_[current_step_index_];
    if (!step.scan) {
      ++current_step_index_;
      startNextStep();
      return;
    }

    scan_start_stamp_ = now();
    accepted_barcode_ = 0;
    setVisualTakeover(true);
    phase_ = MissionPhase::SCAN;
    RCLCPP_INFO(get_logger(), "Scan started at %s.", step.coord.c_str());
    logEvent("scan_started", step.coord, 0, "visual_takeover_on");
  }

  void processScan()
  {
    if (current_step_index_ >= active_steps_.size()) {
      startReturn();
      return;
    }

    const auto & step = active_steps_[current_step_index_];
    int value = 0;
    const bool has_barcode = freshBarcode(value);
    const bool value_matches =
      has_barcode &&
      (mission_mode_ == MissionMode::INVENTORY || value == target_id_);
    const bool centered = fineDataCentered();
    const double elapsed = (now() - scan_start_stamp_).seconds();

    if (value_matches && (centered || elapsed >= kVisualLockTimeoutSec)) {
      accepted_barcode_ = value;
      acceptScan(step, value, centered ? "barcode_centered" : "barcode_no_center_lock");
      startLaser();
      return;
    }

    if (elapsed < kScanTimeoutSec) {
      return;
    }

    setVisualTakeover(false);
    if (mission_mode_ == MissionMode::TARGET) {
      RCLCPP_ERROR(
        get_logger(),
        "Target scan failed at %s. expected=%d last=%d.",
        step.coord.c_str(),
        target_id_,
        has_barcode ? value : 0);
      logEvent("target_scan_failed", step.coord, has_barcode ? value : 0, "return_without_laser");
      startReturn();
      return;
    }

    RCLCPP_WARN(get_logger(), "Inventory scan timed out at %s.", step.coord.c_str());
    logEvent("scan_timeout", step.coord, has_barcode ? value : 0, "skip_without_laser");
    ++current_step_index_;
    startNextStep();
  }

  void acceptScan(const RouteStep & step, int value, const std::string & reason)
  {
    setVisualTakeover(false);
    InventoryRecord record{value, step.coord, step.target, now().seconds()};
    inventory_results_[value] = record;
    publishScanResult(record, reason);
    logEvent("scan_accepted", step.coord, value, reason);
  }

  void publishScanResult(const InventoryRecord & record, const std::string & reason)
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"id\":" << record.id << ','
        << "\"coord\":\"" << jsonEscape(record.coord) << "\","
        << "\"x_cm\":" << record.target.x_cm << ','
        << "\"y_cm\":" << record.target.y_cm << ','
        << "\"z_cm\":" << record.target.z_cm << ','
        << "\"yaw_deg\":" << record.target.yaw_deg << ','
        << "\"reason\":\"" << jsonEscape(reason) << "\"}";
    msg.data = out.str();
    scan_result_pub_->publish(msg);
  }

  void startLaser()
  {
    aux_.setMagnet(true);
    action_stamp_ = now();
    phase_ = MissionPhase::LASER_ON;
    const auto coord = current_step_index_ < active_steps_.size() ?
      active_steps_[current_step_index_].coord : "";
    RCLCPP_INFO(get_logger(), "Laser on at %s for barcode=%d.", coord.c_str(), accepted_barcode_);
    logEvent("laser_on", coord, accepted_barcode_, "magnet=1");
  }

  void processLaser()
  {
    if ((now() - action_stamp_).seconds() < kLaserOnSec) {
      return;
    }
    aux_.setMagnet(false);
    aux_.setSignal(true);
    action_stamp_ = now();
    phase_ = MissionPhase::LED_ON;
    const auto coord = current_step_index_ < active_steps_.size() ?
      active_steps_[current_step_index_].coord : "";
    logEvent("laser_off_led_on", coord, accepted_barcode_, "signal=1");
  }

  void processLed()
  {
    if ((now() - action_stamp_).seconds() < kLedOnSec) {
      return;
    }
    aux_.setSignal(false);
    const auto coord = current_step_index_ < active_steps_.size() ?
      active_steps_[current_step_index_].coord : "";
    logEvent("led_off", coord, accepted_barcode_, "signal=0");

    if (mission_mode_ == MissionMode::TARGET) {
      startReturn();
      return;
    }

    ++current_step_index_;
    startNextStep();
  }

  void startReturn()
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    return_steps_.clear();

    WaypointTarget current = homeAtCruise();
    if (const auto active_target = flight_.currentTarget()) {
      current = *active_target;
    }
    const auto landing_cruise = landingAtCruise(current.yaw_deg);
    appendTransit(current, landing_cruise, return_steps_);
    return_steps_.push_back(RouteStep{landing_cruise, false, "", "landing_cruise"});
    return_step_index_ = 0;
    phase_ = MissionPhase::RETURN_TRANSIT;
    RCLCPP_INFO(get_logger(), "Returning to landing point. return_steps=%zu.", return_steps_.size());
    logEvent("return_start", "", accepted_barcode_, "landing_cruise");
    startNextReturnStep();
  }

  void startNextReturnStep()
  {
    if (return_step_index_ >= return_steps_.size()) {
      const double yaw_deg = return_steps_.empty() ? 0.0 : return_steps_.back().target.yaw_deg;
      flight_.goTo(landingFinal(yaw_deg));
      phase_ = MissionPhase::DESCEND;
      logEvent("landing_descend", "", accepted_barcode_, "final");
      return;
    }
    const auto & step = return_steps_[return_step_index_];
    flight_.goTo(step.target);
    RCLCPP_INFO(
      get_logger(),
      "Return step %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f.",
      return_step_index_ + 1,
      return_steps_.size(),
      step.target.x_cm,
      step.target.y_cm,
      step.target.z_cm,
      step.target.yaw_deg);
  }

  void processReturnTransit()
  {
    if (!flight_.isReached()) {
      return;
    }
    ++return_step_index_;
    startNextReturnStep();
  }

  void processDescend()
  {
    if (!flight_.isReached()) {
      return;
    }
    completeTask();
  }

  bool inventoryComplete() const
  {
    if (inventory_results_.size() != static_cast<std::size_t>(kInventoryCount)) {
      return false;
    }
    for (int id = 1; id <= kInventoryCount; ++id) {
      if (inventory_results_.find(id) == inventory_results_.end()) {
        return false;
      }
    }
    return true;
  }

  void completeTask()
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);

    if (mission_mode_ == MissionMode::INVENTORY) {
      const bool success = inventoryComplete();
      const auto json = inventoryResultsJson(success);
      writeInventoryJson(json, success);
      std_msgs::msg::String all_msg;
      all_msg.data = json;
      all_results_pub_->publish(all_msg);
      RCLCPP_INFO(
        get_logger(),
        "Warehouse inventory completed: success=%d records=%zu.",
        success ? 1 : 0,
        inventory_results_.size());
      logEvent("inventory_complete", "", 0, success ? "success" : "incomplete");
    } else {
      RCLCPP_INFO(get_logger(), "Warehouse target task completed.");
      logEvent("target_complete", target_slot_ ? target_slot_->coord : "", accepted_barcode_, "done");
    }

    std_msgs::msg::Empty complete_msg;
    mission_complete_pub_->publish(complete_msg);
    phase_ = MissionPhase::COMPLETE;
    publishDTaskStatus("mission_complete");
  }

  std::string inventoryResultsJson(bool success) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{"
        << "\"task\":\"warehouse_inventory\","
        << "\"complete\":" << (success ? "true" : "false") << ','
        << "\"timestamp_ns\":" << now().nanoseconds() << ','
        << "\"safety_clearance_cm\":" << kScanClearanceCm << ','
        << "\"events_csv\":\"" << jsonEscape(event_csv_path_) << "\","
        << "\"route\":" << routeToJson(publishable_route_) << ','
        << "\"inventory\":{";
    bool first = true;
    for (const auto & item : inventory_results_) {
      if (!first) {
        out << ',';
      }
      first = false;
      const auto & record = item.second;
      out << "\"" << record.id << "\":{"
          << "\"coord\":\"" << jsonEscape(record.coord) << "\","
          << "\"x_cm\":" << record.target.x_cm << ','
          << "\"y_cm\":" << record.target.y_cm << ','
          << "\"z_cm\":" << record.target.z_cm << ','
          << "\"yaw_deg\":" << record.target.yaw_deg << ','
          << "\"time_sec\":" << record.time_sec << "}";
    }
    out << "}}";
    return out.str();
  }

  void writeInventoryJson(const std::string & json, bool success)
  {
    const auto stamp = now().nanoseconds();
    std::ostringstream inventory_path;
    inventory_path << log_dir_ << "/inventory_" << stamp << ".json";
    {
      std::ofstream output(inventory_path.str(), std::ios::out | std::ios::trunc);
      if (!output.is_open()) {
        RCLCPP_WARN(get_logger(), "Failed to write inventory log: %s", inventory_path.str().c_str());
        return;
      }
      output << json << '\n';
    }
    RCLCPP_INFO(get_logger(), "Inventory JSON log: %s", inventory_path.str().c_str());

    if (!success) {
      return;
    }

    const auto latest_path = latestSuccessPath();
    std::ofstream latest(latest_path, std::ios::out | std::ios::trunc);
    if (!latest.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to update latest success inventory: %s", latest_path.c_str());
      return;
    }
    latest << json << '\n';
    RCLCPP_INFO(get_logger(), "Updated latest success inventory: %s", latest_path.c_str());
  }

  std::string latestSuccessPath() const
  {
    return log_dir_ + "/latest_success.json";
  }

  static std::optional<std::string> parseJsonString(
    const std::string & json,
    std::size_t start,
    const std::string & key)
  {
    const auto key_pos = json.find("\"" + key + "\"", start);
    if (key_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto colon = json.find(':', key_pos);
    if (colon == std::string::npos) {
      return std::nullopt;
    }
    const auto quote_start = json.find('"', colon + 1);
    if (quote_start == std::string::npos) {
      return std::nullopt;
    }
    const auto quote_end = json.find('"', quote_start + 1);
    if (quote_end == std::string::npos) {
      return std::nullopt;
    }
    return json.substr(quote_start + 1, quote_end - quote_start - 1);
  }

  static std::optional<double> parseJsonNumber(
    const std::string & json,
    std::size_t start,
    const std::string & key)
  {
    const auto key_pos = json.find("\"" + key + "\"", start);
    if (key_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto colon = json.find(':', key_pos);
    if (colon == std::string::npos) {
      return std::nullopt;
    }
    const auto begin = json.find_first_of("-0123456789", colon + 1);
    if (begin == std::string::npos) {
      return std::nullopt;
    }
    const auto end = json.find_first_not_of("-0123456789.eE+", begin);
    try {
      return std::stod(json.substr(begin, end - begin));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  bool loadLatestSuccessTarget(int target_id, SlotTarget & target_slot) const
  {
    std::ifstream input(latestSuccessPath());
    if (!input.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "No latest success inventory log found: %s",
        latestSuccessPath().c_str());
      return false;
    }
    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string json = buffer.str();
    const std::string needle = "\"" + std::to_string(target_id) + "\"";
    const auto id_pos = json.find(needle);
    if (id_pos == std::string::npos) {
      RCLCPP_ERROR(get_logger(), "Target id %d not found in latest success inventory.", target_id);
      return false;
    }

    const auto coord = parseJsonString(json, id_pos, "coord");
    const auto x = parseJsonNumber(json, id_pos, "x_cm");
    const auto y = parseJsonNumber(json, id_pos, "y_cm");
    const auto z = parseJsonNumber(json, id_pos, "z_cm");
    const auto yaw = parseJsonNumber(json, id_pos, "yaw_deg");
    if (!coord || !x || !y || !z || !yaw || coord->empty()) {
      RCLCPP_ERROR(get_logger(), "Target id %d record is malformed in latest success inventory.", target_id);
      return false;
    }

    target_slot = SlotTarget{
      *coord,
      (*coord)[0],
      coord->size() > 1 ? std::stoi(coord->substr(1)) : 0,
      WaypointTarget{*x, *y, *z, *yaw},
    };
    return true;
  }

  WaypointNavigator flight_;
  AuxController aux_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scan_result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr all_results_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qr_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr d_task_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr barcode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr query_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ground_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ground_route_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  MissionMode mission_mode_{MissionMode::INVENTORY};
  MissionPhase phase_{MissionPhase::WAIT_MODE};
  double timer_period_sec_{0.05};
  double mode_select_grace_sec_{0.5};
  std::string barcode_topic_;
  std::string fine_data_topic_;
  std::string ground_mode_topic_;
  std::string ground_route_topic_;
  std::string d_task_qr_id_topic_;
  std::string d_task_status_topic_;
  std::string log_dir_;
  std::string event_csv_path_;
  std::ofstream event_csv_;

  std::vector<RackSpec> racks_;
  std::vector<FaceSpec> faces_;
  std::vector<SafetyRect> safety_rects_;
  std::unordered_map<char, std::vector<SlotTarget>> slots_by_face_;
  std::vector<RouteStep> active_steps_;
  std::vector<RouteStep> publishable_route_;
  std::vector<RouteStep> ground_route_steps_;
  std::vector<RouteStep> return_steps_;
  std::size_t current_step_index_{0};
  std::size_t return_step_index_{0};

  std::map<int, InventoryRecord> inventory_results_;
  std::optional<SlotTarget> target_slot_;
  int target_id_{0};
  int accepted_barcode_{0};
  int last_barcode_value_{0};
  rclcpp::Time last_barcode_stamp_;
  int last_fine_x_px_{0};
  int last_fine_y_px_{0};
  rclcpp::Time last_fine_stamp_;
  rclcpp::Time scan_start_stamp_;
  rclcpp::Time action_stamp_;
  rclcpp::Time mode_select_deadline_;
  rclcpp::Time last_status_pub_stamp_;
  bool visual_takeover_enabled_{false};
  bool mission_locked_{false};
  bool ground_route_valid_{false};
  bool ground_route_loaded_{false};
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::WarehouseInventoryTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
