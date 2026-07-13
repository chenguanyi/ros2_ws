#include "activity_control_pkg/core/aux_controller.hpp"
#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
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
#include <set>
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

#include <fcntl.h>
#include <unistd.h>

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
constexpr double kTargetModeQrDecisionWindowSec = 1.0;
constexpr int kFineDataDeadzonePx = 28;
constexpr double kRouteXySpeedCmS = 36.0;
constexpr double kRouteZSpeedCmS = 30.0;
constexpr double kRouteYawSpeedDegS = 25.0;
constexpr double kTurnYawThresholdDeg = 2.0;
constexpr int kInventoryCount = 24;
constexpr double kFixedBypassLeftXCm = -35.0;
constexpr double kFixedBypassRightXCm = 285.0;
constexpr double kFixedBypassAYCm = 0.0;
constexpr double kFixedBypassBYCm = -135.0;
constexpr double kFixedBypassCYCm = -215.0;
constexpr double kFixedBypassDYCm = -335.0;
constexpr double kTargetCoordMatchXyToleranceCm = 20.0;
constexpr double kTargetCoordMatchZToleranceCm = 15.0;
constexpr double kTargetCoordMatchYawToleranceDeg = 45.0;

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

struct GroundTargetRequest
{
  SlotTarget slot;
  int id{0};
};

struct FixedTargetPlan
{
  std::vector<RouteStep> outbound_steps;
  std::vector<RouteStep> return_steps;
  std::vector<RouteStep> display_steps;
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
    barcode_topic_ =
      declare_parameter("barcode_topic", std::string("/warehouse_inventory/barcode_value"));
    fine_data_topic_ = declare_parameter("fine_data_topic", std::string("/fine_data"));
    ground_mode_topic_ = declare_parameter("ground_mode_topic", std::string("/d_task/mode"));
    ground_route_topic_ = declare_parameter("ground_route_topic", std::string("/d_task/route"));
    planned_route_topic_ =
      declare_parameter("planned_route_topic", std::string("/d_task/planned_route"));
    st_ready_topic_ = declare_parameter("st_ready_topic", std::string("/is_st_ready"));
    d_task_qr_id_topic_ = declare_parameter("d_task_qr_id_topic", std::string("/d_task/qr_id"));
    d_task_status_topic_ =
      declare_parameter("d_task_status_topic", std::string("/d_task/status"));

    if (timer_period_sec_ <= 0.0) {
      throw std::runtime_error("timer_period_sec must be > 0.");
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
    planned_route_pub_ =
      create_publisher<std_msgs::msg::String>(planned_route_topic_, durable_qos);
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
    st_ready_sub_ = create_subscription<std_msgs::msg::UInt8>(
      st_ready_topic_,
      durable_qos,
      std::bind(&WarehouseInventoryTaskNode::stReadyCallback, this, std::placeholders::_1));

    if (mission_mode_ == MissionMode::TARGET) {
      active_steps_.clear();
      publishable_route_.clear();
    } else {
      publishRoute(publishable_route_);
    }

    phase_ = mission_mode_ == MissionMode::TARGET ?
      MissionPhase::WAIT_TARGET_BARCODE : MissionPhase::WAIT_MODE;
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
      "mode_topic=%s route_topic=%s planned_route_topic=%s st_ready_topic=%s.",
      mission_mode_ == MissionMode::TARGET ? "target" : "inventory",
      active_steps_.size(),
      kScanClearanceCm,
      ground_mode_topic_.c_str(),
      ground_route_topic_.c_str(),
      planned_route_topic_.c_str(),
      st_ready_topic_.c_str());
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

  void publishPlannedRoute(const std::vector<RouteStep> & steps)
  {
    std_msgs::msg::String msg;
    msg.data = routeToJson(steps);
    route_pub_->publish(msg);
    planned_route_pub_->publish(msg);
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
        << "\"st_ready\":" << (st_ready_received_ ? "true" : "false") << ','
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
           !st_ready_received_ &&
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
    ground_return_steps_.clear();
    target_mode_qr_window_active_ = false;
    target_mode_qr_window_checks_ = 0;
    target_qr_laser_active_ = false;
    inventory_results_.clear();
    inventory_by_coord_.clear();
    missed_coords_.clear();
    final_inventory_written_ = false;
    final_inventory_path_.clear();
    active_steps_ = planBestInventoryRoute();
    publishable_route_ = routeWithLanding(active_steps_);
    publishRoute(publishable_route_);
    phase_ = st_ready_received_ ? MissionPhase::WAIT_STATE : MissionPhase::WAIT_MODE;
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    RCLCPP_INFO(get_logger(), "D task mode set to inventory by %s.", reason.c_str());
    logEvent("mode_inventory", "", 0, reason);
    publishDTaskStatus("mode_inventory_" + reason);
  }

  void startTargetQrWindow(const std::string & reason)
  {
    last_barcode_value_ = 0;
    last_barcode_stamp_ = rclcpp::Time();
    last_fine_stamp_ = rclcpp::Time();
    target_mode_qr_window_start_stamp_ = now();
    target_mode_qr_window_active_ = true;
    target_mode_qr_window_checks_ = 0;
    setVisualTakeover(true);
    RCLCPP_INFO(
      get_logger(),
      "Target mode QR recognition window started by %s. Checking for %.1fs after mode switch.",
      reason.c_str(),
      kTargetModeQrDecisionWindowSec);
    logEvent("target_qr_window_started", "", 0, reason);
    publishDTaskStatus("target_qr_window_started_" + reason);
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
    ground_return_steps_.clear();
    active_steps_.clear();
    publishable_route_.clear();
    target_qr_laser_active_ = false;
    phase_ = MissionPhase::WAIT_TARGET_BARCODE;
    aux_.setAll(0, 0, 0);
    flight_.publishActiveController(3);
    startTargetQrWindow(reason);
    RCLCPP_INFO(get_logger(), "D task mode set to ground target by %s.", reason.c_str());
    logEvent("mode_ground_target", "", 0, reason);
    publishDTaskStatus("mode_ground_target_" + reason);
  }

  void groundModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!modeSwitchAllowed()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignored /d_task/mode=%u because mission is locked, ST ready was received, or phase is not selectable.",
        static_cast<unsigned>(msg->data));
      publishDTaskStatus(st_ready_received_ ? "mode_ignored_st_ready" : "mode_ignored_locked");
      return;
    }

    if (msg->data == 0U) {
      if (mission_mode_ == MissionMode::INVENTORY) {
        publishDTaskStatus("mode_inventory_already_selected");
        return;
      }
      selectInventoryMode("ground_mode_0");
      return;
    }

    if (msg->data == 1U) {
      if (mission_mode_ == MissionMode::TARGET) {
        if (target_id_ <= 0 && !target_mode_qr_window_active_) {
          startTargetQrWindow("ground_mode_1_repeat");
        } else {
          publishDTaskStatus("mode_target_already_selected");
        }
        return;
      }
      selectGroundTargetMode("ground_mode_1");
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Ignored invalid /d_task/mode=%u. Expected 0 or 1.",
      static_cast<unsigned>(msg->data));
    publishDTaskStatus("invalid_mode_value");
  }

  void startTargetQrLaserPulse(int target_id, const std::string & reason)
  {
    target_qr_laser_active_ = true;
    target_qr_laser_stamp_ = now();
    aux_.setMagnet(true);
    RCLCPP_INFO(
      get_logger(),
      "Initial target QR accepted: id=%d. Laser confirmation pulse on for %.1fs.",
      target_id,
      kLaserOnSec);
    logEvent(
      "target_qr_laser_on",
      target_slot_ ? target_slot_->coord : "",
      target_id,
      reason);
    publishDTaskStatus("target_qr_laser_on_" + reason);
  }

  void processTargetQrLaserPulse()
  {
    if (!target_qr_laser_active_) {
      return;
    }

    if ((now() - target_qr_laser_stamp_).seconds() < kLaserOnSec) {
      return;
    }

    target_qr_laser_active_ = false;
    aux_.setMagnet(false);
    RCLCPP_INFO(get_logger(), "Initial target QR laser confirmation pulse off.");
    logEvent(
      "target_qr_laser_off",
      target_slot_ ? target_slot_->coord : "",
      target_id_,
      "magnet=0");
    publishDTaskStatus("target_qr_laser_off");
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

  bool readOptionalTargetId(
    const Json::Value & value,
    const std::string & key,
    int & id,
    std::string & error) const
  {
    if (!value.isMember(key)) {
      return true;
    }
    if (!value[key].isNumeric()) {
      error = key + " must be a positive integer";
      return false;
    }

    const double raw = value[key].asDouble();
    const double rounded = std::round(raw);
    if (!std::isfinite(raw) || rounded <= 0.0 || std::fabs(raw - rounded) > 1e-6) {
      error = key + " must be a positive integer";
      return false;
    }

    id = static_cast<int>(rounded);
    return true;
  }

  std::string normalizedCoord(const std::string & raw_coord) const
  {
    std::string coord;
    coord.reserve(raw_coord.size());
    for (const char ch : raw_coord) {
      if (!std::isspace(static_cast<unsigned char>(ch))) {
        coord.push_back(ch);
      }
    }
    if (!coord.empty()) {
      coord[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(coord[0])));
    }
    return coord;
  }

  std::optional<SlotTarget> slotByCoord(const std::string & raw_coord) const
  {
    const auto coord = normalizedCoord(raw_coord);
    if (coord.size() < 2) {
      return std::nullopt;
    }

    const char face = coord.front();
    int slot = 0;
    try {
      slot = std::stoi(coord.substr(1));
    } catch (const std::exception &) {
      return std::nullopt;
    }

    const auto iter = slots_by_face_.find(face);
    if (iter == slots_by_face_.end()) {
      return std::nullopt;
    }

    const auto found = std::find_if(
      iter->second.begin(),
      iter->second.end(),
      [slot](const SlotTarget & target) {
        return target.slot == slot;
      });
    if (found == iter->second.end()) {
      return std::nullopt;
    }
    return *found;
  }

  std::optional<SlotTarget> closestSlotToTarget(const WaypointTarget & requested) const
  {
    std::optional<SlotTarget> best_slot;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & item : slots_by_face_) {
      for (const auto & slot : item.second) {
        const double dxy = std::hypot(
          slot.target.x_cm - requested.x_cm,
          slot.target.y_cm - requested.y_cm);
        const double dz = std::fabs(slot.target.z_cm - requested.z_cm);
        const double dyaw = std::fabs(normalizeAngleDeg(slot.target.yaw_deg - requested.yaw_deg));
        if (dxy > kTargetCoordMatchXyToleranceCm ||
          dz > kTargetCoordMatchZToleranceCm ||
          dyaw > kTargetCoordMatchYawToleranceDeg)
        {
          continue;
        }

        const double score = dxy + dz + dyaw * 0.1;
        if (score < best_score) {
          best_score = score;
          best_slot = slot;
        }
      }
    }
    return best_slot;
  }

  bool resolveFixedTargetSlot(
    const std::string & coord,
    const WaypointTarget & requested,
    SlotTarget & slot,
    std::string & error) const
  {
    if (!coord.empty()) {
      const auto fixed_slot = slotByCoord(coord);
      if (!fixed_slot) {
        error = "unknown fixed warehouse coord: " + coord;
        return false;
      }
      slot = *fixed_slot;
      return true;
    }

    const auto fixed_slot = closestSlotToTarget(requested);
    if (!fixed_slot) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(1)
          << "target point does not match any fixed label within tolerance: x="
          << requested.x_cm << " y=" << requested.y_cm << " z=" << requested.z_cm
          << " yaw=" << requested.yaw_deg;
      error = out.str();
      return false;
    }
    slot = *fixed_slot;
    return true;
  }

  bool parseGroundTargetJson(
    const std::string & text,
    GroundTargetRequest & request,
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

    int root_id = 0;
    if (!readOptionalTargetId(root, "id", root_id, error) ||
      !readOptionalTargetId(root, "target_id", root_id, error))
    {
      return false;
    }

    for (Json::ArrayIndex i = 0; i < json_steps.size(); ++i) {
      const Json::Value & step = json_steps[i];
      if (!step.isObject()) {
        error = "steps[" + std::to_string(i) + "] must be an object";
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
      if (!scan) {
        continue;
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

      std::string coord;
      if (step.isMember("coord")) {
        if (!step["coord"].isString()) {
          error = "steps[" + std::to_string(i) + "].coord must be a string";
          return false;
        }
        coord = step["coord"].asString();
      }
      if (coord.empty() && root.isMember("coord")) {
        if (!root["coord"].isString()) {
          error = "coord must be a string";
          return false;
        }
        coord = root["coord"].asString();
      }

      int id = root_id;
      if (!readOptionalTargetId(step, "id", id, error) ||
        !readOptionalTargetId(step, "target_id", id, error))
      {
        return false;
      }

      SlotTarget slot;
      if (!resolveFixedTargetSlot(coord, WaypointTarget{x_cm, y_cm, z_cm, yaw_deg}, slot, error)) {
        return false;
      }
      request = GroundTargetRequest{slot, id};
      return true;
    }

    error = "route.steps must contain at least one scan=true target step";
    return false;
  }

  WaypointTarget fixedLeftBypass(char face, double yaw_deg) const
  {
    return WaypointTarget{kFixedBypassLeftXCm, fixedBypassY(face), kCruiseHeightCm, yaw_deg};
  }

  WaypointTarget fixedRightBypass(char face, double yaw_deg) const
  {
    return WaypointTarget{kFixedBypassRightXCm, fixedBypassY(face), kCruiseHeightCm, yaw_deg};
  }

  double fixedBypassY(char face) const
  {
    switch (face) {
      case 'A':
        return kFixedBypassAYCm;
      case 'B':
        return kFixedBypassBYCm;
      case 'C':
        return kFixedBypassCYCm;
      case 'D':
        return kFixedBypassDYCm;
      default:
        return kFixedBypassAYCm;
    }
  }

  RouteStep fixedTransitStep(const WaypointTarget & target, const std::string & kind) const
  {
    return RouteStep{target, false, "", kind};
  }

  FixedTargetPlan buildFixedTargetPlan(const SlotTarget & slot) const
  {
    FixedTargetPlan plan;
    const double scan_yaw = slot.target.yaw_deg;
    const double return_yaw = 0.0;

    auto add_outbound = [&plan, this](const WaypointTarget & target, const std::string & kind) {
        plan.outbound_steps.push_back(fixedTransitStep(target, kind));
      };
    auto add_return = [&plan, this](const WaypointTarget & target, const std::string & kind) {
        plan.return_steps.push_back(fixedTransitStep(target, kind));
      };

    switch (slot.face) {
      case 'A':
        break;
      case 'B':
        add_outbound(fixedLeftBypass('A', scan_yaw), "left_bypass_A");
        add_outbound(fixedLeftBypass('B', scan_yaw), "left_bypass_B");
        break;
      case 'C':
        add_outbound(fixedLeftBypass('A', scan_yaw), "left_bypass_A");
        add_outbound(fixedLeftBypass('B', scan_yaw), "left_bypass_B");
        add_outbound(fixedLeftBypass('C', scan_yaw), "left_bypass_C");
        break;
      case 'D':
        add_outbound(fixedLeftBypass('A', scan_yaw), "left_bypass_A");
        add_outbound(fixedLeftBypass('B', scan_yaw), "left_bypass_B");
        add_outbound(fixedLeftBypass('C', scan_yaw), "left_bypass_C");
        add_outbound(fixedLeftBypass('D', scan_yaw), "left_bypass_D");
        break;
      default:
        throw std::runtime_error("Unsupported fixed target face.");
    }

    plan.outbound_steps.push_back(RouteStep{slot.target, true, slot.coord, "scan"});

    switch (slot.face) {
      case 'A':
        add_return(fixedRightBypass('A', return_yaw), "right_bypass_A");
        add_return(fixedRightBypass('D', return_yaw), "right_bypass_D");
        break;
      case 'B':
        add_return(fixedRightBypass('B', return_yaw), "right_bypass_B");
        add_return(fixedRightBypass('D', return_yaw), "right_bypass_D");
        break;
      case 'C':
        add_return(fixedRightBypass('C', return_yaw), "right_bypass_C");
        add_return(fixedRightBypass('D', return_yaw), "right_bypass_D");
        break;
      case 'D':
        break;
      default:
        throw std::runtime_error("Unsupported fixed target face.");
    }
    add_return(landingAtCruise(return_yaw), "landing_cruise");

    plan.display_steps.push_back(fixedTransitStep(homeAtCruise(), "takeoff_cruise"));
    plan.display_steps.insert(
      plan.display_steps.end(),
      plan.outbound_steps.begin(),
      plan.outbound_steps.end());
    plan.display_steps.insert(
      plan.display_steps.end(),
      plan.return_steps.begin(),
      plan.return_steps.end());
    plan.display_steps.push_back(fixedTransitStep(landingFinal(return_yaw), "landing_final"));
    return plan;
  }

  void publishTargetId(int target_id)
  {
    std_msgs::msg::Int32 target_msg;
    target_msg.data = target_id;
    target_id_pub_->publish(target_msg);
    qr_id_pub_->publish(target_msg);
  }

  void activateFixedTargetFromSlot(const SlotTarget & slot, int target_id, const std::string & reason)
  {
    const auto plan = buildFixedTargetPlan(slot);
    target_id_ = target_id;
    target_slot_ = slot;
    ground_route_steps_ = plan.outbound_steps;
    ground_return_steps_ = plan.return_steps;
    publishable_route_ = plan.display_steps;
    active_steps_ = ground_route_steps_;
    ground_route_valid_ = true;
    ground_route_loaded_ = true;
    publishTargetId(target_id_);
    publishPlannedRoute(publishable_route_);
    phase_ = MissionPhase::WAIT_STATE;
    target_mode_qr_window_active_ = false;
    setVisualTakeover(false);

    RCLCPP_INFO(
      get_logger(),
      "Fixed target route planned from latest inventory: id=%d coord=%s outbound_steps=%zu return_steps=%zu.",
      target_id_,
      target_slot_->coord.c_str(),
      active_steps_.size(),
      ground_return_steps_.size());
    logEvent("fixed_target_route_planned", target_slot_->coord, target_id_, reason);
    publishDTaskStatus("fixed_target_route_planned_" + reason);
    startTargetQrLaserPulse(target_id_, reason);
  }

  void stReadyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data == 0U || st_ready_received_) {
      return;
    }

    st_ready_received_ = true;
    RCLCPP_INFO(get_logger(), "Received ST ready on %s. Mode selection is now locked.", st_ready_topic_.c_str());
    logEvent(
      "st_ready_received",
      target_slot_ ? target_slot_->coord : "",
      target_id_,
      mission_mode_ == MissionMode::TARGET ? "target_mode_locked" : "inventory_mode_locked");

    if (mission_mode_ == MissionMode::INVENTORY && phase_ == MissionPhase::WAIT_MODE) {
      phase_ = MissionPhase::WAIT_STATE;
    }

    publishDTaskStatus("st_ready_received");
  }

  void groundRouteCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    (void)msg;
    RCLCPP_WARN(
      get_logger(),
      "Ignored %s. Target mode now uses latest_success.json plus camera QR id.",
      ground_route_topic_.c_str());
    publishDTaskStatus("route_ignored_target_uses_latest_success");
  }

  void barcodeCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data <= 0) {
      return;
    }
    last_barcode_value_ = msg->data;
    last_barcode_stamp_ = now();
    if (mission_mode_ == MissionMode::TARGET && target_id_ <= 0) {
      tryActivateTargetFromBarcode(msg->data, "barcode_latest_success");
    }
  }

  bool tryActivateTargetFromBarcode(int barcode_value, const std::string & reason)
  {
    if (barcode_value <= 0 || mission_mode_ != MissionMode::TARGET || target_id_ > 0) {
      return false;
    }

    SlotTarget target_slot;
    std::string error;
    if (!loadLatestSuccessTarget(barcode_value, target_slot, error)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Target mode rejected QR id %d from latest inventory: %s.",
        barcode_value,
        error.c_str());
      if (last_target_lookup_failure_id_ != barcode_value ||
        last_target_lookup_failure_error_ != error)
      {
        last_target_lookup_failure_id_ = barcode_value;
        last_target_lookup_failure_error_ = error;
        logEvent("target_lookup_failed", "", barcode_value, error);
        publishDTaskStatus("target_lookup_failed_" + error);
      }
      return false;
    }

    last_target_lookup_failure_id_ = 0;
    last_target_lookup_failure_error_.clear();
    RCLCPP_INFO(
      get_logger(),
      "Target mode accepted QR id %d from latest inventory as coord %s.",
      barcode_value,
      target_slot.coord.c_str());
    activateFixedTargetFromSlot(target_slot, barcode_value, reason);
    return true;
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
    processTargetQrLaserPulse();

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
        if (readyToStartMission()) {
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

    if (st_ready_received_) {
      phase_ = MissionPhase::WAIT_STATE;
      RCLCPP_INFO(get_logger(), "ST ready received; starting default inventory when flight state is available.");
      logEvent("mode_default_inventory", "", 0, "st_ready");
      publishDTaskStatus("default_inventory_st_ready");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Waiting for /is_st_ready. /d_task/mode can still select requirement 1 or 2.");
    publishDTaskStatusThrottled("waiting_st_ready_or_mode");
  }

  bool readyToStartMission()
  {
    if (!st_ready_received_) {
      publishDTaskStatusThrottled("waiting_st_ready");
      return false;
    }
    if (!flight_.hasState()) {
      publishDTaskStatusThrottled("waiting_flight_state");
      return false;
    }
    if (active_steps_.empty()) {
      publishDTaskStatusThrottled("waiting_active_route");
      return false;
    }
    if (mission_mode_ == MissionMode::TARGET && !ground_route_loaded_) {
      publishDTaskStatusThrottled("waiting_fixed_target_route_loaded");
      return false;
    }
    if (target_qr_laser_active_) {
      publishDTaskStatusThrottled("waiting_target_qr_laser_off", 0.2);
      return false;
    }
    return true;
  }

  void processWaitTargetBarcode()
  {
    if (target_id_ <= 0) {
      if (!std::filesystem::exists(latestSuccessPath())) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Target mode waiting for latest success inventory log: %s.",
          latestSuccessPath().c_str());
        publishDTaskStatusThrottled("waiting_latest_success_inventory");
        return;
      }

      int barcode_value = 0;
      if (freshBarcode(barcode_value) &&
        tryActivateTargetFromBarcode(
          barcode_value,
          target_mode_qr_window_active_ ? "target_mode_window_qr" : "barcode_latest_success"))
      {
        return;
      }

      if (target_mode_qr_window_active_) {
        ++target_mode_qr_window_checks_;
        const double elapsed = (now() - target_mode_qr_window_start_stamp_).seconds();
        if (elapsed >= kTargetModeQrDecisionWindowSec) {
          target_mode_qr_window_active_ = false;
          setVisualTakeover(false);
          RCLCPP_WARN(
            get_logger(),
            "Target mode QR recognition window ended after %.2fs with %d checks and no accepted QR.",
            elapsed,
            target_mode_qr_window_checks_);
          logEvent(
            "target_qr_window_timeout",
            "",
            barcode_value,
            "checks=" + std::to_string(target_mode_qr_window_checks_));
          publishDTaskStatus("target_qr_window_timeout");
        } else {
          publishDTaskStatusThrottled("target_qr_window_checking", 0.2);
        }
        return;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Target mode waiting for camera QR id on %s to look up latest inventory.",
        barcode_topic_.c_str());
      publishDTaskStatusThrottled("waiting_qr_id");
      return;
    }

    if (ground_route_loaded_) {
      phase_ = MissionPhase::WAIT_STATE;
      publishDTaskStatus("qr_id_route_ready");
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Target id %d is set but no fixed route is loaded; waiting for another QR lookup.",
      target_id_);
    publishDTaskStatusThrottled("qr_id_without_route");
  }

  void processWaitGroundRoute()
  {
    phase_ = mission_mode_ == MissionMode::TARGET ?
      MissionPhase::WAIT_TARGET_BARCODE : MissionPhase::WAIT_MODE;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Unexpected WAIT_GROUND_ROUTE phase; target mode uses latest_success.json plus camera QR id.");
    publishDTaskStatusThrottled("unexpected_wait_ground_route");
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
    inventory_results_.clear();
    inventory_by_coord_.clear();
    missed_coords_.clear();
    final_inventory_written_ = false;
    final_inventory_path_.clear();
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
    recordScanMiss(step, has_barcode ? value : 0, "scan_timeout");
    if (!hasFutureScanStep(current_step_index_ + 1U)) {
      writeFinalInventorySnapshot("last_scan_timeout");
    }
    ++current_step_index_;
    startNextStep();
  }

  void acceptScan(const RouteStep & step, int value, const std::string & reason)
  {
    setVisualTakeover(false);
    InventoryRecord record{value, step.coord, step.target, now().seconds()};
    inventory_by_coord_[step.coord] = record;
    inventory_results_[value] = record;
    missed_coords_.erase(step.coord);
    publishScanResult(record, reason);
    logEvent("scan_accepted", step.coord, value, reason);
    writeProgressInventorySnapshot("scan_accepted");
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

  bool hasFutureScanStep(std::size_t start_index) const
  {
    for (std::size_t i = start_index; i < active_steps_.size(); ++i) {
      if (active_steps_[i].scan) {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> expectedInventoryCoords() const
  {
    std::vector<std::string> coords;
    std::set<std::string> seen;
    for (const auto & step : active_steps_) {
      if (!step.scan || step.coord.empty()) {
        continue;
      }
      if (seen.insert(step.coord).second) {
        coords.push_back(step.coord);
      }
    }
    return coords;
  }

  std::optional<WaypointTarget> targetForCoord(const std::string & coord) const
  {
    for (const auto & step : active_steps_) {
      if (step.scan && step.coord == coord) {
        return step.target;
      }
    }
    const auto slot = slotByCoord(coord);
    if (slot) {
      return slot->target;
    }
    return std::nullopt;
  }

  std::map<int, std::vector<std::string>> barcodeCoordsById() const
  {
    std::map<int, std::vector<std::string>> result;
    for (const auto & item : inventory_by_coord_) {
      result[item.second.id].push_back(item.first);
    }
    return result;
  }

  bool inventoryComplete() const
  {
    const auto expected = expectedInventoryCoords();
    if (expected.size() != static_cast<std::size_t>(kInventoryCount)) {
      return false;
    }
    return std::all_of(expected.begin(), expected.end(), [this](const std::string & coord) {
        return inventory_by_coord_.find(coord) != inventory_by_coord_.end();
      });
  }

  std::vector<std::string> missingInventoryCoords() const
  {
    std::vector<std::string> missing;
    for (const auto & coord : expectedInventoryCoords()) {
      if (inventory_by_coord_.find(coord) == inventory_by_coord_.end()) {
        missing.push_back(coord);
      }
    }
    return missing;
  }

  void recordScanMiss(const RouteStep & step, int observed_barcode, const std::string & reason)
  {
    missed_coords_[step.coord] = now().seconds();
    RCLCPP_WARN(
      get_logger(),
      "Recorded missing inventory scan at %s. observed_barcode=%d reason=%s.",
      step.coord.c_str(),
      observed_barcode,
      reason.c_str());
    writeProgressInventorySnapshot(reason);
  }

  std::string inventoryResultsJson(bool finalized) const
  {
    const bool complete = inventoryComplete();
    const auto missing = missingInventoryCoords();
    const auto duplicates = barcodeCoordsById();

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{"
        << "\"task\":\"warehouse_inventory\","
        << "\"complete\":" << (complete ? "true" : "false") << ','
        << "\"finalized\":" << (finalized ? "true" : "false") << ','
        << "\"timestamp_ns\":" << now().nanoseconds() << ','
        << "\"safety_clearance_cm\":" << kScanClearanceCm << ','
        << "\"events_csv\":\"" << jsonEscape(event_csv_path_) << "\","
        << "\"route\":" << routeToJson(publishable_route_) << ',';

    out << "\"missing_coords\":[";
    for (std::size_t i = 0; i < missing.size(); ++i) {
      if (i > 0) {
        out << ',';
      }
      out << "\"" << jsonEscape(missing[i]) << "\"";
    }
    out << "],";

    out << "\"duplicate_ids\":{";
    bool first_duplicate = true;
    for (const auto & item : duplicates) {
      if (item.second.size() <= 1U) {
        continue;
      }
      if (!first_duplicate) {
        out << ',';
      }
      first_duplicate = false;
      out << "\"" << item.first << "\":[";
      for (std::size_t i = 0; i < item.second.size(); ++i) {
        if (i > 0) {
          out << ',';
        }
        out << "\"" << jsonEscape(item.second[i]) << "\"";
      }
      out << "]";
    }
    out << "},";

    out << "\"inventory_by_coord\":{";
    bool first_coord = true;
    for (const auto & coord : expectedInventoryCoords()) {
      if (!first_coord) {
        out << ',';
      }
      first_coord = false;
      out << "\"" << jsonEscape(coord) << "\":{";
      const auto record_iter = inventory_by_coord_.find(coord);
      if (record_iter == inventory_by_coord_.end()) {
        out << "\"found\":false";
        const auto target = targetForCoord(coord);
        if (target) {
          out << ",\"x_cm\":" << target->x_cm
              << ",\"y_cm\":" << target->y_cm
              << ",\"z_cm\":" << target->z_cm
              << ",\"yaw_deg\":" << target->yaw_deg;
        }
        const auto miss_iter = missed_coords_.find(coord);
        if (miss_iter != missed_coords_.end()) {
          out << ",\"missed\":true,\"time_sec\":" << miss_iter->second;
        } else {
          out << ",\"missed\":false";
        }
      } else {
        const auto & record = record_iter->second;
        out << "\"found\":true,"
            << "\"id\":" << record.id << ','
            << "\"x_cm\":" << record.target.x_cm << ','
            << "\"y_cm\":" << record.target.y_cm << ','
            << "\"z_cm\":" << record.target.z_cm << ','
            << "\"yaw_deg\":" << record.target.yaw_deg << ','
            << "\"time_sec\":" << record.time_sec;
      }
      out << "}";
    }
    out << "},";

    out << "\"inventory\":{";
    bool first_id = true;
    for (const auto & item : inventory_results_) {
      if (!first_id) {
        out << ',';
      }
      first_id = false;
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

  bool atomicWriteTextFile(const std::string & path, const std::string & text)
  {
    const std::string tmp_path = path + ".tmp";
    const int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open temp file %s: %s.",
        tmp_path.c_str(),
        std::strerror(errno));
      return false;
    }

    std::string payload = text;
    payload.push_back('\n');
    const char * data = payload.data();
    std::size_t remaining = payload.size();
    while (remaining > 0U) {
      const ssize_t written = ::write(fd, data, remaining);
      if (written < 0) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to write temp file %s: %s.",
          tmp_path.c_str(),
          std::strerror(errno));
        ::close(fd);
        std::filesystem::remove(tmp_path);
        return false;
      }
      if (written == 0) {
        RCLCPP_WARN(get_logger(), "Failed to write temp file %s: wrote 0 bytes.", tmp_path.c_str());
        ::close(fd);
        std::filesystem::remove(tmp_path);
        return false;
      }
      data += written;
      remaining -= static_cast<std::size_t>(written);
    }

    if (::fsync(fd) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to fsync temp file %s: %s.",
        tmp_path.c_str(),
        std::strerror(errno));
      ::close(fd);
      std::filesystem::remove(tmp_path);
      return false;
    }

    if (::close(fd) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to close temp file %s: %s.",
        tmp_path.c_str(),
        std::strerror(errno));
      std::filesystem::remove(tmp_path);
      return false;
    }

    if (::rename(tmp_path.c_str(), path.c_str()) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to rename %s to %s: %s.",
        tmp_path.c_str(),
        path.c_str(),
        std::strerror(errno));
      std::filesystem::remove(tmp_path);
      return false;
    }

    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
      const int dir_fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
      if (dir_fd >= 0) {
        (void)::fsync(dir_fd);
        (void)::close(dir_fd);
      }
    }
    return true;
  }

  std::string latestProgressPath() const
  {
    return log_dir_ + "/latest_progress.json";
  }

  void writeProgressInventorySnapshot(const std::string & reason)
  {
    if (mission_mode_ != MissionMode::INVENTORY) {
      return;
    }
    const auto json = inventoryResultsJson(false);
    if (atomicWriteTextFile(latestProgressPath(), json)) {
      RCLCPP_DEBUG(
        get_logger(),
        "Updated latest inventory progress after %s: %s",
        reason.c_str(),
        latestProgressPath().c_str());
    }
  }

  void writeFinalInventorySnapshot(const std::string & reason)
  {
    if (mission_mode_ != MissionMode::INVENTORY) {
      return;
    }

    const auto json = inventoryResultsJson(true);
    if (!final_inventory_written_) {
      const auto stamp = now().nanoseconds();
      std::ostringstream inventory_path;
      inventory_path << log_dir_ << "/inventory_" << stamp << ".json";
      if (atomicWriteTextFile(inventory_path.str(), json)) {
        final_inventory_path_ = inventory_path.str();
        final_inventory_written_ = true;
        RCLCPP_INFO(get_logger(), "Inventory JSON log: %s", final_inventory_path_.c_str());
      }
    }

    if (atomicWriteTextFile(latestProgressPath(), json)) {
      RCLCPP_DEBUG(get_logger(), "Updated final latest_progress.json.");
    }
    if (atomicWriteTextFile(latestSuccessPath(), json)) {
      RCLCPP_INFO(
        get_logger(),
        "Updated latest inventory snapshot after %s: %s complete=%d missing=%zu.",
        reason.c_str(),
        latestSuccessPath().c_str(),
        inventoryComplete() ? 1 : 0,
        missingInventoryCoords().size());
    }

    std_msgs::msg::String all_msg;
    all_msg.data = json;
    all_results_pub_->publish(all_msg);
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

    if (!hasFutureScanStep(current_step_index_ + 1U)) {
      writeFinalInventorySnapshot("last_scan_led_off");
    }
    ++current_step_index_;
    startNextStep();
  }

  void startReturn()
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    return_steps_.clear();

    if (mission_mode_ == MissionMode::INVENTORY && !final_inventory_written_) {
      writeFinalInventorySnapshot("start_return");
    }

    if (mission_mode_ == MissionMode::TARGET && !ground_return_steps_.empty()) {
      return_steps_ = ground_return_steps_;
      return_step_index_ = 0;
      phase_ = MissionPhase::RETURN_TRANSIT;
      RCLCPP_INFO(get_logger(), "Returning on fixed target route. return_steps=%zu.", return_steps_.size());
      logEvent("return_start", "", accepted_barcode_, "fixed_target_landing_cruise");
      startNextReturnStep();
      return;
    }

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

  void completeTask()
  {
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);

    if (mission_mode_ == MissionMode::INVENTORY) {
      const bool success = inventoryComplete();
      writeFinalInventorySnapshot("mission_complete");
      RCLCPP_INFO(
        get_logger(),
        "Warehouse inventory completed: success=%d coord_records=%zu id_records=%zu missing=%zu.",
        success ? 1 : 0,
        inventory_by_coord_.size(),
        inventory_results_.size(),
        missingInventoryCoords().size());
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

  std::string latestSuccessPath() const
  {
    return log_dir_ + "/latest_success.json";
  }

  bool readInventoryNumber(
    const Json::Value & value,
    const std::string & key,
    double & output,
    std::string & error) const
  {
    if (!value.isMember(key) || !value[key].isNumeric()) {
      error = "inventory record field " + key + " must be a number";
      return false;
    }

    output = value[key].asDouble();
    if (!std::isfinite(output)) {
      error = "inventory record field " + key + " must be finite";
      return false;
    }
    return true;
  }

  bool loadLatestSuccessTarget(int target_id, SlotTarget & target_slot, std::string & error)
  {
    if (target_id <= 0) {
      error = "target id must be positive";
      return false;
    }

    std::ifstream input(latestSuccessPath());
    if (!input.is_open()) {
      error = "latest success inventory not found: " + latestSuccessPath();
      return false;
    }

    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    Json::Value root;
    std::string parse_errors;
    if (!Json::parseFromStream(builder, input, &root, &parse_errors)) {
      error = "latest success inventory JSON invalid: " + parse_errors;
      return false;
    }

    if (!root.isObject()) {
      error = "latest success inventory root must be an object";
      return false;
    }
    if (root.isMember("complete") && root["complete"].isBool() && !root["complete"].asBool()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Latest inventory is marked incomplete; target lookup will use available IDs only.");
    }

    const Json::Value & inventory = root["inventory"];
    if (!inventory.isObject()) {
      error = "latest success inventory missing inventory object";
      return false;
    }

    const std::string key = std::to_string(target_id);
    if (!inventory.isMember(key)) {
      error = "target id " + key + " not found in latest success inventory";
      return false;
    }

    const Json::Value & record = inventory[key];
    if (!record.isObject()) {
      error = "target id " + key + " inventory record must be an object";
      return false;
    }
    if (!record.isMember("coord") || !record["coord"].isString()) {
      error = "target id " + key + " inventory record coord must be a string";
      return false;
    }

    double x_cm = 0.0;
    double y_cm = 0.0;
    double z_cm = 0.0;
    double yaw_deg = 0.0;
    if (!readInventoryNumber(record, "x_cm", x_cm, error) ||
      !readInventoryNumber(record, "y_cm", y_cm, error) ||
      !readInventoryNumber(record, "z_cm", z_cm, error) ||
      !readInventoryNumber(record, "yaw_deg", yaw_deg, error))
    {
      error = "target id " + key + " " + error;
      return false;
    }

    const std::string coord = record["coord"].asString();
    if (!resolveFixedTargetSlot(coord, WaypointTarget{x_cm, y_cm, z_cm, yaw_deg}, target_slot, error)) {
      error = "target id " + key + " " + error;
      return false;
    }
    return true;
  }

  WaypointNavigator flight_;
  AuxController aux_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planned_route_pub_;
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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr st_ready_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  MissionMode mission_mode_{MissionMode::INVENTORY};
  MissionPhase phase_{MissionPhase::WAIT_MODE};
  double timer_period_sec_{0.05};
  std::string barcode_topic_;
  std::string fine_data_topic_;
  std::string ground_mode_topic_;
  std::string ground_route_topic_;
  std::string planned_route_topic_;
  std::string st_ready_topic_;
  std::string d_task_qr_id_topic_;
  std::string d_task_status_topic_;
  std::string log_dir_;
  std::string event_csv_path_;
  std::string final_inventory_path_;
  std::ofstream event_csv_;

  std::vector<RackSpec> racks_;
  std::vector<FaceSpec> faces_;
  std::vector<SafetyRect> safety_rects_;
  std::unordered_map<char, std::vector<SlotTarget>> slots_by_face_;
  std::vector<RouteStep> active_steps_;
  std::vector<RouteStep> publishable_route_;
  std::vector<RouteStep> ground_route_steps_;
  std::vector<RouteStep> ground_return_steps_;
  std::vector<RouteStep> return_steps_;
  std::size_t current_step_index_{0};
  std::size_t return_step_index_{0};

  std::map<int, InventoryRecord> inventory_results_;
  std::map<std::string, InventoryRecord> inventory_by_coord_;
  std::map<std::string, double> missed_coords_;
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
  rclcpp::Time last_status_pub_stamp_;
  rclcpp::Time target_mode_qr_window_start_stamp_;
  rclcpp::Time target_qr_laser_stamp_;
  int target_mode_qr_window_checks_{0};
  int last_target_lookup_failure_id_{0};
  std::string last_target_lookup_failure_error_;
  bool visual_takeover_enabled_{false};
  bool mission_locked_{false};
  bool st_ready_received_{false};
  bool ground_route_valid_{false};
  bool ground_route_loaded_{false};
  bool target_mode_qr_window_active_{false};
  bool target_qr_laser_active_{false};
  bool final_inventory_written_{false};
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
