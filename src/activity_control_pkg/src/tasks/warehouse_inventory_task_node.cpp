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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

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
  WAIT_TARGET_BARCODE,
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
    case MissionPhase::WAIT_TARGET_BARCODE:
      return "WAIT_TARGET_BARCODE";
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
    scan_result_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/scan_result", durable_qos);
    all_results_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/all_results", durable_qos);
    target_id_pub_ =
      create_publisher<std_msgs::msg::Int32>("/warehouse_inventory/target_id", durable_qos);
    query_result_pub_ =
      create_publisher<std_msgs::msg::String>("/warehouse_inventory/query_result", durable_qos);

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

    publishRoute(publishable_route_);

    phase_ = mission_mode_ == MissionMode::TARGET ?
      MissionPhase::WAIT_TARGET_BARCODE : MissionPhase::WAIT_STATE;
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.publishActiveController(3);

    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&WarehouseInventoryTaskNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Warehouse inventory task ready: mode=%s, route_steps=%zu, safety_clearance=%.1fcm.",
      mission_mode_ == MissionMode::TARGET ? "target" : "inventory",
      active_steps_.size(),
      kScanClearanceCm);
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
      FaceSpec{'A', 0, -1, 90.0},
      FaceSpec{'B', 0, 1, -90.0},
      FaceSpec{'C', 1, -1, 90.0},
      FaceSpec{'D', 1, 1, -90.0},
    };

    safety_rects_.clear();
    for (const auto & rack : racks_) {
      const double rack_x = fieldToMapX(rack.x_field_cm);
      safety_rects_.push_back(SafetyRect{
        rack_x - kRackHalfThicknessCm - kScanClearanceCm,
        fieldToMapY(kRackYMinFieldCm - kScanClearanceCm),
        rack_x + kRackHalfThicknessCm + kScanClearanceCm,
        fieldToMapY(kRackYMaxFieldCm + kScanClearanceCm),
      });
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
      WaypointTarget{
        fieldToMapX(scan_x_field),
        fieldToMapY(slot_y_field_cm),
        slot_z_cm,
        face.yaw_deg,
      },
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

    std::vector<std::vector<int>> orders;
    for (const bool top_first : {true, false}) {
      for (const bool increasing : {true, false}) {
        std::vector<int> top{1, 2, 3};
        std::vector<int> bottom{4, 5, 6};
        if (!increasing) {
          std::reverse(top.begin(), top.end());
          std::reverse(bottom.begin(), bottom.end());
        }
        auto second = top_first ? bottom : top;
        std::reverse(second.begin(), second.end());
        auto first = top_first ? top : bottom;
        first.insert(first.end(), second.begin(), second.end());
        orders.push_back(first);
      }
    }

    for (const bool increasing : {true, false}) {
      for (const bool top_first : {true, false}) {
        std::vector<int> cols{0, 1, 2};
        if (!increasing) {
          std::reverse(cols.begin(), cols.end());
        }
        std::vector<int> order;
        bool current_top_first = top_first;
        for (const int col : cols) {
          const int top_slot = col + 1;
          const int bottom_slot = col + 4;
          if (current_top_first) {
            order.push_back(top_slot);
            order.push_back(bottom_slot);
          } else {
            order.push_back(bottom_slot);
            order.push_back(top_slot);
          }
          current_top_first = !current_top_first;
        }
        orders.push_back(order);
      }
    }

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

    const double transit_z = std::max(start.z_cm, end.z_cm);
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      if (distance2d(path[i], path[i - 1]) < 0.5) {
        continue;
      }
      result.push_back(WaypointTarget{path[i].x, path[i].y, transit_z, end.yaw_deg});
    }
    return result;
  }

  void appendTransit(
    const WaypointTarget & start,
    const WaypointTarget & end,
    std::vector<RouteStep> & steps) const
  {
    const auto transit = safeTransitWaypoints(start, end);
    for (const auto & target : transit) {
      steps.push_back(RouteStep{target, false, "", "transit"});
    }
  }

  double safeCostBetween(const WaypointTarget & start, const WaypointTarget & end) const
  {
    double cost = 0.0;
    WaypointTarget current = start;
    auto transit = safeTransitWaypoints(start, end);
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
    std::array<char, 4> face_order{'A', 'B', 'C', 'D'};
    std::unordered_map<char, std::vector<std::vector<SlotTarget>>> variants;
    std::vector<SlotTarget> all_slots;
    std::unordered_map<std::string, std::size_t> slot_index;
    for (const char face : face_order) {
      variants[face] = faceVariants(face);
      const auto iter = slots_by_face_.find(face);
      if (iter == slots_by_face_.end()) {
        throw std::runtime_error("Missing face slots during planning.");
      }
      for (const auto & slot : iter->second) {
        slot_index[slot.coord] = all_slots.size();
        all_slots.push_back(slot);
      }
    }

    constexpr std::size_t kHomeIndex = kInventoryCount;
    constexpr std::size_t kLandingIndex = kInventoryCount + 1U;
    std::vector<WaypointTarget> cost_points;
    cost_points.reserve(kInventoryCount + 2U);
    for (const auto & slot : all_slots) {
      cost_points.push_back(slot.target);
    }
    cost_points.push_back(homeAtCruise());
    cost_points.push_back(landingAtCruise());

    std::vector<std::vector<double>> pair_cost(
      cost_points.size(),
      std::vector<double>(cost_points.size(), 0.0));
    for (std::size_t i = 0; i < cost_points.size(); ++i) {
      for (std::size_t j = 0; j < cost_points.size(); ++j) {
        if (i == j) {
          continue;
        }
        pair_cost[i][j] = safeCostBetween(cost_points[i], cost_points[j]);
      }
    }

    auto sequence_cost = [&slot_index, &pair_cost](const std::vector<SlotTarget> & sequence) {
        if (sequence.empty()) {
          return 0.0;
        }
        auto index_of = [&slot_index](const SlotTarget & slot) {
            const auto found = slot_index.find(slot.coord);
            if (found == slot_index.end()) {
              throw std::runtime_error("Route sequence references unknown slot.");
            }
            return found->second;
          };
        double cost = pair_cost[kHomeIndex][index_of(sequence.front())];
        for (std::size_t i = 1; i < sequence.size(); ++i) {
          cost += pair_cost[index_of(sequence[i - 1])][index_of(sequence[i])];
        }
        cost += pair_cost[index_of(sequence.back())][kLandingIndex];
        return cost;
      };

    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<SlotTarget> best_sequence;
    std::sort(face_order.begin(), face_order.end());
    do {
      const auto & v0 = variants[face_order[0]];
      const auto & v1 = variants[face_order[1]];
      const auto & v2 = variants[face_order[2]];
      const auto & v3 = variants[face_order[3]];
      for (const auto & s0 : v0) {
        for (const auto & s1 : v1) {
          for (const auto & s2 : v2) {
            for (const auto & s3 : v3) {
              std::vector<SlotTarget> sequence;
              sequence.reserve(24);
              sequence.insert(sequence.end(), s0.begin(), s0.end());
              sequence.insert(sequence.end(), s1.begin(), s1.end());
              sequence.insert(sequence.end(), s2.begin(), s2.end());
              sequence.insert(sequence.end(), s3.begin(), s3.end());
              const double cost = sequence_cost(sequence);
              if (cost < best_cost) {
                best_cost = cost;
                best_sequence = sequence;
              }
            }
          }
        }
      }
    } while (std::next_permutation(face_order.begin(), face_order.end()));

    if (best_sequence.size() != kInventoryCount) {
      throw std::runtime_error("Failed to plan a complete 24-slot warehouse route.");
    }

    auto steps = expandSequenceToSteps(best_sequence);
    RCLCPP_INFO(
      get_logger(),
      "Planned warehouse route: scan_points=%zu expanded_steps=%zu score=%.3f.",
      best_sequence.size(),
      steps.size(),
      best_cost);
    logRouteSafety(steps);
    return steps;
  }

  void logRouteSafety(const std::vector<RouteStep> & steps) const
  {
    for (const auto & step : steps) {
      if (!step.scan) {
        continue;
      }
      const auto face = step.coord.empty() ? '?' : step.coord.front();
      const int rack_index = (face == 'A' || face == 'B') ? 0 : 1;
      const auto rack_x = fieldToMapX(racks_[rack_index].x_field_cm);
      const double clearance = std::fabs(step.target.x_cm - rack_x);
      RCLCPP_INFO(
        get_logger(),
        "Route scan %-2s x=%.1f y=%.1f z=%.1f yaw=%.1f clearance=%.1fcm.",
        step.coord.c_str(),
        step.target.x_cm,
        step.target.y_cm,
        step.target.z_cm,
        step.target.yaw_deg,
        clearance);
    }
  }

  WaypointTarget homeAtCruise() const
  {
    return WaypointTarget{0.0, 0.0, kCruiseHeightCm, 0.0};
  }

  WaypointTarget landingAtCruise() const
  {
    return WaypointTarget{
      fieldToMapX(kLandingFieldXCm),
      fieldToMapY(kLandingFieldYCm),
      kCruiseHeightCm,
      0.0,
    };
  }

  WaypointTarget landingFinal() const
  {
    return WaypointTarget{
      fieldToMapX(kLandingFieldXCm),
      fieldToMapY(kLandingFieldYCm),
      kHomeZCm,
      0.0,
    };
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
      RCLCPP_INFO(get_logger(), "Target mode accepted preflight target id: %d.", target_id_);
      logEvent("target_id_detected", "", target_id_, "preflight");
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
      case MissionPhase::WAIT_TARGET_BARCODE:
        processWaitTargetBarcode();
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

  void processWaitTargetBarcode()
  {
    if (target_id_ <= 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Target mode waiting for preflight QR code on %s.",
        barcode_topic_.c_str());
      return;
    }

    SlotTarget target_slot;
    if (!loadLatestSuccessTarget(target_id_, target_slot)) {
      stopWithoutTakeoff("latest_success_missing_or_target_not_found");
      return;
    }

    active_steps_ = planDirectTargetRoute(target_slot);
    publishable_route_ = routeWithLanding(active_steps_);
    publishRoute(publishable_route_);
    target_slot_ = target_slot;
    RCLCPP_INFO(
      get_logger(),
      "Target mode loaded latest inventory: id=%d coord=%s. Direct route steps=%zu.",
      target_id_,
      target_slot.coord.c_str(),
      active_steps_.size());
    logEvent("target_route_loaded", target_slot.coord, target_id_, "wait_for_flight_state");
    phase_ = MissionPhase::WAIT_STATE;
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
    appendTransit(current, landingAtCruise(), display_steps);
    display_steps.push_back(RouteStep{landingAtCruise(), false, "", "landing_cruise"});
    display_steps.push_back(RouteStep{landingFinal(), false, "", "landing_final"});
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
    current_step_index_ = 0;
    return_step_index_ = 0;
    accepted_barcode_ = 0;
    aux_.setAll(0, 0, 0);
    setVisualTakeover(false);
    flight_.goTo(homeAtCruise());
    phase_ = MissionPhase::TAKEOFF;
    RCLCPP_INFO(get_logger(), "Warehouse mission takeoff to %.1fcm.", kCruiseHeightCm);
    logEvent("takeoff", "", 0, "start_mission");
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
    appendTransit(current, landingAtCruise(), return_steps_);
    return_steps_.push_back(RouteStep{landingAtCruise(), false, "", "landing_cruise"});
    return_step_index_ = 0;
    phase_ = MissionPhase::RETURN_TRANSIT;
    RCLCPP_INFO(get_logger(), "Returning to landing point. return_steps=%zu.", return_steps_.size());
    logEvent("return_start", "", accepted_barcode_, "landing_cruise");
    startNextReturnStep();
  }

  void startNextReturnStep()
  {
    if (return_step_index_ >= return_steps_.size()) {
      flight_.goTo(landingFinal());
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_result_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr barcode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr query_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  MissionMode mission_mode_{MissionMode::INVENTORY};
  MissionPhase phase_{MissionPhase::WAIT_STATE};
  double timer_period_sec_{0.05};
  std::string barcode_topic_;
  std::string fine_data_topic_;
  std::string log_dir_;
  std::string event_csv_path_;
  std::ofstream event_csv_;

  std::vector<RackSpec> racks_;
  std::vector<FaceSpec> faces_;
  std::vector<SafetyRect> safety_rects_;
  std::unordered_map<char, std::vector<SlotTarget>> slots_by_face_;
  std::vector<RouteStep> active_steps_;
  std::vector<RouteStep> publishable_route_;
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
  bool visual_takeover_enabled_{false};
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
