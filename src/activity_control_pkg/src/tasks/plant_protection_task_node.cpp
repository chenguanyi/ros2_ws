#include "activity_control_pkg/core/aux_controller.hpp"
#include "activity_control_pkg/core/image_cache.hpp"
#include "activity_control_pkg/core/waypoint_navigator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>

namespace activity_control_pkg
{
namespace tasks
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kColorHistogramBins = 96;
constexpr double kGreenIndexMin = -1.0;
constexpr double kGreenIndexMax = 2.0;
constexpr double kColorClusterMinFraction = 0.03;
constexpr double kColorModelAlpha = 0.25;
constexpr double kDecisionMarginRatio = 0.25;
constexpr std::size_t kMinColorSamples = 16;

struct RoiBounds
{
  std::size_t x0{0};
  std::size_t y0{0};
  std::size_t x1{0};
  std::size_t y1{0};
};

struct ColorRoiStats
{
  bool valid{false};
  std::size_t count{0};
  double mean{0.0};
  std::array<std::size_t, kColorHistogramBins> histogram{};
};

struct ColorSplit
{
  bool valid{false};
  double low_mean{0.0};
  double high_mean{0.0};
  double confidence{0.0};
};

struct ColorDecision
{
  bool green{false};
  bool model_ready{false};
  double center_index{0.0};
  double threshold{0.0};
  double confidence{0.0};
  std::string reason{"unknown"};
};

RoiBounds centeredRoi(std::uint32_t width, std::uint32_t height, double fraction)
{
  const double clamped_fraction = std::clamp(fraction, 0.01, 1.0);
  const auto roi_width = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::lround(static_cast<double>(width) * clamped_fraction)));
  const auto roi_height = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::lround(static_cast<double>(height) * clamped_fraction)));
  const auto x0 = (static_cast<std::size_t>(width) - roi_width) / 2U;
  const auto y0 = (static_cast<std::size_t>(height) - roi_height) / 2U;
  return RoiBounds{x0, y0, x0 + roi_width, y0 + roi_height};
}

double greenIndex(std::uint8_t b, std::uint8_t g, std::uint8_t r)
{
  const double blue = static_cast<double>(b);
  const double green = static_cast<double>(g);
  const double red = static_cast<double>(r);
  const double total = red + green + blue;
  if (total <= 1.0) {
    return 0.0;
  }
  return (2.0 * green - red - blue) / total;
}

std::size_t histogramBin(double value)
{
  const double clamped = std::clamp(value, kGreenIndexMin, kGreenIndexMax);
  const double normalized = (clamped - kGreenIndexMin) / (kGreenIndexMax - kGreenIndexMin);
  const auto bin = static_cast<std::size_t>(
    std::floor(normalized * static_cast<double>(kColorHistogramBins)));
  return std::min(bin, kColorHistogramBins - 1U);
}

double histogramBinCenter(std::size_t bin)
{
  const double width =
    (kGreenIndexMax - kGreenIndexMin) / static_cast<double>(kColorHistogramBins);
  return kGreenIndexMin + (static_cast<double>(bin) + 0.5) * width;
}

std::size_t sampleStride(const RoiBounds & roi)
{
  const auto width = roi.x1 - roi.x0;
  const auto height = roi.y1 - roi.y0;
  const double pixel_count = static_cast<double>(std::max<std::size_t>(1U, width * height));
  constexpr double kTargetSamples = 8000.0;
  return std::max<std::size_t>(1U, static_cast<std::size_t>(std::sqrt(pixel_count / kTargetSamples)));
}

bool hasSupportedColorEncoding(const sensor_msgs::msg::Image & image)
{
  return image.encoding == sensor_msgs::image_encodings::BGR8;
}

bool hasUsableImageLayout(const sensor_msgs::msg::Image & image)
{
  if (!hasSupportedColorEncoding(image) || image.width == 0U || image.height == 0U) {
    return false;
  }
  const auto min_step = static_cast<std::size_t>(image.width) * 3U;
  if (static_cast<std::size_t>(image.step) < min_step) {
    return false;
  }
  const auto required_size =
    static_cast<std::size_t>(image.step) * (static_cast<std::size_t>(image.height) - 1U) +
    min_step;
  return image.data.size() >= required_size;
}

ColorRoiStats analyzeColorRoi(
  const sensor_msgs::msg::Image & image,
  const RoiBounds & roi,
  bool collect_histogram)
{
  ColorRoiStats stats;
  if (!hasUsableImageLayout(image)) {
    return stats;
  }

  const auto stride = sampleStride(roi);
  double sum = 0.0;
  for (std::size_t y = roi.y0; y < roi.y1; y += stride) {
    const auto row_offset = static_cast<std::size_t>(image.step) * y;
    for (std::size_t x = roi.x0; x < roi.x1; x += stride) {
      const auto offset = row_offset + x * 3U;
      if (offset + 2U >= image.data.size()) {
        continue;
      }

      const auto b = image.data[offset];
      const auto g = image.data[offset + 1U];
      const auto r = image.data[offset + 2U];
      if (static_cast<int>(b) + static_cast<int>(g) + static_cast<int>(r) < 30) {
        continue;
      }

      const double index = greenIndex(b, g, r);
      sum += index;
      ++stats.count;
      if (collect_histogram) {
        ++stats.histogram[histogramBin(index)];
      }
    }
  }

  if (stats.count >= kMinColorSamples) {
    stats.valid = true;
    stats.mean = sum / static_cast<double>(stats.count);
  }
  return stats;
}

ColorSplit splitColorHistogram(const ColorRoiStats & stats)
{
  ColorSplit split;
  if (!stats.valid || stats.count < kMinColorSamples) {
    return split;
  }

  double total_sum = 0.0;
  for (std::size_t bin = 0; bin < kColorHistogramBins; ++bin) {
    total_sum += static_cast<double>(stats.histogram[bin]) * histogramBinCenter(bin);
  }

  std::size_t low_count = 0U;
  double low_sum = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  ColorSplit best_split;
  for (std::size_t bin = 0; bin + 1U < kColorHistogramBins; ++bin) {
    low_count += stats.histogram[bin];
    low_sum += static_cast<double>(stats.histogram[bin]) * histogramBinCenter(bin);
    const auto high_count = stats.count - low_count;
    if (low_count == 0U || high_count == 0U) {
      continue;
    }

    const double low_fraction = static_cast<double>(low_count) / static_cast<double>(stats.count);
    const double high_fraction = static_cast<double>(high_count) / static_cast<double>(stats.count);
    if (std::min(low_fraction, high_fraction) < kColorClusterMinFraction) {
      continue;
    }

    const double low_mean = low_sum / static_cast<double>(low_count);
    const double high_mean = (total_sum - low_sum) / static_cast<double>(high_count);
    const double separation = high_mean - low_mean;
    if (separation <= 0.0) {
      continue;
    }

    const double score = low_fraction * high_fraction * separation * separation;
    if (score > best_score) {
      best_score = score;
      best_split = ColorSplit{true, low_mean, high_mean, separation};
    }
  }

  return best_split;
}

}  // namespace

enum class MissionPhase
{
  WAIT_STATE,
  TAKEOFF,
  GO_TO_BARCODE_SCAN,
  SCAN_BARCODE,
  GO_TO_A,
  SPRAY_CELL,
  NEXT_CELL,
  DISPLAY_BARCODE,
  RETURN_OR_CIRCLE_LAND,
  COMPLETE,
};

enum class LandingStep
{
  NONE,
  TRAVEL_AT_CRUISE,
  DESCEND,
};

struct PlantCell
{
  int id{0};
  WaypointTarget target;
  bool excluded{false};
};

class PlantProtectionTaskNode : public rclcpp::Node
{
public:
  PlantProtectionTaskNode()
  : rclcpp::Node("plant_protection_task_node"),
    flight_(*this),
    aux_(*this),
    image_cache_(*this)
  {
    timer_period_sec_ = declare_parameter("timer_period_sec", 0.05);
    cruise_height_cm_ = declare_parameter("cruise_height_cm", 150.0);
    home_pose_ = declare_parameter("home_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    cell_size_cm_ = declare_parameter("cell_size_cm", 50.0);
    laser_on_sec_ = declare_parameter("laser_on_sec", 0.4);
    laser_off_sec_ = declare_parameter("laser_off_sec", 0.8);
    laser_pulse_count_ = declare_parameter("laser_pulse_count", 2);
    color_decision_roi_fraction_ = declare_parameter("color_decision_roi_fraction", 0.35);
    color_threshold_roi_fraction_ = declare_parameter("color_threshold_roi_fraction", 0.80);
    color_max_image_age_sec_ = declare_parameter("color_max_image_age_sec", 0.5);
    color_min_confidence_ = declare_parameter("color_min_confidence", 0.08);
    color_non_green_index_max_ = declare_parameter("color_non_green_index_max", 0.10);
    configured_excluded_cells_ = declare_parameter("excluded_cells", std::vector<int64_t>{});
    barcode_value_ = declare_parameter("barcode_value", 0);
    barcode_topic_ =
      declare_parameter("barcode_topic", std::string("/plant_protection/barcode_value"));
    barcode_scan_pose_ =
      declare_parameter("barcode_scan_pose", std::vector<double>{0.0, 120.0, 105.0, 0.0});
    barcode_scan_timeout_sec_ = declare_parameter("barcode_scan_timeout_sec", 10.0);
    circle_landing_angle_deg_ = declare_parameter("circle_landing_angle_deg", 0.0);
    led_on_sec_ = declare_parameter("led_on_sec", 0.25);
    led_off_sec_ = declare_parameter("led_off_sec", 0.25);
    led_repeat_gap_sec_ = declare_parameter("led_repeat_gap_sec", 2.0);

    validateParameters();
    loadCells();

    mission_complete_pub_ =
      create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
    const auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    barcode_sub_ = create_subscription<std_msgs::msg::Int32>(
      barcode_topic_,
      barcode_qos,
      std::bind(&PlantProtectionTaskNode::barcodeCallback, this, std::placeholders::_1));
    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&PlantProtectionTaskNode::timerCallback, this));

    aux_.setAll(0, 0, 0);
    flight_.publishActiveController(3);
    RCLCPP_INFO(
      get_logger(),
      "Plant protection task ready. magnet=laser, signal=LED, camera=color gate. Waiting for height and TF.");
  }

private:
  void validateParameters()
  {
    if (home_pose_.size() != 4) {
      throw std::runtime_error("home_pose must contain [x_cm, y_cm, z_cm, yaw_deg].");
    }
    if (timer_period_sec_ <= 0.0) {
      throw std::runtime_error("timer_period_sec must be > 0.");
    }
    if (cell_size_cm_ <= 0.0) {
      throw std::runtime_error("cell_size_cm must be > 0.");
    }
    if (cruise_height_cm_ <= home_pose_[2]) {
      throw std::runtime_error("cruise_height_cm must be greater than home_pose z.");
    }
    if (laser_on_sec_ < 0.0 || laser_off_sec_ < 0.0) {
      throw std::runtime_error("laser_on_sec and laser_off_sec must be >= 0.");
    }
    if (laser_pulse_count_ < 0) {
      throw std::runtime_error("laser_pulse_count must be >= 0.");
    }
    if (color_decision_roi_fraction_ <= 0.0 || color_decision_roi_fraction_ > 1.0) {
      throw std::runtime_error("color_decision_roi_fraction must be in (0, 1].");
    }
    if (color_threshold_roi_fraction_ <= 0.0 || color_threshold_roi_fraction_ > 1.0) {
      throw std::runtime_error("color_threshold_roi_fraction must be in (0, 1].");
    }
    if (color_max_image_age_sec_ <= 0.0) {
      throw std::runtime_error("color_max_image_age_sec must be > 0.");
    }
    if (color_min_confidence_ <= 0.0) {
      throw std::runtime_error("color_min_confidence must be > 0.");
    }
    if (color_non_green_index_max_ < kGreenIndexMin ||
      color_non_green_index_max_ > kGreenIndexMax)
    {
      throw std::runtime_error("color_non_green_index_max is outside the green index range.");
    }
    if (barcode_scan_pose_.size() != 4) {
      throw std::runtime_error("barcode_scan_pose must contain [x_cm, y_cm, z_cm, yaw_deg].");
    }
    if (barcode_scan_timeout_sec_ <= 0.0) {
      throw std::runtime_error("barcode_scan_timeout_sec must be > 0.");
    }
    if (led_on_sec_ < 0.0 || led_off_sec_ < 0.0 || led_repeat_gap_sec_ < 0.0) {
      throw std::runtime_error("LED timing parameters must be >= 0.");
    }
  }

  WaypointTarget makeTarget(double x_cells, double y_cells) const
  {
    return WaypointTarget{
      home_pose_[0] + x_cells * cell_size_cm_,
      home_pose_[1] + y_cells * cell_size_cm_,
      cruise_height_cm_,
      home_pose_[3],
    };
  }

  std::unordered_map<int, WaypointTarget> buildCellMap() const
  {
    return {
      {1, makeTarget(7.0, 0.0)},
      {2, makeTarget(6.0, 0.0)},
      {3, makeTarget(5.0, 0.0)},
      {4, makeTarget(4.0, 0.0)},
      {5, makeTarget(7.0, 1.0)},
      {6, makeTarget(6.0, 1.0)},
      {7, makeTarget(5.0, 1.0)},
      {8, makeTarget(4.0, 1.0)},
      {9, makeTarget(5.0, 2.0)},
      {10, makeTarget(4.0, 2.0)},
      {11, makeTarget(7.0, 3.0)},
      {12, makeTarget(6.0, 3.0)},
      {13, makeTarget(5.0, 3.0)},
      {14, makeTarget(4.0, 3.0)},
      {15, makeTarget(7.0, 4.0)},
      {16, makeTarget(6.0, 4.0)},
      {17, makeTarget(5.0, 4.0)},
      {18, makeTarget(4.0, 4.0)},
      {19, makeTarget(3.0, 4.0)},
      {20, makeTarget(2.0, 4.0)},
      {21, makeTarget(1.0, 4.0)},
      {22, makeTarget(7.0, 5.0)},
      {23, makeTarget(6.0, 5.0)},
      {24, makeTarget(5.0, 5.0)},
      {25, makeTarget(4.0, 5.0)},
      {26, makeTarget(3.0, 5.0)},
      {27, makeTarget(2.0, 5.0)},
      {28, makeTarget(1.0, 5.0)},
    };
  }

  void loadCells()
  {
    const std::vector<int> route{
      21, 20, 19, 18, 14, 10, 8, 4,
      3, 2, 1, 5, 6, 7, 9, 13,
      12, 11, 15, 22, 23, 16, 17, 24,
      25, 26, 27, 28,
    };

    const auto cell_map = buildCellMap();
    std::unordered_set<int> excluded;
    for (const auto value : configured_excluded_cells_) {
      const int cell_id = static_cast<int>(value);
      if (cell_map.find(cell_id) == cell_map.end()) {
        RCLCPP_WARN(get_logger(), "Ignoring excluded cell %d because it is not in 1..28.", cell_id);
        continue;
      }
      excluded.insert(cell_id);
    }

    cells_.clear();
    cells_.reserve(route.size());
    total_spray_cells_ = 0;
    for (const int cell_id : route) {
      const auto iter = cell_map.find(cell_id);
      if (iter == cell_map.end()) {
        throw std::runtime_error("Internal route references unknown cell " + std::to_string(cell_id));
      }
      const bool is_excluded = excluded.count(cell_id) > 0;
      cells_.push_back(PlantCell{cell_id, iter->second, is_excluded});
      if (!is_excluded) {
        ++total_spray_cells_;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Plant route ready: cells=%zu spray=%zu excluded=%zu barcode=%d.",
      cells_.size(),
      total_spray_cells_,
      excluded.size(),
      barcode_value_);
  }

  WaypointTarget homeAtCruise() const
  {
    return WaypointTarget{home_pose_[0], home_pose_[1], cruise_height_cm_, home_pose_[3]};
  }

  WaypointTarget barcodeScanTarget() const
  {
    return WaypointTarget{
      barcode_scan_pose_[0],
      barcode_scan_pose_[1],
      barcode_scan_pose_[2],
      barcode_scan_pose_[3],
    };
  }

  WaypointTarget landingTarget() const
  {
    if (barcode_value_ <= 0) {
      return WaypointTarget{home_pose_[0], home_pose_[1], home_pose_[2], home_pose_[3]};
    }

    const double radius_cm = static_cast<double>(barcode_value_) * 10.0;
    const double angle_rad = circle_landing_angle_deg_ * kPi / 180.0;
    return WaypointTarget{
      home_pose_[0] + radius_cm * std::cos(angle_rad),
      home_pose_[1] + radius_cm * std::sin(angle_rad),
      home_pose_[2],
      home_pose_[3],
    };
  }

  void startTask()
  {
    current_cell_index_ = 0;
    sprayed_cells_ = 0;
    laser_cells_ = 0;
    barcode_detected_ = barcode_value_ > 0;
    barcode_scan_started_ = false;
    barcode_scan_complete_ = false;
    led_display_active_ = false;
    aux_.setAll(0, 0, 0);
    flight_.goTo(homeAtCruise());
    phase_ = MissionPhase::TAKEOFF;
    RCLCPP_INFO(get_logger(), "Mission started: takeoff to %.1fcm.", cruise_height_cm_);
  }

  void startBarcodeScanTravel()
  {
    aux_.setMagnet(false);
    const auto target = barcodeScanTarget();
    flight_.goTo(target);
    phase_ = MissionPhase::GO_TO_BARCODE_SCAN;
    RCLCPP_INFO(
      get_logger(),
      "Going to barcode scan pose x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg.",
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg);
  }

  void startBarcodeScan()
  {
    aux_.setMagnet(false);
    barcode_scan_mark_ = now();
    barcode_scan_started_ = true;
    phase_ = MissionPhase::SCAN_BARCODE;
    RCLCPP_INFO(
      get_logger(),
      "Barcode scan started. timeout=%.1fs.",
      barcode_scan_timeout_sec_);
  }

  void processBarcodeScan()
  {
    aux_.setMagnet(false);
    if (barcode_detected_ && barcode_value_ > 0) {
      barcode_scan_complete_ = true;
      startBarcodeDisplay();
      startGoToA();
      return;
    }

    if ((now() - barcode_scan_mark_).seconds() < barcode_scan_timeout_sec_) {
      return;
    }

    barcode_value_ = 0;
    barcode_detected_ = false;
    barcode_scan_complete_ = true;
    RCLCPP_WARN(
      get_logger(),
      "Barcode scan timed out after %.1fs. Continuing spray mission with barcode=0.",
      barcode_scan_timeout_sec_);
    startGoToA();
  }

  void startGoToA()
  {
    aux_.setMagnet(false);
    if (cells_.empty()) {
      startReturnOrCircleLanding();
      return;
    }
    flight_.goTo(cells_.front().target);
    phase_ = MissionPhase::GO_TO_A;
    RCLCPP_INFO(get_logger(), "Going to A start cell: %d.", cells_.front().id);
  }

  void startNextCell()
  {
    resetSprayState();
    aux_.setMagnet(false);
    while (current_cell_index_ < cells_.size() && cells_[current_cell_index_].excluded) {
      aux_.setMagnet(false);
      RCLCPP_DEBUG(
        get_logger(),
        "Skipping excluded cell %d.",
        cells_[current_cell_index_].id);
      ++current_cell_index_;
    }

    if (current_cell_index_ >= cells_.size()) {
      startReturnOrCircleLanding();
      return;
    }

    const auto & cell = cells_[current_cell_index_];
    flight_.goTo(cell.target);
    phase_ = MissionPhase::SPRAY_CELL;
    RCLCPP_DEBUG(get_logger(), "Going to cell %d (%zu/%zu).", cell.id, current_cell_index_ + 1, cells_.size());
  }

  void resetSprayState()
  {
    spray_decision_made_ = false;
    laser_is_on_ = false;
    laser_off_started_ = false;
    laser_pulses_started_ = 0;
    laser_pulses_completed_ = 0;
  }

  void updateColorModelFromLatestImage()
  {
    const auto image = image_cache_.latest();
    if (!image || image == last_color_update_image_) {
      return;
    }
    last_color_update_image_ = image;

    const auto roi = centeredRoi(
      image->width,
      image->height,
      color_threshold_roi_fraction_);
    const auto stats = analyzeColorRoi(*image, roi, true);
    if (!stats.valid) {
      return;
    }

    const auto split = splitColorHistogram(stats);
    if (split.valid && split.confidence >= color_min_confidence_) {
      updateColorClusters(split.low_mean, split.high_mean);
      return;
    }

    observeColorMean(stats.mean);
  }

  void observeColorMean(double mean)
  {
    if (!color_model_initialized_) {
      color_low_mean_ = mean;
      color_high_mean_ = mean;
      color_model_initialized_ = true;
      return;
    }

    color_low_mean_ = std::min(color_low_mean_, mean);
    color_high_mean_ = std::max(color_high_mean_, mean);
  }

  void updateColorClusters(double low_mean, double high_mean)
  {
    if (low_mean > high_mean) {
      std::swap(low_mean, high_mean);
    }

    if (!color_model_initialized_) {
      color_low_mean_ = low_mean;
      color_high_mean_ = high_mean;
      color_model_initialized_ = true;
      return;
    }

    color_low_mean_ = (1.0 - kColorModelAlpha) * color_low_mean_ + kColorModelAlpha * low_mean;
    color_high_mean_ = (1.0 - kColorModelAlpha) * color_high_mean_ + kColorModelAlpha * high_mean;
    if (color_low_mean_ > color_high_mean_) {
      std::swap(color_low_mean_, color_high_mean_);
    }
  }

  bool colorModelReady() const
  {
    return color_model_initialized_ && colorConfidence() >= color_min_confidence_;
  }

  double colorThreshold() const
  {
    return (color_low_mean_ + color_high_mean_) * 0.5;
  }

  double colorConfidence() const
  {
    return color_high_mean_ - color_low_mean_;
  }

  ColorDecision decideCellColor()
  {
    updateColorModelFromLatestImage();

    ColorDecision decision;
    const auto image = image_cache_.latest();
    if (!image) {
      decision.green = true;
      decision.reason = "vision_fail_open_no_image";
      return decision;
    }

    const rclcpp::Time stamp(image->header.stamp);
    if (stamp.nanoseconds() == 0) {
      decision.green = true;
      decision.reason = "vision_fail_open_no_stamp";
      return decision;
    }

    const double image_age_sec = (this->now() - stamp).seconds();
    if (image_age_sec < 0.0 || image_age_sec > color_max_image_age_sec_) {
      decision.green = true;
      decision.reason = "vision_fail_open_stale_image";
      return decision;
    }

    if (!hasSupportedColorEncoding(*image)) {
      decision.green = true;
      decision.reason = "vision_fail_open_unsupported_encoding_" + image->encoding;
      return decision;
    }

    const auto roi = centeredRoi(
      image->width,
      image->height,
      color_decision_roi_fraction_);
    const auto center_stats = analyzeColorRoi(*image, roi, false);
    if (!center_stats.valid) {
      decision.green = true;
      decision.reason = "vision_fail_open_invalid_center_roi";
      return decision;
    }

    decision.center_index = center_stats.mean;
    decision.model_ready = colorModelReady();
    decision.threshold = colorThreshold();
    decision.confidence = colorConfidence();
    if (decision.center_index <= color_non_green_index_max_) {
      decision.reason = "non_green_strong";
      return decision;
    }

    decision.green = true;
    if (!decision.model_ready) {
      decision.reason = "green_relaxed_threshold_unstable";
      return decision;
    }

    const double margin = color_min_confidence_ * kDecisionMarginRatio;
    decision.reason =
      decision.center_index >= decision.threshold + margin ? "green" : "green_relaxed";
    return decision;
  }

  void startLaserPulse(const PlantCell & cell)
  {
    const auto now = this->now();
    aux_.setMagnet(true);
    laser_is_on_ = true;
    laser_off_started_ = false;
    ++laser_pulses_started_;
    spray_mark_ = now;
    RCLCPP_INFO(
      get_logger(),
      "Cell %d: laser pulse %d/%d on via magnet=1.",
      cell.id,
      laser_pulses_started_,
      laser_pulse_count_);
  }

  void finishCurrentCell(const PlantCell & cell, bool laser_fired)
  {
    aux_.setMagnet(false);
    recordCellProgress(cell.id, laser_fired);
    ++current_cell_index_;
    phase_ = MissionPhase::NEXT_CELL;
    resetSprayState();
  }

  void processSprayCell()
  {
    if (current_cell_index_ >= cells_.size()) {
      startNextCell();
      return;
    }

    if (!flight_.isReached()) {
      aux_.setMagnet(false);
      return;
    }

    const auto & cell = cells_[current_cell_index_];
    if (cell.excluded) {
      aux_.setMagnet(false);
      RCLCPP_DEBUG(get_logger(), "Skipping excluded cell %d.", cell.id);
      ++current_cell_index_;
      phase_ = MissionPhase::NEXT_CELL;
      return;
    }

    if (!spray_decision_made_) {
      const auto decision = decideCellColor();
      spray_decision_made_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Cell %d color decision: %s center=%.3f threshold=%.3f confidence=%.3f.",
        cell.id,
        decision.reason.c_str(),
        decision.center_index,
        decision.threshold,
        decision.confidence);

      if (!decision.green) {
        finishCurrentCell(cell, false);
        return;
      }

      if (laser_on_sec_ <= 0.0 || laser_pulse_count_ <= 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "Dry-run spray skipped at cell %d because laser timing disables pulses.",
          cell.id);
        finishCurrentCell(cell, false);
        return;
      }

      startLaserPulse(cell);
      return;
    }

    const auto now = this->now();
    if (laser_is_on_) {
      if ((now - spray_mark_).seconds() < laser_on_sec_) {
        return;
      }
      aux_.setMagnet(false);
      laser_is_on_ = false;
      ++laser_pulses_completed_;
      spray_mark_ = now;
      RCLCPP_INFO(
        get_logger(),
        "Cell %d: laser pulse %d/%d off via magnet=0.",
        cell.id,
        laser_pulses_completed_,
        laser_pulse_count_);
      if (laser_pulses_completed_ >= laser_pulse_count_) {
        finishCurrentCell(cell, true);
        return;
      }
      laser_off_started_ = true;
      return;
    }

    if (laser_off_started_ && (now - spray_mark_).seconds() < laser_off_sec_) {
      return;
    }

    startLaserPulse(cell);
  }

  void recordCellProgress(int cell_id, bool laser_fired)
  {
    ++sprayed_cells_;
    if (laser_fired) {
      ++laser_cells_;
    }
    if (sprayed_cells_ == 1 || sprayed_cells_ == total_spray_cells_ || sprayed_cells_ % 5 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Cell progress: %zu/%zu candidate cells processed (last=%d, laser_cells=%zu).",
        sprayed_cells_,
        total_spray_cells_,
        cell_id,
        laser_cells_);
      return;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "Cell progress: %zu/%zu candidate cells processed (last=%d, laser_cells=%zu).",
      sprayed_cells_,
      total_spray_cells_,
      cell_id,
      laser_cells_);
  }

  void startBarcodeDisplay()
  {
    if (barcode_value_ <= 0) {
      return;
    }
    aux_.setSignal(false);
    led_display_active_ = true;
    led_round_ = 0;
    led_flash_index_ = 0;
    led_phase_active_ = false;
    led_is_on_ = false;
    led_gap_active_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Starting continuous LED barcode display: value=%d.",
      barcode_value_);
  }

  void processBarcodeDisplay()
  {
    if (!led_display_active_ || barcode_value_ <= 0) {
      aux_.setSignal(false);
      return;
    }

    const auto now = this->now();
    if (led_flash_index_ >= barcode_value_) {
      aux_.setSignal(false);
      if (!led_gap_active_) {
        led_gap_active_ = true;
        led_mark_ = now;
        RCLCPP_DEBUG(get_logger(), "LED display round %d complete.", led_round_ + 1);
        return;
      }
      if ((now - led_mark_).seconds() < led_repeat_gap_sec_) {
        return;
      }
      led_gap_active_ = false;
      ++led_round_;
      led_flash_index_ = 0;
      led_phase_active_ = false;
      return;
    }

    if (!led_phase_active_) {
      aux_.setSignal(true);
      led_phase_active_ = true;
      led_is_on_ = true;
      led_mark_ = now;
      return;
    }

    if (led_is_on_) {
      if ((now - led_mark_).seconds() < led_on_sec_) {
        return;
      }
      aux_.setSignal(false);
      led_is_on_ = false;
      led_mark_ = now;
      return;
    }

    if ((now - led_mark_).seconds() < led_off_sec_) {
      return;
    }

    ++led_flash_index_;
    led_phase_active_ = false;
  }

  void startReturnOrCircleLanding()
  {
    aux_.setArm(false);
    aux_.setMagnet(false);
    final_landing_target_ = landingTarget();
    const WaypointTarget cruise_target{
      final_landing_target_.x_cm,
      final_landing_target_.y_cm,
      cruise_height_cm_,
      final_landing_target_.yaw_deg,
    };
    flight_.goTo(cruise_target);
    landing_step_ = LandingStep::TRAVEL_AT_CRUISE;
    phase_ = MissionPhase::RETURN_OR_CIRCLE_LAND;
    RCLCPP_INFO(
      get_logger(),
      "Returning to landing point x=%.1fcm y=%.1fcm, barcode=%d.",
      final_landing_target_.x_cm,
      final_landing_target_.y_cm,
      barcode_value_);
  }

  void processLanding()
  {
    aux_.setMagnet(false);
    if (landing_step_ == LandingStep::TRAVEL_AT_CRUISE) {
      if (!flight_.isReached()) {
        return;
      }
      flight_.goTo(final_landing_target_);
      landing_step_ = LandingStep::DESCEND;
      RCLCPP_INFO(get_logger(), "Descending to final landing height %.1fcm.", final_landing_target_.z_cm);
      return;
    }

    if (landing_step_ == LandingStep::DESCEND) {
      if (!flight_.isReached()) {
        return;
      }
      completeTask();
    }
  }

  void completeTask()
  {
    if (phase_ == MissionPhase::COMPLETE) {
      return;
    }

    aux_.setAll(0, 0, 0);
    flight_.publishActiveController(3);
    std_msgs::msg::Empty message;
    mission_complete_pub_->publish(message);
    phase_ = MissionPhase::COMPLETE;
    RCLCPP_INFO(get_logger(), "Plant protection task completed.");
  }

  void barcodeCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data <= 0) {
      return;
    }
    if (barcode_scan_complete_) {
      return;
    }

    const bool changed = barcode_value_ != msg->data || !barcode_detected_;
    barcode_value_ = msg->data;
    barcode_detected_ = true;
    if (changed) {
      RCLCPP_INFO(get_logger(), "Received stable barcode value: %d.", barcode_value_);
    }
    if (barcode_scan_started_ && !led_display_active_) {
      startBarcodeDisplay();
    }
  }

  void timerCallback()
  {
    if (phase_ != MissionPhase::WAIT_STATE && phase_ != MissionPhase::COMPLETE) {
      updateColorModelFromLatestImage();
    }

    switch (phase_) {
      case MissionPhase::WAIT_STATE:
        if (flight_.hasState()) {
          startTask();
        }
        break;
      case MissionPhase::TAKEOFF:
        aux_.setMagnet(false);
        if (flight_.isReached()) {
          startBarcodeScanTravel();
        }
        break;
      case MissionPhase::GO_TO_BARCODE_SCAN:
        aux_.setMagnet(false);
        if (flight_.isReached()) {
          startBarcodeScan();
        }
        break;
      case MissionPhase::SCAN_BARCODE:
        processBarcodeScan();
        break;
      case MissionPhase::GO_TO_A:
        aux_.setMagnet(false);
        if (flight_.isReached()) {
          phase_ = MissionPhase::SPRAY_CELL;
          RCLCPP_INFO(get_logger(), "Reached A start cell.");
        }
        break;
      case MissionPhase::SPRAY_CELL:
        processSprayCell();
        break;
      case MissionPhase::NEXT_CELL:
        startNextCell();
        break;
      case MissionPhase::DISPLAY_BARCODE:
        processBarcodeDisplay();
        break;
      case MissionPhase::RETURN_OR_CIRCLE_LAND:
        processLanding();
        break;
      case MissionPhase::COMPLETE:
        break;
    }

    if (phase_ != MissionPhase::WAIT_STATE && phase_ != MissionPhase::COMPLETE) {
      processBarcodeDisplay();
    }
  }

  WaypointNavigator flight_;
  AuxController aux_;
  ImageCache image_cache_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr barcode_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  double timer_period_sec_{0.05};
  double cruise_height_cm_{150.0};
  std::vector<double> home_pose_{0.0, 0.0, 0.0, 0.0};
  double cell_size_cm_{50.0};
  double laser_on_sec_{0.4};
  double laser_off_sec_{0.8};
  int laser_pulse_count_{2};
  double color_decision_roi_fraction_{0.35};
  double color_threshold_roi_fraction_{0.80};
  double color_max_image_age_sec_{0.5};
  double color_min_confidence_{0.08};
  double color_non_green_index_max_{0.10};
  std::vector<int64_t> configured_excluded_cells_;
  int barcode_value_{0};
  std::string barcode_topic_{"/plant_protection/barcode_value"};
  std::vector<double> barcode_scan_pose_{0.0, 120.0, 105.0, 0.0};
  double barcode_scan_timeout_sec_{10.0};
  double circle_landing_angle_deg_{0.0};
  double led_on_sec_{0.25};
  double led_off_sec_{0.25};
  double led_repeat_gap_sec_{2.0};

  std::vector<PlantCell> cells_;
  std::size_t current_cell_index_{0};
  std::size_t total_spray_cells_{0};
  std::size_t sprayed_cells_{0};
  std::size_t laser_cells_{0};
  MissionPhase phase_{MissionPhase::WAIT_STATE};
  LandingStep landing_step_{LandingStep::NONE};
  WaypointTarget final_landing_target_;

  bool spray_decision_made_{false};
  bool laser_is_on_{false};
  bool laser_off_started_{false};
  int laser_pulses_started_{0};
  int laser_pulses_completed_{0};
  rclcpp::Time spray_mark_;

  bool barcode_detected_{false};
  bool barcode_scan_started_{false};
  bool barcode_scan_complete_{false};
  rclcpp::Time barcode_scan_mark_;

  sensor_msgs::msg::Image::ConstSharedPtr last_color_update_image_;
  bool color_model_initialized_{false};
  double color_low_mean_{0.0};
  double color_high_mean_{0.0};

  int led_round_{0};
  int led_flash_index_{0};
  bool led_display_active_{false};
  bool led_phase_active_{false};
  bool led_is_on_{false};
  bool led_gap_active_{false};
  rclcpp::Time led_mark_;
};

}  // namespace tasks
}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::tasks::PlantProtectionTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
