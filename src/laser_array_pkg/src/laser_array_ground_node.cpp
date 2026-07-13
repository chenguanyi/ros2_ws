// 面阵激光 —— 抗高台突变的地面高度估计节点
//
// 与 laser_array_driver.cpp 完全解耦：自带串口解析，只是在
// 64 束有效距离上跑空间 + 时间滤波，输出真正的"离地高度"，
// 同时提供障碍检测和障碍高度供搬运/避障逻辑使用。
//
// 发布的话题：
//   /laser_array/ground_height    (Float32, 单位: m)   给飞控的稳定高度
//   /laser_array/min_range        (Float32, 单位: m)   64 束中最小值（可能打到障碍/高台顶）
//   /laser_array/obstacle_below   (Bool)               下方是否有明显低于地面的物体
//   /laser_array/obstacle_height  (Float32, 单位: m)   障碍物相对地面的高度，无障碍时为 0
//   /laser_array/raw_percentile   (Float32)            滤波前的原始分位值（调参观察用）

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <boost/asio.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace laser_array_ground {

namespace {
constexpr double kPi = 3.14159265358979323846;

enum class ObstacleFusionMode : int {
  kNone = 0,
  kGeometry = 1,
  kGeometryUartBlend = 2,
  kLegacy = 3,
  kLegacyUartBlend = 4,
};
}

namespace protocol {
constexpr std::uint8_t HEADER[4] = {0x57, 0x01, 0xFF, 0x00};
constexpr std::size_t HEADER_SIZE = 4;
constexpr std::size_t TIMESTAMP_SIZE = 4;
constexpr std::size_t COUNT_SIZE = 1;
constexpr std::size_t DATA_UNIT_SIZE = 6;
constexpr std::size_t FOOTER_SIZE = 6;
constexpr std::size_t CHECKSUM_SIZE = 1;
constexpr std::uint8_t DATA_COUNT = 64;
constexpr std::size_t PACKET_SIZE =
  HEADER_SIZE + TIMESTAMP_SIZE + COUNT_SIZE +
  DATA_COUNT * DATA_UNIT_SIZE + FOOTER_SIZE + CHECKSUM_SIZE;
constexpr std::size_t MAX_BUFFER_SIZE = 4096;
}  // namespace protocol

class GroundHeightNode : public rclcpp::Node {
 public:
  GroundHeightNode() : Node("laser_array_ground_node") {
    // --- 串口 ---
    declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    declare_parameter<int>("baud_rate", 921600);

    // --- 空间滤波 ---
    // 无双峰时使用：1.0=最大值，0.8=80分位
    declare_parameter<double>("percentile", 1.0);
    // 双峰时优先使用: 远簇(地面)分位
    declare_parameter<double>("ground_percentile", 0.95);
    // 双峰时优先使用: 近簇(障碍)分位
    declare_parameter<double>("obstacle_percentile", 0.10);
    // 相邻两束距离差超过此值认为出现"双峰"(柱子 vs 地面)，单位 m
    declare_parameter<double>("cluster_gap", 0.20);

    // --- 时间滤波 ---
    declare_parameter<double>("max_slew_rate", 1.5);
    declare_parameter<double>("ema_alpha", 0.6);
    declare_parameter<double>("jump_threshold", 0.15);
    // 当所有光束都"短"且无远簇时：最多保持上一帧输出多少帧
    // 50Hz 数据下 20 帧 ≈ 0.4s，足够飞越一个 15cm 高台
    declare_parameter<int>("max_hold_frames", 20);

    // --- 障碍检测 ---
    declare_parameter<double>("obstacle_margin", 0.20);
    // --- 障碍高度偏置 ---
    declare_parameter<double>("obstacle_height_bias_cm", 0.0);

    // --- 几何化障碍测高 ---
    declare_parameter<bool>("geometry_obstacle_enabled", true);
    declare_parameter<bool>("use_geometry_for_ground_height", false);
    declare_parameter<int>("beam_rows", 8);
    declare_parameter<int>("beam_cols", 8);
    declare_parameter<double>("beam_horizontal_fov_deg", 45.0);
    declare_parameter<double>("beam_vertical_fov_deg", 45.0);
    declare_parameter<bool>("flip_rows", false);
    declare_parameter<bool>("flip_cols", false);
    declare_parameter<double>("beam_mount_roll_deg", 0.0);
    declare_parameter<double>("beam_mount_pitch_deg", 0.0);
    declare_parameter<double>("beam_mount_yaw_deg", 0.0);
    declare_parameter<double>("obstacle_top_size_cm", 20.0);
    declare_parameter<double>("obstacle_center_slack_cm", 15.0);
    declare_parameter<int>("geometry_min_support", 3);
    declare_parameter<double>("geometry_candidate_margin_cm", 5.0);
    declare_parameter<double>("geometry_height_tolerance_cm", 6.0);

    // --- UART 单点激光融合 ---
    declare_parameter<bool>("uart_height_fusion_enabled", true);
    declare_parameter<std::string>("uart_height_topic", "/height_raw");
    declare_parameter<double>("uart_height_bias_cm", 5.0);
    declare_parameter<double>("uart_height_timeout_sec", 0.30);
    declare_parameter<double>("uart_block_margin_cm", 8.0);
    declare_parameter<double>("uart_geometry_consistency_cm", 8.0);
    declare_parameter<double>("uart_blend_weight", 0.35);
    declare_parameter<double>("obstacle_ema_alpha", 0.3);

    // --- 日志 ---
    declare_parameter<double>("log_period_sec", 5.0);

    get_parameter("serial_port", serial_port_name_);
    get_parameter("baud_rate", baud_rate_);
    get_parameter("percentile", percentile_);
    get_parameter("ground_percentile", ground_percentile_);
    get_parameter("obstacle_percentile", obstacle_percentile_);
    get_parameter("cluster_gap", cluster_gap_);
    get_parameter("max_slew_rate", max_slew_rate_);
    get_parameter("ema_alpha", ema_alpha_);
    get_parameter("jump_threshold", jump_threshold_);
    get_parameter("max_hold_frames", max_hold_frames_);
    get_parameter("obstacle_margin", obstacle_margin_);
    get_parameter("obstacle_height_bias_cm", obstacle_height_bias_cm_);
    get_parameter("geometry_obstacle_enabled", geometry_obstacle_enabled_);
    get_parameter("use_geometry_for_ground_height", use_geometry_for_ground_height_);
    get_parameter("beam_rows", beam_rows_);
    get_parameter("beam_cols", beam_cols_);
    get_parameter("beam_horizontal_fov_deg", beam_horizontal_fov_deg_);
    get_parameter("beam_vertical_fov_deg", beam_vertical_fov_deg_);
    get_parameter("flip_rows", flip_rows_);
    get_parameter("flip_cols", flip_cols_);
    get_parameter("beam_mount_roll_deg", beam_mount_roll_deg_);
    get_parameter("beam_mount_pitch_deg", beam_mount_pitch_deg_);
    get_parameter("beam_mount_yaw_deg", beam_mount_yaw_deg_);
    get_parameter("obstacle_top_size_cm", obstacle_top_size_cm_);
    get_parameter("obstacle_center_slack_cm", obstacle_center_slack_cm_);
    get_parameter("geometry_min_support", geometry_min_support_);
    get_parameter("geometry_candidate_margin_cm", geometry_candidate_margin_cm_);
    get_parameter("geometry_height_tolerance_cm", geometry_height_tolerance_cm_);
    get_parameter("uart_height_fusion_enabled", uart_height_fusion_enabled_);
    get_parameter("uart_height_topic", uart_height_topic_);
    get_parameter("uart_height_bias_cm", uart_height_bias_cm_);
    get_parameter("uart_height_timeout_sec", uart_height_timeout_sec_);
    get_parameter("uart_block_margin_cm", uart_block_margin_cm_);
    get_parameter("uart_geometry_consistency_cm", uart_geometry_consistency_cm_);
    get_parameter("uart_blend_weight", uart_blend_weight_);
    get_parameter("obstacle_ema_alpha", obstacle_ema_alpha_);
    double log_period_sec = 5.0;
    get_parameter("log_period_sec", log_period_sec);

    percentile_ = std::clamp(percentile_, 0.0, 1.0);
    ground_percentile_ = std::clamp(ground_percentile_, 0.0, 1.0);
    obstacle_percentile_ = std::clamp(obstacle_percentile_, 0.0, 1.0);
    ema_alpha_ = std::clamp(ema_alpha_, 0.0, 1.0);
    obstacle_ema_alpha_ = std::clamp(obstacle_ema_alpha_, 0.0, 1.0);
    beam_rows_ = std::max(1, beam_rows_);
    beam_cols_ = std::max(1, beam_cols_);
    geometry_min_support_ = std::max(1, geometry_min_support_);
    uart_height_timeout_sec_ = std::max(0.05, uart_height_timeout_sec_);
    uart_block_margin_cm_ = std::max(0.0, uart_block_margin_cm_);
    uart_geometry_consistency_cm_ = std::max(0.0, uart_geometry_consistency_cm_);
    uart_blend_weight_ = std::clamp(uart_blend_weight_, 0.0, 1.0);

    ground_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/ground_height", 10);
    min_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/min_range", 10);
    obstacle_height_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/obstacle_height", 10);
    raw_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/raw_percentile", 10);
    obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("/laser_array/obstacle_below", 10);

    if (uart_height_fusion_enabled_) {
      uart_height_sub_ = create_subscription<std_msgs::msg::Int16>(
        uart_height_topic_,
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Int16::SharedPtr msg) {
          this->uartHeightCallback(msg);
        });
    }

    read_buffer_.resize(1024);
    parse_buffer_.resize(protocol::MAX_BUFFER_SIZE);
    configureBeamDirections();

    openSerial();

    // 定时打印当前高度（默认 5s 一次）
    const auto period = std::chrono::milliseconds(
        static_cast<int>(std::max(0.05, log_period_sec) * 1000.0));
    log_timer_ = create_wall_timer(period, [this]() {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_last_) {
        RCLCPP_INFO(get_logger(), "[height] waiting for first valid frame...");
        return;
      }
      RCLCPP_INFO(get_logger(),
                  "[height] out=%.3fm raw=%.3fm min=%.3fm obstacle_h=%.3fm geo_ground=%.3fm geo_support=%d uart_h=%.3fm uart_ob=%.3fm uart_fresh=%d uart_block=%d mode=%d valid=%d bimodal=%d hold=%d obstacle=%d geo=%d",
                  last_output_, last_raw_, last_min_, last_obstacle_height_,
                  last_geometry_ground_, last_geometry_support_, last_uart_height_corrected_,
                  last_uart_candidate_obstacle_height_, last_uart_fresh_ ? 1 : 0,
                  last_uart_blocked_ ? 1 : 0, static_cast<int>(last_fusion_mode_), last_valid_count_,
                  last_bimodal_ ? 1 : 0,
                  last_hold_counter_, last_obstacle_ ? 1 : 0, last_geometry_valid_ ? 1 : 0);
    });

    RCLCPP_INFO(get_logger(),
                "ground_node up: port=%s baud=%d pct=%.2f g_pct=%.2f o_pct=%.2f gap=%.2f slew=%.2fm/s alpha=%.2f jump=%.2fm hold=%d bias=%.1fcm geo=%d use_geo_ground=%d rows=%d cols=%d hfov=%.1f vfov=%.1f top=%.1fcm slack=%.1fcm support=%d",
                serial_port_name_.c_str(), baud_rate_, percentile_, ground_percentile_, obstacle_percentile_,
                cluster_gap_, max_slew_rate_, ema_alpha_, jump_threshold_, max_hold_frames_,
                obstacle_height_bias_cm_, geometry_obstacle_enabled_ ? 1 : 0,
                use_geometry_for_ground_height_ ? 1 : 0, beam_rows_, beam_cols_,
                beam_horizontal_fov_deg_, beam_vertical_fov_deg_, obstacle_top_size_cm_,
                obstacle_center_slack_cm_, geometry_min_support_);
    RCLCPP_INFO(
      get_logger(),
      "uart fusion: enabled=%d topic=%s bias=%.1fcm timeout=%.2fs block_margin=%.1fcm consistency=%.1fcm weight=%.2f",
      uart_height_fusion_enabled_ ? 1 : 0,
      uart_height_topic_.c_str(),
      uart_height_bias_cm_,
      uart_height_timeout_sec_,
      uart_block_margin_cm_,
      uart_geometry_consistency_cm_,
      uart_blend_weight_);
  }

  ~GroundHeightNode() override {
    running_ = false;
    if (serial_ && serial_->is_open()) {
      try { serial_->close(); } catch (...) {}
    }
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  struct BeamSample {
    std::size_t index;
    float distance_m;
  };

  struct BeamDirection {
    float x;
    float y;
    float z;
  };

  struct GeometryResult {
    bool has_ground{false};
    float ground_height_m{0.0f};
    bool valid{false};
    float obstacle_height_m{0.0f};
    int support_count{0};
    float center_x_m{0.0f};
    float center_y_m{0.0f};
  };

  struct UartHeightMeasurement {
    bool fresh{false};
    float raw_height_m{0.0f};
    float corrected_height_m{0.0f};
  };

  // 参数
  std::string serial_port_name_;
  int baud_rate_{921600};
  double percentile_{1.0};
  double ground_percentile_{0.95};
  double obstacle_percentile_{0.10};
  double cluster_gap_{0.20};
  double max_slew_rate_{1.5};
  double ema_alpha_{0.6};
  double obstacle_ema_alpha_{0.3};
  double jump_threshold_{0.15};
  int max_hold_frames_{20};
  double obstacle_margin_{0.20};
  double obstacle_height_bias_cm_{0.0};
  bool geometry_obstacle_enabled_{true};
  bool use_geometry_for_ground_height_{false};
  int beam_rows_{8};
  int beam_cols_{8};
  double beam_horizontal_fov_deg_{45.0};
  double beam_vertical_fov_deg_{45.0};
  bool flip_rows_{false};
  bool flip_cols_{false};
  double beam_mount_roll_deg_{0.0};
  double beam_mount_pitch_deg_{0.0};
  double beam_mount_yaw_deg_{0.0};
  double obstacle_top_size_cm_{20.0};
  double obstacle_center_slack_cm_{15.0};
  int geometry_min_support_{3};
  double geometry_candidate_margin_cm_{5.0};
  double geometry_height_tolerance_cm_{6.0};
  bool uart_height_fusion_enabled_{true};
  std::string uart_height_topic_{"/height_raw"};
  double uart_height_bias_cm_{5.0};
  double uart_height_timeout_sec_{0.30};
  double uart_block_margin_cm_{8.0};
  double uart_geometry_consistency_cm_{8.0};
  double uart_blend_weight_{0.35};
  std::vector<BeamDirection> beam_directions_;

  // 串口
  boost::asio::io_service io_;
  std::shared_ptr<boost::asio::serial_port> serial_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};

  // 解析缓冲
  std::vector<std::uint8_t> read_buffer_;
  std::vector<std::uint8_t> parse_buffer_;
  std::size_t buf_idx_{0};
  std::mutex buf_mtx_;

  // UART 单点高度缓存
  mutable std::mutex uart_mtx_;
  bool have_uart_height_{false};
  float latest_uart_height_m_{0.0f};
  rclcpp::Time latest_uart_stamp_;

  // 时间滤波状态
  std::mutex state_mtx_;
  bool have_last_{false};
  float last_output_{0.0f};
  float last_raw_{0.0f};
  float last_min_{0.0f};
  float last_obstacle_height_{0.0f};
  int last_valid_count_{0};
  bool last_bimodal_{false};
  bool last_obstacle_{false};
  float last_geometry_ground_{0.0f};
  int last_geometry_support_{0};
  bool last_geometry_valid_{false};
  float last_uart_height_corrected_{0.0f};
  float last_uart_candidate_obstacle_height_{0.0f};
  bool last_uart_fresh_{false};
  bool last_uart_blocked_{false};
  ObstacleFusionMode last_fusion_mode_{ObstacleFusionMode::kNone};
  int last_hold_counter_{0};
  int hold_counter_{0};
  rclcpp::Time last_stamp_;

  // 发布器 + 日志定时器
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ground_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr uart_height_sub_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  void uartHeightCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (!uart_height_fusion_enabled_) {
      return;
    }

    const float raw_height_m = static_cast<float>(msg->data) / 100.0f;
    std::lock_guard<std::mutex> lk(uart_mtx_);
    if (!std::isfinite(raw_height_m) || raw_height_m <= 0.0f) {
      have_uart_height_ = false;
      return;
    }

    latest_uart_height_m_ = raw_height_m;
    latest_uart_stamp_ = this->now();
    have_uart_height_ = true;
  }

  UartHeightMeasurement readUartHeightMeasurement(const rclcpp::Time& now) const {
    UartHeightMeasurement measurement;
    if (!uart_height_fusion_enabled_) {
      return measurement;
    }

    std::lock_guard<std::mutex> lk(uart_mtx_);
    if (!have_uart_height_ || latest_uart_stamp_.nanoseconds() == 0) {
      return measurement;
    }

    const double age_sec = (now - latest_uart_stamp_).seconds();
    if (age_sec < 0.0 || age_sec > uart_height_timeout_sec_) {
      return measurement;
    }

    measurement.fresh = true;
    measurement.raw_height_m = latest_uart_height_m_;
    measurement.corrected_height_m = latest_uart_height_m_ +
      static_cast<float>(uart_height_bias_cm_ / 100.0);
    return measurement;
  }

  void openSerial() {
    serial_ = std::make_shared<boost::asio::serial_port>(io_);
    serial_->open(serial_port_name_);
    using spb = boost::asio::serial_port_base;
    serial_->set_option(spb::baud_rate(baud_rate_));
    serial_->set_option(spb::character_size(8));
    serial_->set_option(spb::stop_bits(spb::stop_bits::one));
    serial_->set_option(spb::parity(spb::parity::none));
    serial_->set_option(spb::flow_control(spb::flow_control::none));

    running_ = true;
    io_thread_ = std::thread(&GroundHeightNode::readLoop, this);
  }

  void readLoop() {
    while (running_) {
      try {
        if (!serial_->is_open()) break;
        std::size_t n = serial_->read_some(boost::asio::buffer(read_buffer_));
        if (n > 0) {
          std::lock_guard<std::mutex> lk(buf_mtx_);
          if (buf_idx_ + n > parse_buffer_.size()) buf_idx_ = 0;
          std::memcpy(parse_buffer_.data() + buf_idx_, read_buffer_.data(), n);
          buf_idx_ += n;
          drainPackets();
        }
      } catch (const std::exception& e) {
        if (running_) {
          RCLCPP_WARN(get_logger(), "serial read err: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void drainPackets() {
    while (buf_idx_ >= protocol::PACKET_SIZE) {
      bool found = false;
      for (std::size_t i = 0; i + protocol::PACKET_SIZE <= buf_idx_; ++i) {
        if (std::memcmp(parse_buffer_.data() + i, protocol::HEADER,
                        protocol::HEADER_SIZE) != 0) continue;
        std::uint8_t cnt = parse_buffer_[i + protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE];
        if (cnt != protocol::DATA_COUNT) continue;
        std::uint8_t cksum = 0;
        for (std::size_t k = 0; k < protocol::PACKET_SIZE - 1; ++k)
          cksum += parse_buffer_[i + k];
        if (cksum != parse_buffer_[i + protocol::PACKET_SIZE - 1]) continue;

        handlePacket(parse_buffer_.data() + i);
        shift(i + protocol::PACKET_SIZE);
        found = true;
        break;
      }
      if (!found) { shift(1); break; }
    }
  }

  void shift(std::size_t off) {
    if (off >= buf_idx_) { buf_idx_ = 0; return; }
    std::memmove(parse_buffer_.data(), parse_buffer_.data() + off, buf_idx_ - off);
    buf_idx_ -= off;
  }

  static double degToRad(double deg) {
    return deg * kPi / 180.0;
  }

  static BeamDirection normalizeDirection(double x, double y, double z) {
    const double norm = std::sqrt(x * x + y * y + z * z);
    if (norm <= 0.0) {
      return BeamDirection{0.0f, 0.0f, 1.0f};
    }
    return BeamDirection{
      static_cast<float>(x / norm),
      static_cast<float>(y / norm),
      static_cast<float>(z / norm)};
  }

  BeamDirection applyMountRotation(const BeamDirection& dir) const {
    const double roll = degToRad(beam_mount_roll_deg_);
    const double pitch = degToRad(beam_mount_pitch_deg_);
    const double yaw = degToRad(beam_mount_yaw_deg_);

    double x = dir.x;
    double y = dir.y;
    double z = dir.z;

    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);

    const double y1 = cr * y - sr * z;
    const double z1 = sr * y + cr * z;
    y = y1;
    z = z1;

    const double x2 = cp * x + sp * z;
    const double z2 = -sp * x + cp * z;
    x = x2;
    z = z2;

    const double x3 = cy * x - sy * y;
    const double y3 = sy * x + cy * y;
    x = x3;
    y = y3;

    return normalizeDirection(x, y, z);
  }

  void configureBeamDirections() {
    if (beam_rows_ * beam_cols_ != static_cast<int>(protocol::DATA_COUNT)) {
      RCLCPP_WARN(
        get_logger(),
        "beam_rows*beam_cols=%d does not match %u samples, falling back to 8x8.",
        beam_rows_ * beam_cols_, protocol::DATA_COUNT);
      beam_rows_ = 8;
      beam_cols_ = 8;
    }

    beam_directions_.clear();
    beam_directions_.reserve(protocol::DATA_COUNT);
    for (std::size_t index = 0; index < protocol::DATA_COUNT; ++index) {
      int row = static_cast<int>(index) / beam_cols_;
      int col = static_cast<int>(index) % beam_cols_;
      if (flip_rows_) {
        row = beam_rows_ - 1 - row;
      }
      if (flip_cols_) {
        col = beam_cols_ - 1 - col;
      }

      const double row_norm = beam_rows_ > 1
        ? (static_cast<double>(row) / static_cast<double>(beam_rows_ - 1)) - 0.5
        : 0.0;
      const double col_norm = beam_cols_ > 1
        ? (static_cast<double>(col) / static_cast<double>(beam_cols_ - 1)) - 0.5
        : 0.0;
      const double pitch = row_norm * degToRad(beam_vertical_fov_deg_);
      const double yaw = col_norm * degToRad(beam_horizontal_fov_deg_);

      BeamDirection dir = normalizeDirection(std::tan(pitch), std::tan(yaw), 1.0);
      dir = applyMountRotation(dir);
      beam_directions_.push_back(dir);
    }
  }

  static float percentileValue(
      const std::vector<float>& data,
      std::size_t start,
      std::size_t count,
      double pct) {
    if (count == 0) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const double clamped = std::clamp(pct, 0.0, 1.0);
    const std::size_t offset = static_cast<std::size_t>(
      std::lround(clamped * static_cast<double>(count - 1)));
    const std::size_t idx = std::min(start + offset, start + count - 1);
    return data[idx];
  }

  static float decodeDistance(const std::uint8_t* p) {
    std::int32_t raw = p[0] | (p[1] << 8) | (p[2] << 16);
    if (raw & 0x800000) raw |= 0xFF000000;
    return static_cast<float>(raw) / 1000000.0f;
  }

  GeometryResult estimateGeometryObstacle(const std::vector<BeamSample>& valid) const {
    struct ProjectedBeam {
      float x_m;
      float y_m;
      float vertical_m;
      float obstacle_height_m;
    };

    GeometryResult result;
    if (!geometry_obstacle_enabled_ || beam_directions_.size() != protocol::DATA_COUNT) {
      return result;
    }

    std::vector<ProjectedBeam> projected;
    projected.reserve(valid.size());
    std::vector<float> vertical_ranges;
    vertical_ranges.reserve(valid.size());

    for (const auto& sample : valid) {
      if (sample.index >= beam_directions_.size() || !std::isfinite(sample.distance_m) ||
          sample.distance_m <= 0.0f) {
        continue;
      }
      const BeamDirection& dir = beam_directions_[sample.index];
      if (dir.z <= 0.05f) {
        continue;
      }

      const float x_m = dir.x * sample.distance_m;
      const float y_m = dir.y * sample.distance_m;
      const float vertical_m = dir.z * sample.distance_m;
      if (!std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(vertical_m) ||
          vertical_m <= 0.0f) {
        continue;
      }

      projected.push_back(ProjectedBeam{x_m, y_m, vertical_m, 0.0f});
      vertical_ranges.push_back(vertical_m);
    }

    if (vertical_ranges.empty()) {
      return result;
    }

    std::sort(vertical_ranges.begin(), vertical_ranges.end());
    float geometry_ground = percentileValue(vertical_ranges, 0, vertical_ranges.size(), ground_percentile_);
    if (!std::isfinite(geometry_ground)) {
      geometry_ground = vertical_ranges.back();
    }
    if (!std::isfinite(geometry_ground) || geometry_ground <= 0.0f) {
      return result;
    }

    result.has_ground = true;
    result.ground_height_m = geometry_ground;

    const float candidate_margin_m =
      static_cast<float>(std::max(0.0, geometry_candidate_margin_cm_)) / 100.0f;
    const float height_tol_m =
      static_cast<float>(std::max(0.0, geometry_height_tolerance_cm_)) / 100.0f;
    const float search_half_span_m =
      static_cast<float>(std::max(0.0, obstacle_top_size_cm_ * 0.5 + obstacle_center_slack_cm_)) / 100.0f;
    const float min_publish_height_m = static_cast<float>(obstacle_margin_);

    for (auto& beam : projected) {
      beam.obstacle_height_m = geometry_ground - beam.vertical_m;
    }

    int best_support = 0;
    float best_height = 0.0f;
    float best_spread = std::numeric_limits<float>::infinity();
    float best_center_x = 0.0f;
    float best_center_y = 0.0f;

    for (const auto& center : projected) {
      if (center.obstacle_height_m < candidate_margin_m) {
        continue;
      }

      std::vector<float> group_heights;
      group_heights.reserve(projected.size());
      float sum_x = 0.0f;
      float sum_y = 0.0f;
      for (const auto& beam : projected) {
        if (beam.obstacle_height_m < candidate_margin_m) {
          continue;
        }
        if (std::fabs(beam.x_m - center.x_m) > search_half_span_m ||
            std::fabs(beam.y_m - center.y_m) > search_half_span_m ||
            std::fabs(beam.obstacle_height_m - center.obstacle_height_m) > height_tol_m) {
          continue;
        }
        group_heights.push_back(beam.obstacle_height_m);
        sum_x += beam.x_m;
        sum_y += beam.y_m;
      }

      if (static_cast<int>(group_heights.size()) < geometry_min_support_) {
        continue;
      }

      std::sort(group_heights.begin(), group_heights.end());
      const float median_height =
        percentileValue(group_heights, 0, group_heights.size(), 0.5);
      const float q1 = percentileValue(group_heights, 0, group_heights.size(), 0.25);
      const float q3 = percentileValue(group_heights, 0, group_heights.size(), 0.75);
      const float spread = std::max(0.0f, q3 - q1);
      const int support = static_cast<int>(group_heights.size());

      const bool better_group =
        support > best_support ||
        (support == best_support && spread < best_spread) ||
        (support == best_support && std::fabs(spread - best_spread) < 1e-6f &&
         median_height > best_height);
      if (!better_group) {
        continue;
      }

      best_support = support;
      best_height = median_height;
      best_spread = spread;
      best_center_x = sum_x / static_cast<float>(support);
      best_center_y = sum_y / static_cast<float>(support);
    }

    if (best_support < geometry_min_support_ || best_height < min_publish_height_m) {
      return result;
    }

    result.valid = true;
    result.obstacle_height_m = best_height;
    result.support_count = best_support;
    result.center_x_m = best_center_x;
    result.center_y_m = best_center_y;
    return result;
  }

  void handlePacket(const std::uint8_t* pkt) {
    std::vector<BeamSample> valid;
    valid.reserve(protocol::DATA_COUNT);
    for (int i = 0; i < protocol::DATA_COUNT; ++i) {
      std::size_t off = protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE +
                        protocol::COUNT_SIZE + i * protocol::DATA_UNIT_SIZE;
      if (pkt[off + 3] != 0) {
        continue;
      }
      const float distance_m = decodeDistance(pkt + off);
      if (!std::isfinite(distance_m) || distance_m <= 0.0f) {
        continue;
      }
      valid.push_back(BeamSample{static_cast<std::size_t>(i), distance_m});
    }

    if (valid.empty()) {
      publishFloat(ground_pub_, std::numeric_limits<float>::quiet_NaN());
      publishFloat(obstacle_height_pub_, std::numeric_limits<float>::quiet_NaN());
      return;
    }

    processAndPublish(valid);
  }

  void processAndPublish(const std::vector<BeamSample>& valid_samples) {
    std::vector<float> valid;
    valid.reserve(valid_samples.size());
    for (const auto& sample : valid_samples) {
      valid.push_back(sample.distance_m);
    }
    std::sort(valid.begin(), valid.end());
    const std::size_t n = valid.size();
    const float min_range = valid.front();

    // --- 空间滤波：找最大间隙判断是否双峰 ---
    // 如果有柱子在下方但还有部分光束打到地面，距离值会分成"近簇(柱面)"
    // 和"远簇(地面)"两堆，中间有一个明显间隙。
    float max_gap = 0.0f;
    std::size_t split_idx = n;  // [split_idx, n) 为远簇
    for (std::size_t i = 1; i < n; ++i) {
      float gap = valid[i] - valid[i - 1];
      if (gap > max_gap) {
        max_gap = gap;
        split_idx = i;
      }
    }
    const bool bimodal = (max_gap > static_cast<float>(cluster_gap_)) &&
                         split_idx < n;

    float ground_est = 0.0f;
    float obstacle_est = 0.0f;
    if (bimodal && split_idx > 0 && split_idx < n) {
      const std::size_t near_count = split_idx;
      const std::size_t far_count = n - split_idx;
      obstacle_est = percentileValue(valid, 0, near_count, obstacle_percentile_);
      ground_est = percentileValue(valid, split_idx, far_count, ground_percentile_);
    } else {
      ground_est = percentileValue(valid, 0, n, ground_percentile_);
      obstacle_est = percentileValue(valid, 0, n, obstacle_percentile_);
    }

    if (!std::isfinite(ground_est)) {
      ground_est = valid.back();
    }
    if (!std::isfinite(obstacle_est)) {
      obstacle_est = min_range;
    }

    const GeometryResult geometry_result = estimateGeometryObstacle(valid_samples);
    const float ground_for_output =
      (use_geometry_for_ground_height_ && geometry_result.has_ground) ? geometry_result.ground_height_m : ground_est;
    const auto now = this->now();
    const UartHeightMeasurement uart_measurement = readUartHeightMeasurement(now);

    publishFloat(raw_pub_, ground_for_output);
    publishFloat(min_pub_, min_range);

    // 障碍检测：有双峰 或 最小束显著近于"地面"
    const float obstacle_height_raw = std::max(0.0f, ground_est - obstacle_est);
    const float bias_m = static_cast<float>(obstacle_height_bias_cm_) / 100.0f;
    const float legacy_obstacle_height = std::max(0.0f, obstacle_height_raw - bias_m);
    const bool legacy_obstacle = bimodal ||
          legacy_obstacle_height > static_cast<float>(obstacle_margin_);
    const float geometry_obstacle_height = geometry_result.valid
      ? std::max(0.0f, geometry_result.obstacle_height_m - bias_m)
      : 0.0f;
    const float ground_reference_m = geometry_result.has_ground ? geometry_result.ground_height_m : ground_for_output;
    const float uart_block_margin_m = static_cast<float>(uart_block_margin_cm_ / 100.0);
    const float uart_consistency_m = static_cast<float>(uart_geometry_consistency_cm_ / 100.0);
    const float uart_candidate_obstacle_raw = uart_measurement.fresh
      ? std::max(0.0f, ground_reference_m - uart_measurement.corrected_height_m)
      : 0.0f;
    const bool uart_blocked = uart_measurement.fresh &&
      uart_candidate_obstacle_raw > std::max(static_cast<float>(obstacle_margin_), uart_block_margin_m);
    const float uart_candidate_obstacle_height = uart_blocked
      ? std::max(0.0f, uart_candidate_obstacle_raw - bias_m)
      : 0.0f;

    ObstacleFusionMode fusion_mode = ObstacleFusionMode::kNone;
    float published_obstacle_height = 0.0f;
    bool obstacle = false;

    if (geometry_result.valid) {
      obstacle = true;
      published_obstacle_height = geometry_obstacle_height;
      fusion_mode = ObstacleFusionMode::kGeometry;

      const bool uart_can_raise_geometry =
        uart_blocked &&
        uart_candidate_obstacle_height > geometry_obstacle_height &&
        (uart_candidate_obstacle_height - geometry_obstacle_height) <= uart_consistency_m;
      if (uart_can_raise_geometry) {
        published_obstacle_height =
          static_cast<float>((1.0 - uart_blend_weight_) * geometry_obstacle_height +
                             uart_blend_weight_ * uart_candidate_obstacle_height);
        fusion_mode = ObstacleFusionMode::kGeometryUartBlend;
      }
    } else if (legacy_obstacle) {
      obstacle = true;
      published_obstacle_height = legacy_obstacle_height;
      fusion_mode = ObstacleFusionMode::kLegacy;

      const bool uart_can_raise_legacy =
        uart_blocked &&
        uart_candidate_obstacle_height > legacy_obstacle_height &&
        (uart_candidate_obstacle_height - legacy_obstacle_height) <= uart_consistency_m;
      if (uart_can_raise_legacy) {
        published_obstacle_height =
          static_cast<float>((1.0 - uart_blend_weight_) * legacy_obstacle_height +
                             uart_blend_weight_ * uart_candidate_obstacle_height);
        fusion_mode = ObstacleFusionMode::kLegacyUartBlend;
      }
    }

    std_msgs::msg::Bool ob; ob.data = obstacle; obstacle_pub_->publish(ob);

    // EMA low-pass for obstacle_height (same pattern as ground_height below)
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      float filtered = published_obstacle_height;
      if (have_last_) {
        filtered = static_cast<float>(obstacle_ema_alpha_) * published_obstacle_height +
                   static_cast<float>(1.0 - obstacle_ema_alpha_) * last_obstacle_height_;
      }
      last_obstacle_height_ = filtered;
      publishFloat(obstacle_height_pub_, filtered);
    }

    // --- 时间滤波 (ground_height) ---
    float output;
    int hold_ctr;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_last_) {
        output = ground_for_output;
        hold_counter_ = 0;
        have_last_ = true;
      } else {
        double dt = (now - last_stamp_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;

        float delta = ground_for_output - last_output_;

        // 关键场景：下方被柱子完全遮挡（所有束都短、无远簇）、
        // 且 raw 比上一帧输出低很多 —— 显然不是真的掉下去，保持上一帧
        const bool suspicious_drop =
            !bimodal && delta < -static_cast<float>(jump_threshold_);

        if (suspicious_drop && hold_counter_ < max_hold_frames_) {
          output = last_output_;            // 冻结
          hold_counter_++;
        } else {
          // 正常分支：斜率限制 + 一阶低通
          const float max_delta = static_cast<float>(max_slew_rate_ * dt);
          if (delta > max_delta) delta = max_delta;
          else if (delta < -max_delta) delta = -max_delta;
          const float slew = last_output_ + delta;
          output = static_cast<float>(ema_alpha_) * slew +
                   static_cast<float>(1.0 - ema_alpha_) * last_output_;
          hold_counter_ = 0;
        }
      }

      last_output_ = output;
      last_raw_ = ground_for_output;
      last_min_ = min_range;
      last_valid_count_ = static_cast<int>(n);
      last_bimodal_ = bimodal;
      last_obstacle_ = obstacle;
      last_geometry_ground_ = geometry_result.has_ground ? geometry_result.ground_height_m : 0.0f;
      last_geometry_support_ = geometry_result.support_count;
      last_geometry_valid_ = geometry_result.valid;
      last_uart_height_corrected_ = uart_measurement.fresh ? uart_measurement.corrected_height_m : 0.0f;
      last_uart_candidate_obstacle_height_ = uart_candidate_obstacle_height;
      last_uart_fresh_ = uart_measurement.fresh;
      last_uart_blocked_ = uart_blocked;
      last_fusion_mode_ = fusion_mode;
      last_hold_counter_ = hold_counter_;
      last_stamp_ = now;
      hold_ctr = hold_counter_;
    }
    (void)hold_ctr;
    publishFloat(ground_pub_, output);
  }

  void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub,
                    float v) {
    std_msgs::msg::Float32 m;
    m.data = v;
    pub->publish(m);
  }
};

}  // namespace laser_array_ground

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<laser_array_ground::GroundHeightNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "ground_node fatal: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
