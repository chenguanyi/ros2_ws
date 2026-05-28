#include "uart_to_stm32/uart_to_stm32.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  update_rate_(0.0),
  current_yaw_(0.0),
  yaw_valid_(false),
  velocity_valid_(false),
  route_task_active_(false),
  has_st_ready_pub_(false),
  forward_height_0x05_(false),
  height_source_("serial_raw"),
  laser_height_topic_("/laser_array/ground_height"),
  cruise_height_cm_(130),
  height_band_cm_(20),
  min_valid_height_cm_(0),
  max_valid_height_cm_(500)
{
  RCLCPP_DEBUG(node_->get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
  if (timer_) {
    timer_->cancel();
  }
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize(double update_rate, const std::string & source_frame, const std::string & target_frame)
{
  try {
    update_rate_ = update_rate;
    source_frame_ = source_frame;
    target_frame_ = target_frame;

    RCLCPP_DEBUG(node_->get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
    RCLCPP_DEBUG(
      node_->get_logger(), "Looking for transform from '%s' to '%s'",
      source_frame_.c_str(), target_frame_.c_str());

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize("/dev/ttyS6", 921600)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial port /dev/ttyS6 at 921600 baudrate");
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Serial port /dev/ttyS6 initialized at 921600 baudrate");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = node_->create_wall_timer(period, std::bind(&UartToStm32::lookupTransform, this));

    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/velocity_map", 10,
      std::bind(&UartToStm32::velocityCallback, this, std::placeholders::_1));

    const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    active_controller_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/active_controller",
      durable_qos,
      std::bind(&UartToStm32::activeControllerCallback, this, std::placeholders::_1));

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));

    mission_complete_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/mission_complete", rclcpp::QoS(10),
      std::bind(&UartToStm32::missionCompleteCallback, this, std::placeholders::_1));

    arm_command_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/arm_command", rclcpp::QoS(10),
      std::bind(&UartToStm32::armCommandCallback, this, std::placeholders::_1));
    magnet_command_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/magnet_command", rclcpp::QoS(10),
      std::bind(&UartToStm32::magnetCommandCallback, this, std::placeholders::_1));
    signal_command_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/signal_command", rclcpp::QoS(10),
      std::bind(&UartToStm32::signalCommandCallback, this, std::placeholders::_1));

    cruise_height_cm_ = static_cast<int>(
      node_->declare_parameter<int>("cruise_height_cm", 130));
    height_band_cm_ = static_cast<int>(
      node_->declare_parameter<int>("height_band_cm", 20));
    min_valid_height_cm_ = static_cast<int>(
      node_->declare_parameter<int>("min_valid_height_cm", 0));
    max_valid_height_cm_ = static_cast<int>(
      node_->declare_parameter<int>("max_valid_height_cm", 500));
    height_source_ = node_->declare_parameter<std::string>("height_source", "serial_raw");
    laser_height_topic_ = node_->declare_parameter<std::string>(
      "laser_height_topic", "/laser_array/ground_height");
    const bool legacy_forward_raw_height_0x05 =
      node_->declare_parameter<bool>("forward_raw_height_0x05", false);
    forward_height_0x05_ =
      node_->declare_parameter<bool>("forward_height_0x05", legacy_forward_raw_height_0x05);

    if (height_source_ != "serial_raw" && height_source_ != "laser_ground") {
      RCLCPP_WARN(
        node_->get_logger(),
        "Unknown height_source='%s'. Falling back to 'serial_raw'.",
        height_source_.c_str());
      height_source_ = "serial_raw";
    }

    if (legacy_forward_raw_height_0x05) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Parameter 'forward_raw_height_0x05' is deprecated. Use 'forward_height_0x05' instead.");
    }

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    on_pillar_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/on_pillar", latched_qos,
      std::bind(&UartToStm32::onPillarCallback, this, std::placeholders::_1));
    height_filter_enabled_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/height_filter_enabled", latched_qos,
      std::bind(&UartToStm32::heightFilterEnabledCallback, this, std::placeholders::_1));

    if (height_source_ == "laser_ground") {
      laser_ground_height_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        laser_height_topic_,
        rclcpp::QoS(10),
        std::bind(&UartToStm32::laserGroundHeightCallback, this, std::placeholders::_1));
    }

    pillar_signal_timer_ = node_->create_wall_timer(
      1s, std::bind(&UartToStm32::pillarSignalTimerCallback, this));

    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    height_raw_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height_raw", 10);
    is_st_ready_pub_ =
      node_->create_publisher<std_msgs::msg::UInt8>("/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);

    set_arm_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/debug/set_arm",
      std::bind(
        &UartToStm32::setArmServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
    set_magnet_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/debug/set_magnet",
      std::bind(
        &UartToStm32::setMagnetServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
    set_signal_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/debug/set_signal",
      std::bind(
        &UartToStm32::setSignalServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(
      node_->get_logger(),
      "Height source=%s laser_topic=%s forward_0x05=%s",
      height_source_.c_str(),
      laser_height_topic_.c_str(),
      forward_height_0x05_ ? "on" : "off");
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Subscribed to /velocity_map, /active_controller, /target_velocity, /mission_complete, and aux command topics");
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize topic subscriber: %s", e.what());
    return false;
  }
}

void UartToStm32::lookupTransform()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);
    processTfTransform(transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

void UartToStm32::processTfTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;
  const double z = transform.transform.translation.z;

  const double qx = transform.transform.rotation.x;
  const double qy = transform.transform.rotation.y;
  const double qz = transform.transform.rotation.z;
  const double qw = transform.transform.rotation.w;

  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
  yaw_valid_ = true;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)",
    source_frame_.c_str(), target_frame_.c_str(), x, y, z, roll, pitch, yaw);
}

void UartToStm32::activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 2) {
    if (!route_task_active_) {
      route_task_active_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "Received /active_controller=2. Target velocity forwarding to STM32 is enabled.");
    }
    return;
  }

  if (msg->data == 3) {
    if (route_task_active_) {
      route_task_active_ = false;
      RCLCPP_INFO(
        node_->get_logger(),
        "Received /active_controller=3. Target velocity forwarding to STM32 is disabled.");
    }
    return;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Ignoring /active_controller=%u. Target velocity forwarding stays %s.",
    static_cast<unsigned>(msg->data),
    route_task_active_ ? "enabled" : "disabled");
}

void UartToStm32::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_velocity_ = *msg;
  velocity_valid_ = true;

  const double linear_x = msg->linear.x;
  const double linear_y = msg->linear.y;
  const double linear_z = msg->linear.z;
  const double angular_x = msg->angular.x;
  const double angular_y = msg->angular.y;
  const double angular_z = msg->angular.z;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f)",
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);

  if (yaw_valid_ && velocity_valid_) {
    const Eigen::Vector3d linear_vel(linear_x, linear_y, linear_z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}

Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d & linear, double yaw)
{
  Eigen::Matrix3d rz;
  rz << std::cos(yaw), std::sin(yaw), 0.0,
    -std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d transformed = rz * linear;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity transform: yaw=%.3f deg, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    yaw * 180.0 / M_PI,
    linear.x(), linear.y(), linear.z(),
    transformed.x(), transformed.y(), transformed.z());

  return transformed;
}

void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send velocity data");
    return;
  }

  try {
    constexpr double scale_factor = 100.0;

    const int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
    const int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
    const int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);

    std::vector<uint8_t> data(6);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (!route_task_active_) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Dropping /target_velocity because /active_controller is not in mission mode.");
    return;
  }

  if (msg->data.size() < 4) {
    RCLCPP_WARN(node_->get_logger(),
      "Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
    return;
  }

  const float vx_cm_per_s = msg->data[0];
  const float vy_cm_per_s = msg->data[1];
  const float vz_cm_per_s = msg->data[2];
  const float vyaw_deg_per_s = msg->data[3];

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

void UartToStm32::sendTargetVelocityToSerial(
  float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send target velocity data");
    return;
  }

  try {
    const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
    const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
    const int16_t vel_z = static_cast<int16_t>(std::lround(vz_cm_per_s));
    const int16_t vel_yaw = static_cast<int16_t>(std::lround(vyaw_deg_per_s));

    std::vector<uint8_t> data(8);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(vel_yaw & 0xFF);
    data[7] = static_cast<uint8_t>((vel_yaw >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d",
        vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)
{
  switch (id) {
    case ST_READY_QUERY_ID: {
      if (data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
        break;
      }
      const uint8_t first = data[0];
      if (mission_step_pub_) {
        std_msgs::msg::UInt8 msg;
        msg.data = first;
        mission_step_pub_->publish(msg);
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
      }
      if (has_st_ready_pub_) {
        break;
      }
      const uint8_t second = data[1];
      if (second == 1) {
        if (is_st_ready_pub_) {
          std_msgs::msg::UInt8 msg;
          msg.data = 1;
          is_st_ready_pub_->publish(msg);
          RCLCPP_INFO(node_->get_logger(), "Published /is_st_ready: 1 (from 0xF1 frame)");
        }
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case RAW_HEIGHT_FRAME_ID: {
      if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0x%02X data too short", RAW_HEIGHT_FRAME_ID);
        break;
      }
      const int16_t value = static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
      publishRawHeight(value);
      break;
    }
    case 0xB1: {
      if (data.size() < 8) {
        RCLCPP_WARN(node_->get_logger(),
          "protocolDataHandler: ID 0xB1 data too short (expected 8, got %zu)", data.size());
        break;
      }

      const int16_t vel_x = static_cast<int16_t>(data[0] | (data[1] << 8));
      const int16_t vel_y = static_cast<int16_t>(data[2] | (data[3] << 8));
      const int16_t vel_z = static_cast<int16_t>(data[4] | (data[5] << 8));
      const int16_t yaw = static_cast<int16_t>(data[6] | (data[7] << 8));

      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[0xB1] Target Speed -> X:%d, Y:%d, Z:%d, Yaw:%d",
        vel_x, vel_y, vel_z, yaw);
      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

void UartToStm32::sendMissionCompleteToSerial()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send mission complete data");
    return;
  }

  const std::vector<uint8_t> data(1, MISSION_COMPLETE_VALUE);
  if (serial_comm_->send_protocol_data(MISSION_COMPLETE_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Sent mission complete frame: id=0x%02X value=0x%02X",
      static_cast<unsigned>(MISSION_COMPLETE_FRAME_ID),
      static_cast<unsigned>(MISSION_COMPLETE_VALUE));
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to send mission complete frame: %s",
      serial_comm_->get_last_error().c_str());
  }
}

void UartToStm32::missionCompleteCallback(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Received mission complete event. Sending frame 0x%02X three times.",
    static_cast<unsigned>(MISSION_COMPLETE_FRAME_ID));

  for (int i = 0; i < 3; ++i) {
    sendMissionCompleteToSerial();
    std::this_thread::sleep_for(100ms);
  }

  route_task_active_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "Mission complete sent. Target velocity forwarding is now disabled until /active_controller=2.");
}

void UartToStm32::onPillarCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool new_state = msg->data;
  const bool prev = on_pillar_.exchange(new_state);
  if (prev != new_state) {
    RCLCPP_INFO(
      node_->get_logger(),
      "/on_pillar changed: %s -> %s", prev ? "true" : "false", new_state ? "true" : "false");
  }
}

void UartToStm32::pillarSignalTimerCallback()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    return;
  }

  const uint8_t payload = on_pillar_.load() ? uint8_t{0x01} : uint8_t{0x00};
  const std::vector<uint8_t> data(1, payload);

  if (!serial_comm_->send_protocol_data(PILLAR_SIGNAL_FRAME_ID, 1, data)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to send pillar signal frame 0x22: %s",
      serial_comm_->get_last_error().c_str());
    return;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Sent pillar signal: 0x22/0x%02X", static_cast<unsigned>(payload));
}

void UartToStm32::heightFilterEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool new_state = msg->data;
  const bool prev = height_filter_enabled_.exchange(new_state);
  if (prev != new_state) {
    RCLCPP_INFO(
      node_->get_logger(),
      "/height_filter_enabled changed: %s -> %s",
      prev ? "true" : "false", new_state ? "true" : "false");
  }
}

void UartToStm32::laserGroundHeightCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (height_source_ != "laser_ground") {
    return;
  }

  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Ignoring non-finite laser ground height.");
    return;
  }

  const double height_cm = static_cast<double>(msg->data) * 100.0;
  if (height_cm < static_cast<double>(std::numeric_limits<int16_t>::min()) ||
    height_cm > static_cast<double>(std::numeric_limits<int16_t>::max()))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Ignoring out-of-range laser ground height=%.3fm (%.1fcm).",
      static_cast<double>(msg->data),
      height_cm);
    return;
  }

  publishMissionHeight(static_cast<int16_t>(std::lround(height_cm)), "laser_ground");
}

void UartToStm32::armCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > 1U) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring /arm_command=%u, only 0 or 1 is allowed.", static_cast<unsigned>(msg->data));
    return;
  }
  updateAuxStateAndSend("arm", arm_state_, msg->data);
}

void UartToStm32::magnetCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > 1U) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring /magnet_command=%u, only 0 or 1 is allowed.", static_cast<unsigned>(msg->data));
    return;
  }
  updateAuxStateAndSend("magnet", magnet_state_, msg->data);
}

void UartToStm32::signalCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > 1U) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring /signal_command=%u, only 0 or 1 is allowed.", static_cast<unsigned>(msg->data));
    return;
  }
  updateAuxStateAndSend("signal", signal_state_, msg->data);
}

void UartToStm32::setArmServiceCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  response->success = updateAuxStateAndSend("arm", arm_state_, request->data ? 1U : 0U);
  response->message = response->success ? "arm updated" : "failed to send arm command";
}

void UartToStm32::setMagnetServiceCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  response->success = updateAuxStateAndSend("magnet", magnet_state_, request->data ? 1U : 0U);
  response->message = response->success ? "magnet updated" : "failed to send magnet command";
}

void UartToStm32::setSignalServiceCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  response->success = updateAuxStateAndSend("signal", signal_state_, request->data ? 1U : 0U);
  response->message = response->success ? "signal updated" : "failed to send signal command";
}

bool UartToStm32::updateAuxStateAndSend(const char * name, std::atomic<std::uint8_t> & slot, std::uint8_t value)
{
  slot.store(value);
  const bool ok = sendCombinedAuxControlToSerial();
  if (ok) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Updated %s state=%u and sent combined 0x%02X packet.",
      name,
      static_cast<unsigned>(value),
      static_cast<unsigned>(AUX_CONTROL_FRAME_ID));
  }
  return ok;
}

bool UartToStm32::sendCombinedAuxControlToSerial()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send combined aux control data");
    return false;
  }

  const std::vector<uint8_t> data{
    arm_state_.load(),
    magnet_state_.load(),
    signal_state_.load(),
  };

  const bool ok = serial_comm_->send_protocol_data(
    AUX_CONTROL_FRAME_ID,
    static_cast<uint8_t>(data.size()),
    data);
  if (!ok) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to send combined aux control frame 0x%02X: %s",
      static_cast<unsigned>(AUX_CONTROL_FRAME_ID),
      serial_comm_->get_last_error().c_str());
    return false;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Sent combined aux control frame 0x%02X: [arm=%u, magnet=%u, signal=%u]",
    static_cast<unsigned>(AUX_CONTROL_FRAME_ID),
    static_cast<unsigned>(data[0]),
    static_cast<unsigned>(data[1]),
    static_cast<unsigned>(data[2]));
  return true;
}

bool UartToStm32::sendHeightToSerial(int16_t height_cm)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot forward height data");
    return false;
  }

  std::vector<uint8_t> data(2);
  data[0] = static_cast<uint8_t>(height_cm & 0xFF);
  data[1] = static_cast<uint8_t>((height_cm >> 8) & 0xFF);

  const bool ok = serial_comm_->send_protocol_data(
    RAW_HEIGHT_FRAME_ID,
    static_cast<uint8_t>(data.size()),
    data);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to forward height frame 0x%02X: %s",
      static_cast<unsigned>(RAW_HEIGHT_FRAME_ID),
      serial_comm_->get_last_error().c_str());
    return false;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Forwarded height frame 0x%02X: %d cm",
    static_cast<unsigned>(RAW_HEIGHT_FRAME_ID),
    height_cm);
  return true;
}

bool UartToStm32::isValidFilteredHeight(int16_t raw_value_cm) const
{
  return raw_value_cm >= min_valid_height_cm_ && raw_value_cm <= max_valid_height_cm_;
}

void UartToStm32::publishMissionHeight(int16_t height_cm, const char * source_name)
{
  if (!height_pub_) {
    RCLCPP_WARN(node_->get_logger(), "Height publisher not initialized");
    return;
  }

  if (!isValidFilteredHeight(height_cm)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Dropped invalid %s height=%d cm. Allowed range is [%d, %d] cm.",
      source_name,
      height_cm,
      min_valid_height_cm_,
      max_valid_height_cm_);
    return;
  }

  std_msgs::msg::Int16 msg;
  msg.data = height_cm;
  height_pub_->publish(msg);

  if (forward_height_0x05_) {
    sendHeightToSerial(height_cm);
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Published /height=%d cm from %s, forward_0x05=%s",
    height_cm,
    source_name,
    forward_height_0x05_ ? "on" : "off");
}

void UartToStm32::publishRawHeight(int16_t raw_value_cm)
{
  if (!height_raw_pub_) {
    RCLCPP_WARN(node_->get_logger(), "Height raw publisher not initialized");
    return;
  }

  std_msgs::msg::Int16 raw_msg;
  raw_msg.data = raw_value_cm;
  height_raw_pub_->publish(raw_msg);

  if (height_source_ != "serial_raw") {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Received serial raw height=%d cm, but mission height source is %s. Publishing /height_raw only.",
      raw_value_cm,
      height_source_.c_str());
    return;
  }

  publishMissionHeight(raw_value_cm, "serial_raw");
}

}  // namespace uart_to_stm32
