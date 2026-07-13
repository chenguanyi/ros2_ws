#include "pid_control_pkg/pid_controller.hpp"

#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace pid_control_pkg
{

PIDController::PIDController(
  double kp,
  double ki,
  double kd,
  double max_output,
  double min_output,
  double integral_limit,
  double deadzone)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  min_output_(min_output),
  integral_limit_(integral_limit),
  deadzone_(deadzone),
  prev_error_(0.0),
  current_error_(0.0),
  integral_(0.0),
  prev_derivative_(0.0),
  first_call_(true),
  derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
  current_error_ = setpoint - measured_value;

  if (std::fabs(current_error_) < deadzone_) {
    current_error_ = 0.0;
  }

  if (first_call_) {
    prev_error_ = current_error_;
    first_call_ = false;
  }

  const double proportional = kp_ * current_error_;

  integral_ += current_error_ * dt;
  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }
  const double integral_term = ki_ * integral_;

  const double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
  const double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ +
    (1.0 - derivative_filter_alpha_) * derivative_raw;
  const double derivative_term = kd_ * derivative_filtered;

  double output = proportional + integral_term + derivative_term;

  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  if ((output >= max_output_ && current_error_ > 0.0) ||
    (output <= min_output_ && current_error_ < 0.0))
  {
    integral_ -= current_error_ * dt;
  }

  prev_error_ = current_error_;
  prev_derivative_ = derivative_filtered;

  return output;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  current_error_ = 0.0;
  integral_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
  max_output_ = max_output;
  min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
  integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}

PositionPIDController::PositionPIDController()
: rclcpp::Node("position_pid_controller"),
  pid_yaw_(1.0, 0.0, 0.2, 30.0, -30.0, 2.0, 0.5),
  pid_z_(1.0, 0.0, 0.2, 25.0, -60.0, 3.0, 0.6),
  pid_xy_speed_(0.8, 0.0, 0.2, 36.0, -36.0, 5.0, 0.6),
  pid_visual_x_(0.08, 0.0, 0.01, 20.0, -20.0, 500.0, 5.0),
  pid_visual_y_(0.08, 0.0, 0.01, 20.0, -20.0, 500.0, 5.0),
  pid_visual_z_(0.06, 0.0, 0.01, 18.0, -18.0, 500.0, 5.0),
  target_x_cm_(0.0),
  target_y_cm_(0.0),
  target_z_cm_(0.0),
  target_yaw_deg_(0.0),
  has_target_position_(false),
  has_target_height_(false),
  current_x_cm_(0.0),
  current_y_cm_(0.0),
  current_yaw_deg_(0.0),
  current_z_cm_(0.0),
  control_frequency_(50.0),
  map_frame_("map"),
  laser_link_frame_("laser_link"),
  max_linear_vel_(36.0),
  max_angular_vel_(30.0),
  max_vertical_vel_(30.0),
  visual_kp_x_(0.08),
  visual_ki_x_(0.0),
  visual_kd_x_(0.01),
  visual_kp_y_(0.08),
  visual_ki_y_(0.0),
  visual_kd_y_(0.01),
  visual_kp_z_(0.06),
  visual_ki_z_(0.0),
  visual_kd_z_(0.01),
  visual_pixel_deadzone_(5.0),
  visual_max_xy_velocity_(20.0),
  visual_max_z_velocity_(18.0),
  visual_data_timeout_sec_(0.5),
  visual_target_offset_x_px_(0.0),
  visual_target_offset_y_px_(0.0),
  visual_mapping_mode_("legacy_xy"),
  distance_xy_cm_(0.0),
  error_x_cm_(0.0),
  error_y_cm_(0.0),
  error_yaw_deg_(0.0),
  error_z_cm_(0.0),
  visual_takeover_active_(false),
  has_visual_fine_data_(false),
  visual_error_x_px_(0.0),
  visual_error_y_px_(0.0),
  control_csv_ready_(false),
  last_update_time_(now())
{
  loadParameters();
  openControlCsv();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  target_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", durable_qos,
    std::bind(&PositionPIDController::targetPositionCallback, this, std::placeholders::_1));
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PositionPIDController::heightCallback, this, std::placeholders::_1));

  auto takeover_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  visual_takeover_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/visual_takeover_active", takeover_qos,
    std::bind(&PositionPIDController::visualTakeoverCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data", rclcpp::QoS(10),
    std::bind(&PositionPIDController::fineDataCallback, this, std::placeholders::_1));

  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", rclcpp::QoS(10));

  const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PositionPIDController::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Position PID Controller initialized (%.1f Hz)", control_frequency_);
  RCLCPP_DEBUG(get_logger(), "Frames: map=%s, laser_link=%s", map_frame_.c_str(), laser_link_frame_.c_str());
  RCLCPP_DEBUG(get_logger(), "Height source: /height");
}

void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "Target position message requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
    return;
  }

  target_x_cm_ = static_cast<double>(msg->data[0]);
  target_y_cm_ = static_cast<double>(msg->data[1]);
  target_z_cm_ = static_cast<double>(msg->data[2]);
  target_yaw_deg_ = static_cast<double>(msg->data[3]);
  has_target_position_ = true;

  RCLCPP_DEBUG(get_logger(),
    "Received target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
    target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
}

void PositionPIDController::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_z_cm_ = static_cast<double>(msg->data);
  has_target_height_ = true;
}

void PositionPIDController::visualTakeoverCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (visual_takeover_active_ == msg->data) {
    return;
  }

  visual_takeover_active_ = msg->data;
  pid_visual_x_.reset();
  pid_visual_y_.reset();
  pid_visual_z_.reset();

  RCLCPP_INFO(
    get_logger(),
    "Visual takeover mode changed: %s",
    visual_takeover_active_ ? "active" : "inactive");
}

void PositionPIDController::fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/fine_data requires 2 values [x_px, y_px]");
    return;
  }

  visual_error_x_px_ = static_cast<double>(msg->data[0]) - visual_target_offset_x_px_;
  visual_error_y_px_ = static_cast<double>(msg->data[1]) - visual_target_offset_y_px_;
  has_visual_fine_data_ = true;
  last_visual_data_time_ = now();
}

bool PositionPIDController::getCurrentPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);

    current_x_cm_ = meterToCm(transform.transform.translation.x);
    current_y_cm_ = meterToCm(transform.transform.translation.y);

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_deg_ = radToDeg(yaw);

    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to lookup transform %s -> %s: %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

bool PositionPIDController::hasFreshVisualData(const rclcpp::Time & now_time) const
{
  if (!has_visual_fine_data_ || last_visual_data_time_.nanoseconds() == 0) {
    return false;
  }
  return (now_time - last_visual_data_time_).seconds() <= visual_data_timeout_sec_;
}

void PositionPIDController::openControlCsv()
{
  const char * ros_log_dir = std::getenv("ROS_LOG_DIR");
  const std::string log_dir = (ros_log_dir != nullptr && ros_log_dir[0] != '\0') ? ros_log_dir : ".";

  std::ostringstream path;
  path << log_dir << "/plant_control_" << now().nanoseconds() << ".csv";

  control_csv_.open(path.str(), std::ios::out | std::ios::trunc);
  if (!control_csv_.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open control CSV log: %s", path.str().c_str());
    control_csv_ready_ = false;
    return;
  }

  control_csv_ready_ = true;
  control_csv_ <<
    "time_sec,dt_sec,"
    "target_x_cm,target_y_cm,target_z_cm,target_yaw_deg,"
    "current_x_cm,current_y_cm,current_z_cm,current_yaw_deg,"
    "error_x_cm,error_y_cm,error_xy_cm,error_z_cm,error_yaw_deg,"
    "cmd_x_cm_s,cmd_y_cm_s,cmd_z_cm_s,cmd_yaw_deg_s,"
    "visual_takeover\n";
  control_csv_.flush();
  RCLCPP_INFO(get_logger(), "Control CSV log: %s", path.str().c_str());
}

void PositionPIDController::logControlSample(
  const rclcpp::Time & stamp,
  double dt,
  const std_msgs::msg::Float32MultiArray & cmd_vel)
{
  if (!control_csv_ready_) {
    return;
  }

  const double cmd_x = cmd_vel.data.size() > 0 ? static_cast<double>(cmd_vel.data[0]) : 0.0;
  const double cmd_y = cmd_vel.data.size() > 1 ? static_cast<double>(cmd_vel.data[1]) : 0.0;
  const double cmd_z = cmd_vel.data.size() > 2 ? static_cast<double>(cmd_vel.data[2]) : 0.0;
  const double cmd_yaw = cmd_vel.data.size() > 3 ? static_cast<double>(cmd_vel.data[3]) : 0.0;

  control_csv_ << std::fixed << std::setprecision(6)
               << stamp.seconds() << ','
               << dt << ','
               << target_x_cm_ << ','
               << target_y_cm_ << ','
               << target_z_cm_ << ','
               << target_yaw_deg_ << ','
               << current_x_cm_ << ','
               << current_y_cm_ << ','
               << current_z_cm_ << ','
               << current_yaw_deg_ << ','
               << error_x_cm_ << ','
               << error_y_cm_ << ','
               << distance_xy_cm_ << ','
               << error_z_cm_ << ','
               << error_yaw_deg_ << ','
               << cmd_x << ','
               << cmd_y << ','
               << cmd_z << ','
               << cmd_yaw << ','
               << (visual_takeover_active_ ? 1 : 0)
               << '\n';
}

double PositionPIDController::normalizeAngleDeg(double angle_deg) const
{
  double result = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(result);
}

void PositionPIDController::calculateErrors()
{
  error_x_cm_ = target_x_cm_ - current_x_cm_;
  error_y_cm_ = target_y_cm_ - current_y_cm_;
  distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
  error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
  if (has_target_height_) {
    error_z_cm_ = target_z_cm_ - current_z_cm_;
  } else {
    error_z_cm_ = 0.0;
  }
}

std_msgs::msg::Float32MultiArray PositionPIDController::processPID(double dt)
{
  std_msgs::msg::Float32MultiArray cmd;
  cmd.data.resize(4);

  calculateErrors();

  double vel_x_cm = 0.0;
  double vel_y_cm = 0.0;
  double vel_z_cm = 0.0;
  bool velocities_are_in_body_frame = false;
  bool z_velocity_from_visual = false;

  if (visual_takeover_active_) {
    const rclcpp::Time now_time = now();
    if (hasFreshVisualData(now_time)) {
      if (visual_mapping_mode_ == "right_side_camera") {
        vel_x_cm = pid_visual_x_.calculate(0.0, visual_error_x_px_, dt);
        vel_y_cm = 0.0;
        vel_z_cm = pid_visual_z_.calculate(0.0, visual_error_y_px_, dt);
        z_velocity_from_visual = true;
      } else {
        /* legacy_xy 模式坐标约定（当前底部相机安装方向）：
         *
         * 当前摄像头安装方向下，视觉误差轴与机体速度轴交换：
         *   vel_x_cm 使用 visual_error_y_px_
         *   vel_y_cm 使用 visual_error_x_px_
         *
         * pid_visual_x_ / pid_visual_y_ 参数当前相同；关键是速度轴使用哪个视觉误差。
         * 对 X/Y 都取负号，以匹配 setpoint=0 时的速度修正方向。
         */
        vel_x_cm = pid_visual_y_.calculate(0.0, -visual_error_y_px_, dt);
        vel_y_cm = pid_visual_x_.calculate(0.0, visual_error_x_px_, dt);
      }
      velocities_are_in_body_frame = true;
    } else {
      vel_x_cm = 0.0;
      vel_y_cm = 0.0;
      if (visual_mapping_mode_ == "right_side_camera") {
        vel_z_cm = 0.0;
        z_velocity_from_visual = true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Visual takeover active but /fine_data is stale. Holding visual velocity at zero.");
    }
  } else {
    if (distance_xy_cm_ > 0.1) {
      double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
      if (speed_cmd < 0.0) {
        speed_cmd = 0.0;
      }
      const double cos_theta = error_x_cm_ / distance_xy_cm_;
      const double sin_theta = error_y_cm_ / distance_xy_cm_;
      vel_x_cm = speed_cmd * cos_theta;
      vel_y_cm = speed_cmd * sin_theta;
    } else {
      vel_x_cm = 0.0;
      vel_y_cm = 0.0;
    }
  }

  if (!velocities_are_in_body_frame) {
    const double yaw_rad = angles::from_degrees(current_yaw_deg_);
    const double vel_x_map_cm = vel_x_cm;
    const double vel_y_map_cm = vel_y_cm;

    vel_x_cm = std::cos(yaw_rad) * vel_x_map_cm + std::sin(yaw_rad) * vel_y_map_cm;
    vel_y_cm = -std::sin(yaw_rad) * vel_x_map_cm + std::cos(yaw_rad) * vel_y_map_cm;
  }

  const double vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);

  if (z_velocity_from_visual) {
    // The right-side camera mapping owns Z while visual takeover is active.
  } else if (has_target_height_) {
    vel_z_cm = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  } else {
    if (std::fabs(target_z_cm_) > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for /height... Z velocity suppressed (Target Z=%.1f)",
        target_z_cm_);
    }
    vel_z_cm = 0.0;
  }

  cmd.data[0] = static_cast<float>(vel_x_cm);
  cmd.data[1] = static_cast<float>(vel_y_cm);
  cmd.data[2] = static_cast<float>(vel_z_cm);
  cmd.data[3] = static_cast<float>(vel_yaw_deg);
  return cmd;
}

void PositionPIDController::controlTimerCallback()
{
  if (!has_target_position_) {
    return;
  }

  if (!getCurrentPose()) {
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / std::max(control_frequency_, 1.0);
  }
  last_update_time_ = now_time;

  auto cmd_vel = processPID(dt);
  target_velocity_pub_->publish(cmd_vel);
  logControlSample(now_time, dt, cmd_vel);

  if (visual_takeover_active_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Visual mode: fine_data=(%.1f, %.1f) target_velocity=(%.1f, %.1f, %.1f, %.1f)",
      visual_error_x_px_,
      visual_error_y_px_,
      cmd_vel.data[0],
      cmd_vel.data[1],
      cmd_vel.data[2],
      cmd_vel.data[3]);
  }
}

void PositionPIDController::loadParameters()
{
  control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");

  const double kp_xy = declare_parameter<double>("kp_xy", 0.8);
  const double ki_xy = declare_parameter<double>("ki_xy", 0.0);
  const double kd_xy = declare_parameter<double>("kd_xy", 0.2);

  const double kp_yaw = declare_parameter<double>("kp_yaw", 1.0);
  const double ki_yaw = declare_parameter<double>("ki_yaw", 0.0);
  const double kd_yaw = declare_parameter<double>("kd_yaw", 0.2);

  const double kp_z = declare_parameter<double>("kp_z", 1.0);
  const double ki_z = declare_parameter<double>("ki_z", 0.0);
  const double kd_z = declare_parameter<double>("kd_z", 0.2);

  max_linear_vel_ = declare_parameter<double>("max_linear_velocity", 33.0);
  max_angular_vel_ = declare_parameter<double>("max_angular_velocity", 30.0);
  max_vertical_vel_ = declare_parameter<double>("max_vertical_velocity", 30.0);

  visual_kp_x_ = declare_parameter<double>("visual_kp_x", 0.08);
  visual_ki_x_ = declare_parameter<double>("visual_ki_x", 0.0);
  visual_kd_x_ = declare_parameter<double>("visual_kd_x", 0.01);
  visual_kp_y_ = declare_parameter<double>("visual_kp_y", 0.08);
  visual_ki_y_ = declare_parameter<double>("visual_ki_y", 0.0);
  visual_kd_y_ = declare_parameter<double>("visual_kd_y", 0.01);
  visual_kp_z_ = declare_parameter<double>("visual_kp_z", 0.06);
  visual_ki_z_ = declare_parameter<double>("visual_ki_z", 0.0);
  visual_kd_z_ = declare_parameter<double>("visual_kd_z", 0.01);
  visual_pixel_deadzone_ = declare_parameter<double>("visual_pixel_deadzone", 5.0);
  visual_max_xy_velocity_ = declare_parameter<double>("visual_max_xy_velocity", 20.0);
  visual_max_z_velocity_ = declare_parameter<double>("visual_max_z_velocity", 18.0);
  visual_data_timeout_sec_ = declare_parameter<double>("visual_data_timeout_sec", 0.5);
  visual_target_offset_x_px_ = declare_parameter<double>("visual_target_offset_x_px", 0.0);
  visual_target_offset_y_px_ = declare_parameter<double>("visual_target_offset_y_px", 0.0);
  visual_mapping_mode_ = declare_parameter<std::string>("visual_mapping_mode", "legacy_xy");
  if (visual_mapping_mode_ != "legacy_xy" && visual_mapping_mode_ != "right_side_camera") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown visual_mapping_mode='%s'. Falling back to legacy_xy.",
      visual_mapping_mode_.c_str());
    visual_mapping_mode_ = "legacy_xy";
  }

  pid_yaw_.setPID(kp_yaw, ki_yaw, kd_yaw);
  pid_z_.setPID(kp_z, ki_z, kd_z);
  pid_xy_speed_.setPID(kp_xy, ki_xy, kd_xy);

  pid_yaw_.setOutputLimits(max_angular_vel_, -max_angular_vel_);
  pid_z_.setOutputLimits(max_vertical_vel_, -60.0);
  pid_xy_speed_.setOutputLimits(max_linear_vel_, -max_linear_vel_);

  pid_visual_x_.setPID(visual_kp_x_, visual_ki_x_, visual_kd_x_);
  pid_visual_y_.setPID(visual_kp_y_, visual_ki_y_, visual_kd_y_);
  pid_visual_z_.setPID(visual_kp_z_, visual_ki_z_, visual_kd_z_);
  pid_visual_x_.setOutputLimits(visual_max_xy_velocity_, -visual_max_xy_velocity_);
  pid_visual_y_.setOutputLimits(visual_max_xy_velocity_, -visual_max_xy_velocity_);
  pid_visual_z_.setOutputLimits(visual_max_z_velocity_, -visual_max_z_velocity_);
  pid_visual_x_.setDeadzone(visual_pixel_deadzone_);
  pid_visual_y_.setDeadzone(visual_pixel_deadzone_);
  pid_visual_z_.setDeadzone(visual_pixel_deadzone_);

  RCLCPP_DEBUG(get_logger(),
    "PID params: XY(kp=%.2f, ki=%.2f, kd=%.2f) Yaw(kp=%.2f, ki=%.2f, kd=%.2f) Z(kp=%.2f, ki=%.2f, kd=%.2f)",
    kp_xy, ki_xy, kd_xy, kp_yaw, ki_yaw, kd_yaw, kp_z, ki_z, kd_z);
  RCLCPP_DEBUG(get_logger(),
    "Visual PID params: mode=%s X(kp=%.3f, ki=%.3f, kd=%.3f) Y(kp=%.3f, ki=%.3f, kd=%.3f) Z(kp=%.3f, ki=%.3f, kd=%.3f) deadzone=%.1f max_xy=%.1f max_z=%.1f stale=%.1fs offset=(%.1f, %.1f)",
    visual_mapping_mode_.c_str(),
    visual_kp_x_, visual_ki_x_, visual_kd_x_,
    visual_kp_y_, visual_ki_y_, visual_kd_y_,
    visual_kp_z_, visual_ki_z_, visual_kd_z_,
    visual_pixel_deadzone_, visual_max_xy_velocity_, visual_max_z_velocity_, visual_data_timeout_sec_,
    visual_target_offset_x_px_, visual_target_offset_y_px_);
  RCLCPP_DEBUG(get_logger(),
    "Velocity limits: linear=%.1fcm/s angular=%.1fdeg/s vertical=%.1fcm/s",
    max_linear_vel_, max_angular_vel_, max_vertical_vel_);
}

}  // 命名空间 pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
