from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    visual_kp_x = LaunchConfiguration("visual_kp_x")
    visual_kd_x = LaunchConfiguration("visual_kd_x")
    visual_kp_y = LaunchConfiguration("visual_kp_y")
    visual_kd_y = LaunchConfiguration("visual_kd_y")
    visual_target_offset_x_px = LaunchConfiguration("visual_target_offset_x_px")
    visual_target_offset_y_px = LaunchConfiguration("visual_target_offset_y_px")
    visual_pixel_deadzone = LaunchConfiguration("visual_pixel_deadzone")
    visual_max_xy_velocity = LaunchConfiguration("visual_max_xy_velocity")
    visual_data_timeout_sec = LaunchConfiguration("visual_data_timeout_sec")

    pid_params = {
        # 控制循环与 TF 坐标系
        "control_frequency": 50.0,         # PID 更新频率，单位 Hz
        "map_frame": "map",                # 全局目标坐标系
        "laser_link_frame": "laser_link",  # PID 使用的机体位姿坐标系

        # 常规位置控制：XY 位置误差 -> XY 速度指令
        "kp_xy": 0.65,                      # XY 比例增益
        "ki_xy": 0.0,                      # XY 积分增益
        "kd_xy": 0.05,                      # XY 微分增益

        # 偏航控制：偏航误差 -> 偏航角速度指令
        "kp_yaw": 0.65,                     # 偏航比例增益
        "ki_yaw": 0.0,                     # 偏航积分增益
        "kd_yaw": 0.06,                     # 偏航微分增益

        # 高度控制：高度误差 -> Z 方向速度指令
        "kp_z": 0.9,                       # 高度比例增益
        "ki_z": 0.0,                       # 高度积分增益
        "kd_z": 0.07,                       # 高度微分增益

        # 输出限幅
        "max_linear_velocity": 28.0,       # XY 最大速度，单位 cm/s
        "max_angular_velocity": 25.0,      # 最大偏航角速度，单位 deg/s
        "max_vertical_velocity": 26.0,     # Z 方向最大速度，单位 cm/s

        # 视觉接管控制：像素误差 -> XY 速度指令
        "visual_kp_x": visual_kp_x,        # 视觉 X 比例增益
        "visual_ki_x": 0.0,                # 视觉 X 积分增益
        "visual_kd_x": visual_kd_x,        # 视觉 X 微分增益
        "visual_kp_y": visual_kp_y,        # 视觉 Y 比例增益
        "visual_ki_y": 0.0,                # 视觉 Y 积分增益
        "visual_kd_y": visual_kd_y,        # 视觉 Y 微分增益
        "visual_target_offset_x_px": visual_target_offset_x_px,
        "visual_target_offset_y_px": visual_target_offset_y_px,
        "visual_pixel_deadzone": visual_pixel_deadzone,   # 像素死区
        "visual_max_xy_velocity": visual_max_xy_velocity, # 接管期间 XY 最大速度，单位 cm/s
        "visual_data_timeout_sec": visual_data_timeout_sec,  # fine_data 超时后将 XY 保持为 0，单位秒
    }

    return LaunchDescription([
        DeclareLaunchArgument("visual_kp_x", default_value="0.08", description="[视觉PID] X 方向视觉比例增益。"),
        DeclareLaunchArgument("visual_kd_x", default_value="0.01", description="[视觉PID] X 方向视觉微分增益。"),
        DeclareLaunchArgument("visual_kp_y", default_value="0.08", description="[视觉PID] Y 方向视觉比例增益。"),
        DeclareLaunchArgument("visual_kd_y", default_value="0.01", description="[视觉PID] Y 方向视觉微分增益。"),
        DeclareLaunchArgument("visual_target_offset_x_px", default_value="0.0", description="[视觉PID] 目标对准 X 偏移，单位 px，控制器会将 fine_data.x 减去该值。"),
        DeclareLaunchArgument("visual_target_offset_y_px", default_value="0.0", description="[视觉PID] 目标对准 Y 偏移，单位 px，控制器会将 fine_data.y 减去该值。"),
        DeclareLaunchArgument("visual_pixel_deadzone", default_value="5.0", description="[视觉PID] 像素死区。"),
        DeclareLaunchArgument("visual_max_xy_velocity", default_value="20.0", description="[视觉PID] 视觉接管时 XY 最大速度，单位 cm/s。"),
        DeclareLaunchArgument("visual_data_timeout_sec", default_value="0.5", description="[视觉PID] fine_data 超时阈值，单位 s。"),
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[pid_params],
        )
    ])
