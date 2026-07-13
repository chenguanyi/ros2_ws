from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    percentile = LaunchConfiguration("percentile")
    ground_percentile = LaunchConfiguration("ground_percentile")
    obstacle_percentile = LaunchConfiguration("obstacle_percentile")
    obstacle_margin = LaunchConfiguration("obstacle_margin")
    obstacle_height_bias_cm = LaunchConfiguration("obstacle_height_bias_cm")
    geometry_obstacle_enabled = LaunchConfiguration("geometry_obstacle_enabled")
    use_geometry_for_ground_height = LaunchConfiguration("use_geometry_for_ground_height")
    beam_rows = LaunchConfiguration("beam_rows")
    beam_cols = LaunchConfiguration("beam_cols")
    beam_horizontal_fov_deg = LaunchConfiguration("beam_horizontal_fov_deg")
    beam_vertical_fov_deg = LaunchConfiguration("beam_vertical_fov_deg")
    flip_rows = LaunchConfiguration("flip_rows")
    flip_cols = LaunchConfiguration("flip_cols")
    beam_mount_roll_deg = LaunchConfiguration("beam_mount_roll_deg")
    beam_mount_pitch_deg = LaunchConfiguration("beam_mount_pitch_deg")
    beam_mount_yaw_deg = LaunchConfiguration("beam_mount_yaw_deg")
    obstacle_top_size_cm = LaunchConfiguration("obstacle_top_size_cm")
    obstacle_center_slack_cm = LaunchConfiguration("obstacle_center_slack_cm")
    geometry_min_support = LaunchConfiguration("geometry_min_support")
    geometry_candidate_margin_cm = LaunchConfiguration("geometry_candidate_margin_cm")
    geometry_height_tolerance_cm = LaunchConfiguration("geometry_height_tolerance_cm")
    uart_height_fusion_enabled = LaunchConfiguration("uart_height_fusion_enabled")
    uart_height_topic = LaunchConfiguration("uart_height_topic")
    uart_height_bias_cm = LaunchConfiguration("uart_height_bias_cm")
    uart_height_timeout_sec = LaunchConfiguration("uart_height_timeout_sec")
    uart_block_margin_cm = LaunchConfiguration("uart_block_margin_cm")
    uart_geometry_consistency_cm = LaunchConfiguration("uart_geometry_consistency_cm")
    uart_blend_weight = LaunchConfiguration("uart_blend_weight")
    obstacle_ema_alpha = LaunchConfiguration("obstacle_ema_alpha")
    log_period_sec = LaunchConfiguration("log_period_sec")

    params = {
        "serial_port": "/dev/ttyS3",
        "baud_rate": 921600,
        # 空间滤波
        "percentile": percentile,  # 无双峰时: 1.0=最大值, 0.8=80分位
        "ground_percentile": ground_percentile,
        "obstacle_percentile": obstacle_percentile,
        "cluster_gap": 0.20,      # m, 相邻束差距 > 此值认为柱子/地面双峰
        # 时间滤波
        "max_slew_rate": 1.5,     # m/s, 高度变化速率上限
        "ema_alpha": 0.6,         # 0~1, 越大越跟手
        "jump_threshold": 0.15,   # m, 降幅超此值 + 无远簇 -> 冻结输出
        "max_hold_frames": 20,    # 最多保持上一帧多少帧 (50Hz下20帧=0.4s)
        # 障碍检测
        "obstacle_margin": obstacle_margin,  # m
        "obstacle_height_bias_cm": obstacle_height_bias_cm,
        # 几何化障碍测高
        "geometry_obstacle_enabled": geometry_obstacle_enabled,
        "use_geometry_for_ground_height": use_geometry_for_ground_height,
        "beam_rows": beam_rows,
        "beam_cols": beam_cols,
        "beam_horizontal_fov_deg": beam_horizontal_fov_deg,
        "beam_vertical_fov_deg": beam_vertical_fov_deg,
        "flip_rows": flip_rows,
        "flip_cols": flip_cols,
        "beam_mount_roll_deg": beam_mount_roll_deg,
        "beam_mount_pitch_deg": beam_mount_pitch_deg,
        "beam_mount_yaw_deg": beam_mount_yaw_deg,
        "obstacle_top_size_cm": obstacle_top_size_cm,
        "obstacle_center_slack_cm": obstacle_center_slack_cm,
        "geometry_min_support": geometry_min_support,
        "geometry_candidate_margin_cm": geometry_candidate_margin_cm,
        "geometry_height_tolerance_cm": geometry_height_tolerance_cm,
        # UART 单点激光融合
        "uart_height_fusion_enabled": uart_height_fusion_enabled,
        "uart_height_topic": uart_height_topic,
        "uart_height_bias_cm": uart_height_bias_cm,
        "uart_height_timeout_sec": uart_height_timeout_sec,
        "uart_block_margin_cm": uart_block_margin_cm,
        "uart_geometry_consistency_cm": uart_geometry_consistency_cm,
        "uart_blend_weight": uart_blend_weight,
        "obstacle_ema_alpha": obstacle_ema_alpha,
        # 日志
        "log_period_sec": log_period_sec,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_period_sec",
            default_value="5.0",
            description="[激光] 高度摘要日志周期，单位秒。",
        ),
        DeclareLaunchArgument(
            "percentile",
            default_value="1.0",
            description="[激光] 地面高度分位值。1.0=最大值，0.8=80分位。",
        ),
        DeclareLaunchArgument(
            "ground_percentile",
            default_value="0.95",
            description="[激光] 地面估计分位值，用于双峰或单峰情况。",
        ),
        DeclareLaunchArgument(
            "obstacle_percentile",
            default_value="0.10",
            description="[激光] 障碍估计分位值，用于双峰或单峰情况。",
        ),
        DeclareLaunchArgument(
            "obstacle_margin",
            default_value="0.20",
            description="[激光] 判定下方有障碍物的高度差阈值，单位 m。",
        ),
        DeclareLaunchArgument(
            "obstacle_height_bias_cm",
            default_value="0.0",
            description="[激光] 障碍高度偏置修正，单位 cm（正值会降低输出）。",
        ),
        DeclareLaunchArgument(
            "geometry_obstacle_enabled",
            default_value="true",
            description="[激光] 是否启用基于 8x8 波束几何的障碍高度估计。",
        ),
        DeclareLaunchArgument(
            "use_geometry_for_ground_height",
            default_value="false",
            description="[激光] 是否把几何地面高度直接用于 /laser_array/ground_height。默认关闭，仅用于障碍测高。",
        ),
        DeclareLaunchArgument(
            "beam_rows",
            default_value="8",
            description="[激光] 波束行数。当前实现要求 beam_rows * beam_cols = 64。",
        ),
        DeclareLaunchArgument(
            "beam_cols",
            default_value="8",
            description="[激光] 波束列数。当前实现要求 beam_rows * beam_cols = 64。",
        ),
        DeclareLaunchArgument(
            "beam_horizontal_fov_deg",
            default_value="45.0",
            description="[激光] 8x8 波束模型的水平视场角，单位 deg。",
        ),
        DeclareLaunchArgument(
            "beam_vertical_fov_deg",
            default_value="45.0",
            description="[激光] 8x8 波束模型的垂直视场角，单位 deg。",
        ),
        DeclareLaunchArgument(
            "flip_rows",
            default_value="false",
            description="[激光] 是否翻转波束行顺序，用于适配厂家束序。",
        ),
        DeclareLaunchArgument(
            "flip_cols",
            default_value="false",
            description="[激光] 是否翻转波束列顺序，用于适配厂家束序。",
        ),
        DeclareLaunchArgument(
            "beam_mount_roll_deg",
            default_value="0.0",
            description="[激光] 传感器安装滚转修正，单位 deg。",
        ),
        DeclareLaunchArgument(
            "beam_mount_pitch_deg",
            default_value="0.0",
            description="[激光] 传感器安装俯仰修正，单位 deg。",
        ),
        DeclareLaunchArgument(
            "beam_mount_yaw_deg",
            default_value="0.0",
            description="[激光] 传感器安装偏航修正，单位 deg。",
        ),
        DeclareLaunchArgument(
            "obstacle_top_size_cm",
            default_value="20.0",
            description="[激光] 矩形顶面边长，单位 cm。",
        ),
        DeclareLaunchArgument(
            "obstacle_center_slack_cm",
            default_value="15.0",
            description="[激光] 顶面中心允许偏移的额外搜索裕量，单位 cm。",
        ),
        DeclareLaunchArgument(
            "geometry_min_support",
            default_value="3",
            description="[激光] 判定矩形顶面所需的最少支持束数量。",
        ),
        DeclareLaunchArgument(
            "geometry_candidate_margin_cm",
            default_value="5.0",
            description="[激光] 几何候选点相对地面的最小高度，单位 cm。",
        ),
        DeclareLaunchArgument(
            "geometry_height_tolerance_cm",
            default_value="6.0",
            description="[激光] 顶面支持束在高度上的一致性阈值，单位 cm。",
        ),
        DeclareLaunchArgument(
            "uart_height_fusion_enabled",
            default_value="true",
            description="[激光] 是否启用 UART 单点激光对 obstacle_height 的辅助融合。",
        ),
        DeclareLaunchArgument(
            "uart_height_topic",
            default_value="/height_raw",
            description="[激光] 用于辅助障碍测高的 UART 原始高度话题。",
        ),
        DeclareLaunchArgument(
            "uart_height_bias_cm",
            default_value="5.0",
            description="[激光] 单点激光相对面阵地面高度的系统偏差修正，单位 cm。正值表示单点偏低，需要补高。",
        ),
        DeclareLaunchArgument(
            "uart_height_timeout_sec",
            default_value="0.30",
            description="[激光] UART 单点高度样本的超时阈值，单位 s。",
        ),
        DeclareLaunchArgument(
            "uart_block_margin_cm",
            default_value="8.0",
            description="[激光] 判定单点激光被障碍顶部挡住所需的最小高度差，单位 cm。",
        ),
        DeclareLaunchArgument(
            "uart_geometry_consistency_cm",
            default_value="8.0",
            description="[激光] 允许单点候选高度上拉面阵结果的最大差值，单位 cm。",
        ),
        DeclareLaunchArgument(
            "uart_blend_weight",
            default_value="0.35",
            description="[激光] 当单点与面阵一致时，单点高度参与 obstacle_height 融合的权重。",
        ),
        DeclareLaunchArgument(
            "obstacle_ema_alpha",
            default_value="0.3",
            description="[激光] obstacle_height EMA 低通滤波系数，越小越平滑。",
        ),
        Node(
            package="laser_array_pkg",
            executable="laser_array_ground_node",
            name="laser_array_ground_node",
            output="screen",
            parameters=[params],
        )
    ])
