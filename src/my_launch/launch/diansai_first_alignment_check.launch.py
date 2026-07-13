import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# =============================================================================
# 电赛第一题底部摄像头对准检查
# 只启动摄像头、红色方块识别、正式 PID 和假 TF/高度/目标监视脚本。
# 不启动 uart_to_stm32，因此不会向 STM32 发送速度指令。
# =============================================================================

BOTTOM_CAMERA_DEVICE = "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
BOTTOM_CAMERA_TOPIC = "/camera/image_raw"
BOTTOM_CAMERA_FRAME_ID = "bottom_camera"
BOTTOM_CAMERA_WIDTH = 640
BOTTOM_CAMERA_HEIGHT = 480
BOTTOM_CAMERA_FPS = 30
BOTTOM_CAMERA_PUBLISH_FPS = 30.0
BOTTOM_CAMERA_FOURCC = "YUYV"

ALIGN_DEADZONE_PX = 30
# 视觉抓取固定像素偏差：目标点 = 画面中心 + offset。
# x 正值表示把对准目标向画面右侧移动；y 正值表示向画面下方移动。
ALIGN_TARGET_OFFSET_X_PX = -10.0
ALIGN_TARGET_OFFSET_Y_PX = -60.0
SHOW_DEBUG_VIEW = True

# 红色方块 HSV 识别参数（排除橙色干扰，鲜红色 H 范围 0-8 和 155-179）
RED_H1_MIN = 0
RED_H1_MAX = 8
RED_H2_MIN = 155
RED_H2_MAX = 179
RED_S_MIN = 50
RED_S_MAX = 255
RED_V_MIN = 80
RED_V_MAX = 255
RED_BLUR = 5
RED_ERODE = 1
RED_DILATE = 2
RED_MIN_AREA = 300.0
RED_MIN_RECTANGULARITY = 0.6


def _workspace_root() -> str:
    share = FindPackageShare(package="my_launch").find("my_launch")
    install_marker = os.sep + "install" + os.sep
    if install_marker in share:
        return share.split(install_marker, 1)[0]

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if os.path.basename(launch_dir) == "launch":
        return os.path.abspath(os.path.join(launch_dir, "..", "..", ".."))
    return os.getcwd()


def _task_log_dir(task_name: str) -> str:
    log_dir = os.path.join(_workspace_root(), "mylog", task_name)
    os.makedirs(log_dir, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = log_dir
    return log_dir


def _launch_path(package_name: str, filename: str) -> str:
    share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    task_log_dir = _task_log_dir("diansai_first_alignment_check")

    detection_params = {
        "alignment_check_mode": True,
        "image_topic": BOTTOM_CAMERA_TOPIC,
        "fine_data_topic": "/fine_data",
        "show_debug_view": SHOW_DEBUG_VIEW,
        "align_deadzone_px": ALIGN_DEADZONE_PX,
        "align_target_offset_x_px": ALIGN_TARGET_OFFSET_X_PX,
        "align_target_offset_y_px": ALIGN_TARGET_OFFSET_Y_PX,
        "red_h1_min": RED_H1_MIN,
        "red_h1_max": RED_H1_MAX,
        "red_h2_min": RED_H2_MIN,
        "red_h2_max": RED_H2_MAX,
        "red_s_min": RED_S_MIN,
        "red_s_max": RED_S_MAX,
        "red_v_min": RED_V_MIN,
        "red_v_max": RED_V_MAX,
        "red_blur": RED_BLUR,
        "red_erode": RED_ERODE,
        "red_dilate": RED_DILATE,
        "red_min_area": RED_MIN_AREA,
        "red_min_rectangularity": RED_MIN_RECTANGULARITY,
    }

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", task_log_dir),
        LogInfo(msg=f"Alignment-check log directory: {task_log_dir}"),
        Node(
            package="visual_pkg",
            executable="camera_source_node",
            name="diansai_bottom_camera_source",
            output="screen",
            parameters=[{
                "camera_device": BOTTOM_CAMERA_DEVICE,
                "width": BOTTOM_CAMERA_WIDTH,
                "height": BOTTOM_CAMERA_HEIGHT,
                "camera_fps": BOTTOM_CAMERA_FPS,
                "fourcc": BOTTOM_CAMERA_FOURCC,
                "publish_fps": BOTTOM_CAMERA_PUBLISH_FPS,
                "frame_id": BOTTOM_CAMERA_FRAME_ID,
                "image_topic": BOTTOM_CAMERA_TOPIC,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_path("pid_control_pkg", "position_pid_controller.launch.py")
            ),
            launch_arguments={
                "visual_mapping_mode": "legacy_xy",
                "visual_pixel_deadzone": "5.0",
                "visual_data_timeout_sec": "0.5",
            }.items(),
        ),
        Node(
            package="activity_control_pkg",
            executable="diansai_first_task_node",
            name="diansai_first_detection_check_node",
            output="screen",
            parameters=[detection_params],
        ),
        Node(
            package="activity_control_pkg",
            executable="alignment_check_support.py",
            name="diansai_first_alignment_check_support",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "laser_link_frame": "laser_link",
                "height_cm": 80,
                "target_x_cm": 0.0,
                "target_y_cm": 0.0,
                "target_z_cm": 80.0,
                "target_yaw_deg": 0.0,
                "publish_rate_hz": 20.0,
            }],
        ),
    ])
