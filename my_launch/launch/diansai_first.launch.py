import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# =============================================================================
# 电赛第一题任务参数面板
# 平时调试只改这里：
# 对外只保留三个任务点：起飞点、取物点、终点。
# pose = (x_cm, y_cm, z_cm, yaw_deg)，单位分别为厘米和角度。
#
# 任务流程：起飞 -> 固定取物点 -> 底部相机红色圆片视觉对准 -> 锁定当前 XY 下降取物
# -> 最多 3 次取物检查/重试 -> 搬运到终点 -> 放下铁块 -> 下降完成。
# =============================================================================

TAKEOFF_POINT = (0.0, 0.0, 80.0, 0.0)
PICKUP_POINT = (60.0, -60.0, 80.0, 0.0)
ENDPOINT = (60.0, -120.0, 80.0, 0.0)

ACTION_HEIGHT_CM = 20.0
COMPLETE_HEIGHT_CM = 10.0
PICKUP_HOLD_SEC = 5.0
DROPOFF_HOLD_SEC = 1.0

ALIGN_DEADZONE_PX = 30
ALIGN_STABLE_SEC = 0.5
ALIGN_TIMEOUT_SEC = 8.0
MAX_PICKUP_ATTEMPTS = 3
POST_PICKUP_CHECK_SEC = 1.0
POST_PICKUP_VISIBLE_STABLE_SEC = 0.3
SHOW_DEBUG_VIEW = True

# 红色圆片 HSV 识别参数
RED_H1_MIN = 0
RED_H1_MAX = 12
RED_H2_MIN = 139
RED_H2_MAX = 179
RED_S_MIN = 28
RED_S_MAX = 255
RED_V_MIN = 70
RED_V_MAX = 255
RED_BLUR = 5
RED_ERODE = 1
RED_DILATE = 2
RED_MIN_AREA = 300.0
RED_MIN_CIRCULARITY = 0.55


# 高度使用 STM32 串口发来的滤波后高度，不启动面阵激光高度节点。
HEIGHT_SOURCE = "serial_raw"


# 底部摄像头必须启用，型号使用原来的地面 DECXIN 摄像头。
BOTTOM_CAMERA_DEVICE = "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
BOTTOM_CAMERA_TOPIC = "/camera/image_raw"
BOTTOM_CAMERA_FRAME_ID = "bottom_camera"
BOTTOM_CAMERA_WIDTH = 640
BOTTOM_CAMERA_HEIGHT = 480
BOTTOM_CAMERA_FPS = 30
BOTTOM_CAMERA_PUBLISH_FPS = 30.0
BOTTOM_CAMERA_FOURCC = "YUYV"


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


def _target_params(prefix: str, point):
    return {
        f"{prefix}_x_cm": point[0],
        f"{prefix}_y_cm": point[1],
        f"{prefix}_z_cm": point[2],
        f"{prefix}_yaw_deg": point[3],
    }


def _diansai_task_params():
    params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "height_topic": "/height",
        "position_tolerance_cm": 8.0,
        "height_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "timer_period_sec": 0.05,
        "alignment_check_mode": False,
        "image_topic": BOTTOM_CAMERA_TOPIC,
        "fine_data_topic": "/fine_data",
        "show_debug_view": SHOW_DEBUG_VIEW,
        "action_height_cm": ACTION_HEIGHT_CM,
        "complete_height_cm": COMPLETE_HEIGHT_CM,
        "pickup_hold_sec": PICKUP_HOLD_SEC,
        "dropoff_hold_sec": DROPOFF_HOLD_SEC,
        "align_deadzone_px": ALIGN_DEADZONE_PX,
        "align_stable_sec": ALIGN_STABLE_SEC,
        "align_timeout_sec": ALIGN_TIMEOUT_SEC,
        "max_pickup_attempts": MAX_PICKUP_ATTEMPTS,
        "post_pickup_check_sec": POST_PICKUP_CHECK_SEC,
        "post_pickup_visible_stable_sec": POST_PICKUP_VISIBLE_STABLE_SEC,
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
        "red_min_circularity": RED_MIN_CIRCULARITY,
    }
    params.update(_target_params("takeoff", TAKEOFF_POINT))
    params.update(_target_params("pickup", PICKUP_POINT))
    params.update(_target_params("endpoint", ENDPOINT))
    return params


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    height_source = HEIGHT_SOURCE
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    task_log_dir = _task_log_dir("diansai_first")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", task_log_dir),
        LogInfo(msg=f"Task log directory: {task_log_dir}"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="是否启动 RViz。",
        ),
        DeclareLaunchArgument(
            "forward_height_0x05",
            default_value="true",
            description="是否把当前任务高度继续通过 0x05 帧转发给下游。",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_path("my_carto_pkg", "fly_carto.launch.py")
            ),
            launch_arguments={"use_rviz": use_rviz}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_path("uart_to_stm32", "uart_to_stm32.launch.py")
            ),
            launch_arguments={
                "height_source": height_source,
                "forward_height_0x05": forward_height_0x05,
            }.items(),
        ),
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
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _launch_path(
                            "pid_control_pkg",
                            "position_pid_controller.launch.py",
                        )
                    ),
                    launch_arguments={
                        "visual_mapping_mode": "legacy_xy",
                        "visual_pixel_deadzone": "5.0",
                        "visual_data_timeout_sec": "0.5",
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=12.5,
            actions=[
                Node(
                    package="activity_control_pkg",
                    executable="diansai_first_task_node",
                    name="diansai_first_task_node",
                    output="screen",
                    parameters=[_diansai_task_params()],
                )
            ],
        ),
    ])
