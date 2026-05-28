import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# =============================================================================
# G 题植保任务参数面板
# 平时调试只改这里：
# magnet 位复用为向下激光：1 发射，0 关闭。
# signal 位复用为 LED：1 亮，0 灭。
# home_pose = (x_cm, y_cm, z_cm, yaw_deg)
# =============================================================================

CRUISE_HEIGHT_CM = 150.0
HOME_POSE = (0.0, 0.0, 0.0, 0.0)
CELL_SIZE_CM = 50.0
LASER_ON_SEC = 0.4
LASER_OFF_SEC = 1.5
LASER_PULSE_COUNT = 2
EXCLUDED_CELLS = []
BARCODE_VALUE = 0
BARCODE_TOPIC = "/plant_protection/barcode_value"
BARCODE_SCAN_POSE = (0.0, 120.0, 105.0, 0.0)
BARCODE_SCAN_TIMEOUT_SEC = 10.0
CIRCLE_LANDING_ANGLE_DEG = 0.0

DOWN_CAMERA_DEVICE = "/dev/camera_decxin"
SIDE_CAMERA_DEVICE = "/dev/camera_wdr5m"
DOWN_IMAGE_TOPIC = "/down_camera/image_raw"
SIDE_IMAGE_TOPIC = "/side_camera/image_raw"
COLOR_DECISION_ROI_FRACTION = 0.35
COLOR_THRESHOLD_ROI_FRACTION = 0.80
COLOR_MAX_IMAGE_AGE_SEC = 0.5
COLOR_MIN_CONFIDENCE = 0.08
COLOR_NON_GREEN_INDEX_MAX = 0.10

LED_ON_SEC = 0.25
LED_OFF_SEC = 0.25
LED_REPEAT_GAP_SEC = 2.0


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
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")
    height_source = LaunchConfiguration("height_source")
    laser_height_topic = LaunchConfiguration("laser_height_topic")
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    task_log_dir = _task_log_dir("plant_protection")

    plant_task_params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "height_topic": "/height",
        "position_tolerance_cm": 8.0,
        "height_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "log_waypoint_targets": False,
        "timer_period_sec": 0.05,
        "cruise_height_cm": CRUISE_HEIGHT_CM,
        "home_pose": list(HOME_POSE),
        "cell_size_cm": CELL_SIZE_CM,
        "laser_on_sec": LASER_ON_SEC,
        "laser_off_sec": LASER_OFF_SEC,
        "laser_pulse_count": LASER_PULSE_COUNT,
        "image_topic": DOWN_IMAGE_TOPIC,
        "color_decision_roi_fraction": COLOR_DECISION_ROI_FRACTION,
        "color_threshold_roi_fraction": COLOR_THRESHOLD_ROI_FRACTION,
        "color_max_image_age_sec": COLOR_MAX_IMAGE_AGE_SEC,
        "color_min_confidence": COLOR_MIN_CONFIDENCE,
        "color_non_green_index_max": COLOR_NON_GREEN_INDEX_MAX,
        "barcode_value": BARCODE_VALUE,
        "barcode_topic": BARCODE_TOPIC,
        "barcode_scan_pose": list(BARCODE_SCAN_POSE),
        "barcode_scan_timeout_sec": BARCODE_SCAN_TIMEOUT_SEC,
        "circle_landing_angle_deg": CIRCLE_LANDING_ANGLE_DEG,
        "led_on_sec": LED_ON_SEC,
        "led_off_sec": LED_OFF_SEC,
        "led_repeat_gap_sec": LED_REPEAT_GAP_SEC,
    }
    if EXCLUDED_CELLS:
        plant_task_params["excluded_cells"] = EXCLUDED_CELLS

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", task_log_dir),
        LogInfo(msg=f"Task log directory: {task_log_dir}"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="是否启动 RViz。",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="是否启动向下/侧向双摄像头。",
        ),
        DeclareLaunchArgument(
            "height_source",
            default_value="laser_ground",
            description="任务高度来源：laser_ground 或 serial_raw。",
        ),
        DeclareLaunchArgument(
            "laser_height_topic",
            default_value="/laser_array/ground_height",
            description="height_source=laser_ground 时使用的高度话题。",
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
                "laser_height_topic": laser_height_topic,
                "forward_height_0x05": forward_height_0x05,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_path("laser_array_pkg", "laser_array_ground.launch.py")
            ),
        ),
        Node(
            package="visual_pkg",
            executable="camera_source_node",
            name="down_camera_source",
            output="screen",
            parameters=[{
                "camera_device": DOWN_CAMERA_DEVICE,
                "width": 640,
                "height": 480,
                "camera_fps": 30,
                "publish_fps": 30.0,
                "frame_id": "down_camera",
                "image_topic": DOWN_IMAGE_TOPIC,
            }],
            condition=IfCondition(use_camera),
        ),
        Node(
            package="visual_pkg",
            executable="camera_source_node",
            name="side_camera_source",
            output="screen",
            parameters=[{
                "camera_device": SIDE_CAMERA_DEVICE,
                "width": 640,
                "height": 480,
                "camera_fps": 30,
                "publish_fps": 30.0,
                "frame_id": "side_camera",
                "image_topic": SIDE_IMAGE_TOPIC,
            }],
            condition=IfCondition(use_camera),
        ),
        Node(
            package="activity_control_pkg",
            executable="barcode_reader_node",
            name="barcode_reader_node",
            output="screen",
            parameters=[{
                "image_topic": SIDE_IMAGE_TOPIC,
                "barcode_topic": BARCODE_TOPIC,
                "stable_count": 3,
            }],
            condition=IfCondition(use_camera),
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
                )
            ],
        ),
        TimerAction(
            period=12.5,
            actions=[
                Node(
                    package="activity_control_pkg",
                    executable="plant_protection_task_node",
                    name="plant_protection_task_node",
                    output="screen",
                    parameters=[plant_task_params],
                )
            ],
        ),
    ])
