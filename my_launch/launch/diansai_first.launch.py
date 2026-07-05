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
# 任务会在 launch 内部自动展开为固定航点动作序列：
# 1. 起飞到 TAKEOFF_POINT
# 2. 飞到 PICKUP_POINT 安全高度
# 3. 下降到 ACTION_HEIGHT_CM，放机械臂、开磁铁、收机械臂
# 4. 搬运到 ENDPOINT 安全高度
# 5. 下降到 ACTION_HEIGHT_CM，放机械臂、关磁铁、收机械臂
# 6. 下降到 COMPLETE_HEIGHT_CM 后发布 /mission_complete
# =============================================================================

TAKEOFF_POINT = (0.0, 0.0, 80.0, 0.0)
PICKUP_POINT = (60.0, -60.0, 80.0, 0.0)
ENDPOINT = (60.0, -120.0, 80.0, 0.0)

ACTION_HEIGHT_CM = 20.0
COMPLETE_HEIGHT_CM = 10.0
PICKUP_HOLD_SEC = 5.0
DROPOFF_HOLD_SEC = 1.0


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


def _with_height(point, height_cm):
    return (point[0], point[1], height_cm, point[3])


def _mission_steps():
    pickup_action_pose = _with_height(PICKUP_POINT, ACTION_HEIGHT_CM)
    endpoint_action_pose = _with_height(ENDPOINT, ACTION_HEIGHT_CM)
    complete_pose = _with_height(ENDPOINT, COMPLETE_HEIGHT_CM)

    return [
        {
            "pose": TAKEOFF_POINT,
            "arm": 0,
            "magnet": 0,
            "signal": 0,
            "hold_sec": 0.0,
        },
        {
            "pose": PICKUP_POINT,
            "arm": 0,
            "magnet": 0,
            "signal": 0,
            "hold_sec": 0.0,
        },
        {
            "pose": pickup_action_pose,
            "arm": 0,
            "magnet": 0,
            "signal": 0,
            "hold_sec": 0.0,
        },
        {
            "pose": pickup_action_pose,
            "arm": 1,
            "magnet": 0,
            "signal": 0,
            "hold_sec": PICKUP_HOLD_SEC,
        },
        {
            "pose": pickup_action_pose,
            "arm": 1,
            "magnet": 1,
            "signal": 0,
            "hold_sec": PICKUP_HOLD_SEC,
        },
        {
            "pose": pickup_action_pose,
            "arm": 0,
            "magnet": 1,
            "signal": 0,
            "hold_sec": PICKUP_HOLD_SEC,
        },
        {
            "pose": ENDPOINT,
            "arm": 0,
            "magnet": 1,
            "signal": 0,
            "hold_sec": 0.0,
        },
        {
            "pose": endpoint_action_pose,
            "arm": 0,
            "magnet": 1,
            "signal": 0,
            "hold_sec": 0.0,
        },
        {
            "pose": endpoint_action_pose,
            "arm": 1,
            "magnet": 1,
            "signal": 0,
            "hold_sec": DROPOFF_HOLD_SEC,
        },
        {
            "pose": endpoint_action_pose,
            "arm": 1,
            "magnet": 0,
            "signal": 0,
            "hold_sec": DROPOFF_HOLD_SEC,
        },
        {
            "pose": endpoint_action_pose,
            "arm": 0,
            "magnet": 0,
            "signal": 0,
            "hold_sec": DROPOFF_HOLD_SEC,
        },
        {
            "pose": complete_pose,
            "arm": 0,
            "magnet": 0,
            "signal": 0,
            "hold_sec": 0.0,
        },
    ]


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
    height_source = HEIGHT_SOURCE
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    task_log_dir = _task_log_dir("diansai_first")

    mission_steps = _mission_steps()
    waypoints = [value for step in mission_steps for value in step["pose"]]
    fixed_task_params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "height_topic": "/height",
        "position_tolerance_cm": 8.0,
        "height_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "timer_period_sec": 0.05,
        "waypoints": waypoints,
        "arm_states": [step["arm"] for step in mission_steps],
        "magnet_states": [step["magnet"] for step in mission_steps],
        "signal_states": [step["signal"] for step in mission_steps],
        "hold_seconds": [step["hold_sec"] for step in mission_steps],
    }

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
                )
            ],
        ),
        TimerAction(
            period=12.5,
            actions=[
                Node(
                    package="activity_control_pkg",
                    executable="fixed_waypoint_task_node",
                    name="diansai_first_task_node",
                    output="screen",
                    parameters=[fixed_task_params],
                )
            ],
        ),
    ])
