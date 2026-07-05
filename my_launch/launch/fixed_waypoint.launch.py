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
# 固定航点任务参数面板
# 平时调试只改这里：
# 每个 step 是一个航点，到点后会执行 arm/magnet/signal 并等待 hold_sec。
# 注意：hold_sec=0 会在到点后立刻切到下一个航点；如果该点需要激光/电磁铁动作，
# 必须给足停留时间，否则 magnet=1 只会保持一个调度周期左右。
# pose = (x_cm, y_cm, z_cm, yaw_deg)
# =============================================================================

MISSION_STEPS = [
    {
        "pose": (0.0, 0.0, 80.0, 0.0),
        "arm": 0,
        "magnet": 0,
        "signal": 0,
        "hold_sec": 0.0,
    },
    {
        "pose": (100.0, 0.0, 80.0, 0.0),
        "arm": 0,
        "magnet": 1,
        "signal": 0,
        "hold_sec": 1.0,
    },
    {
        "pose": (100.0, -100.0, 80.0, 0.0),
        "arm": 0,
        "magnet": 0,
        "signal": 0,
        "hold_sec": 0.0,
    },
    {
        "pose": (0.0, 0.0, 80.0, 0.0),
        "arm": 0,
        "magnet": 0,
        "signal": 0,
        "hold_sec": 0.0,
    },
    {
        "pose": (0.0, 0.0, 0.0, 0.0),
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
    use_camera = LaunchConfiguration("use_camera")
    height_source = LaunchConfiguration("height_source")
    laser_height_topic = LaunchConfiguration("laser_height_topic")
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    task_log_dir = _task_log_dir("fixed_waypoint")

    waypoints = [value for step in MISSION_STEPS for value in step["pose"]]
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
        "arm_states": [step["arm"] for step in MISSION_STEPS],
        "magnet_states": [step["magnet"] for step in MISSION_STEPS],
        "signal_states": [step["signal"] for step in MISSION_STEPS],
        "hold_seconds": [step["hold_sec"] for step in MISSION_STEPS],
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
            "use_camera",
            default_value="true",
            description="是否启动 /camera/image_raw 相机源。",
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_path("visual_pkg", "camera_source.launch.py")
            ),
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
                    executable="fixed_waypoint_task_node",
                    name="fixed_waypoint_task_node",
                    output="screen",
                    parameters=[fixed_task_params],
                )
            ],
        ),
    ])
