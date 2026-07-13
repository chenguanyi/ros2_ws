import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_MODEL_PATH = "/home/orangepi/ros2_ws/animal_yolov5s_best_verified_deploy/animal_yolov5s_best_fp_rk3588.rknn"
DEFAULT_CLASSES_PATH = "/home/orangepi/ros2_ws/animal_yolov5s_best_verified_deploy/classes.txt"


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _start_display(context, *_args, **_kwargs):
    if not _as_bool(LaunchConfiguration("display").perform(context)):
        return []
    if not _as_bool(LaunchConfiguration("publish_debug_image").perform(context)):
        return [LogInfo(msg="display=true requires publish_debug_image=true; display skipped.")]
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        return [LogInfo(msg="No DISPLAY/WAYLAND_DISPLAY; camera and YOLO continue without a window.")]
    try:
        get_package_prefix("rqt_image_view")
    except PackageNotFoundError:
        return [LogInfo(msg="rqt_image_view is not installed; see /yolo/debug_image instead.")]
    return [Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="wildlife_yolo_image_view",
        arguments=[LaunchConfiguration("debug_image_topic").perform(context)],
        output="screen",
    )]


def _workspace_root() -> str:
    share = FindPackageShare(package="my_launch").find("my_launch")
    marker = os.sep + "install" + os.sep
    return share.split(marker, 1)[0] if marker in share else os.getcwd()


def _launch_path(package_name: str, filename: str) -> str:
    share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")
    height_source = LaunchConfiguration("height_source")
    laser_height_topic = LaunchConfiguration("laser_height_topic")
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    model_path = LaunchConfiguration("model_path")
    classes_path = LaunchConfiguration("classes_path")
    conf_threshold = LaunchConfiguration("conf_threshold")
    animal_confidence_threshold = LaunchConfiguration("animal_confidence_threshold")
    animal_confirm_frames = LaunchConfiguration("animal_confirm_frames")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    animal_topic = LaunchConfiguration("animal_topic")
    status_topic = LaunchConfiguration("status_topic")
    laser_log_period_sec = LaunchConfiguration("laser_log_period_sec")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    display = LaunchConfiguration("display")
    camera_fourcc = LaunchConfiguration("camera_fourcc")
    log_dir = os.path.join(_workspace_root(), "mylog", "wildlife_patrol")

    task_params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "height_topic": "/height",
        "position_tolerance_cm": 8.0,
        "height_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "waypoints_topic": waypoints_topic,
        "detections_topic": detections_topic,
        "animal_topic": animal_topic,
        "status_topic": status_topic,
        "takeoff_x_cm": 0.0,
        "takeoff_y_cm": 0.0,
        "takeoff_z_cm": 120.0,
        "takeoff_yaw_deg": 0.0,
        "route_height_cm": 120.0,
        "route_yaw_deg": 0.0,
        "animal_confidence_threshold": animal_confidence_threshold,
        "animal_confirm_frames": animal_confirm_frames,
    }

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", log_dir),
        LogInfo(msg=f"Task log directory: {log_dir}"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("height_source", default_value="laser_ground"),
        DeclareLaunchArgument("laser_height_topic", default_value="/laser_array/ground_height"),
        DeclareLaunchArgument("forward_height_0x05", default_value="true"),
        DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument("classes_path", default_value=DEFAULT_CLASSES_PATH),
        DeclareLaunchArgument("conf_threshold", default_value="0.70"),
        DeclareLaunchArgument("animal_confidence_threshold", default_value="0.70"),
        DeclareLaunchArgument("animal_confirm_frames", default_value="3"),
        DeclareLaunchArgument("waypoints_topic", default_value="/wildlife/waypoints"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
        DeclareLaunchArgument("animal_topic", default_value="/animal"),
        DeclareLaunchArgument("status_topic", default_value="/wildlife/status"),
        DeclareLaunchArgument("laser_log_period_sec", default_value="5.0"),
        DeclareLaunchArgument("debug_image_topic", default_value="/yolo/debug_image"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        DeclareLaunchArgument("display", default_value="true"),
        DeclareLaunchArgument("camera_fourcc", default_value="MJPG"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py")),
            launch_arguments={"use_rviz": use_rviz}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_path("uart_to_stm32", "uart_to_stm32.launch.py")),
            launch_arguments={
                "height_source": height_source,
                "laser_height_topic": laser_height_topic,
                "forward_height_0x05": forward_height_0x05,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_path("laser_array_pkg", "laser_array_ground.launch.py")),
            launch_arguments={"log_period_sec": laser_log_period_sec}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_path("visual_pkg", "camera_source.launch.py")),
            launch_arguments={"fourcc": camera_fourcc}.items(),
            condition=IfCondition(use_camera),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_path("rknn_yolo_detector_pkg", "rknn_yolo_detector.launch.py")),
            launch_arguments={
                "model_path": model_path,
                "classes_path": classes_path,
                "num_classes": "5",
                "detections_topic": detections_topic,
                "conf_threshold": conf_threshold,
                "debug_image_topic": debug_image_topic,
                "publish_debug_image": publish_debug_image,
            }.items(),
        ),
        TimerAction(
            period=12.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_path("pid_control_pkg", "position_pid_controller.launch.py")),
            )],
        ),
        TimerAction(
            period=12.5,
            actions=[Node(
                package="activity_control_pkg",
                executable="wildlife_patrol_task_node",
                name="wildlife_patrol_task_node",
                output="screen",
                parameters=[task_params],
            )],
        ),
        OpaqueFunction(function=_start_display),
    ])
