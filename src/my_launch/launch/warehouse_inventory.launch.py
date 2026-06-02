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
# D task warehouse inventory.
# Fixed geometry and fixed right-side camera fine-control mapping live in code.
# Only switch mission_mode between inventory and target:
#   inventory: scan all 24 QR codes and update latest_success.json on success.
#   target: preflight QR -> latest_success.json lookup -> direct safe route.
# =============================================================================

SIDE_CAMERA_DEVICE = "/dev/camera_icspring"
SIDE_IMAGE_TOPIC = "/warehouse_inventory/side_camera/image_raw"
BARCODE_TOPIC = "/warehouse_inventory/barcode_value"
BARCODE_CANDIDATE_TOPIC = "/warehouse_inventory/barcode_candidate"
BARCODE_OVERLAY_TOPIC = "/warehouse_inventory/barcode_overlay"
FINE_DATA_TOPIC = "/fine_data"


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
    mission_mode = LaunchConfiguration("mission_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")
    use_viewer = LaunchConfiguration("use_viewer")
    height_source = LaunchConfiguration("height_source")
    laser_height_topic = LaunchConfiguration("laser_height_topic")
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    task_log_dir = _task_log_dir("warehouse_inventory")

    task_params = {
        "mission_mode": mission_mode,
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "height_topic": "/height",
        "position_tolerance_cm": 5.0,
        "height_tolerance_cm": 4.0,
        "yaw_tolerance_deg": 3.0,
        "log_waypoint_targets": False,
        "timer_period_sec": 0.05,
        "barcode_topic": BARCODE_TOPIC,
        "fine_data_topic": FINE_DATA_TOPIC,
    }

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", task_log_dir),
        LogInfo(msg=f"Task log directory: {task_log_dir}"),
        DeclareLaunchArgument(
            "mission_mode",
            default_value="inventory",
            description="D task mode: inventory or target.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Start right-side camera and QR reader.",
        ),
        DeclareLaunchArgument(
            "use_viewer",
            default_value="false",
            description="Start OpenCV side-camera debug viewer.",
        ),
        DeclareLaunchArgument(
            "height_source",
            default_value="laser_ground",
            description="Mission height source: laser_ground or serial_raw.",
        ),
        DeclareLaunchArgument(
            "laser_height_topic",
            default_value="/laser_array/ground_height",
            description="Height topic used when height_source=laser_ground.",
        ),
        DeclareLaunchArgument(
            "forward_height_0x05",
            default_value="true",
            description="Forward selected mission height through frame 0x05.",
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
            name="warehouse_side_camera_source",
            output="screen",
            parameters=[{
                "camera_device": SIDE_CAMERA_DEVICE,
                "width": 640,
                "height": 480,
                "camera_fps": 30,
                "publish_fps": 30.0,
                "frame_id": "warehouse_side_camera",
                "image_topic": SIDE_IMAGE_TOPIC,
            }],
            condition=IfCondition(use_camera),
        ),
        Node(
            package="activity_control_pkg",
            executable="barcode_reader_node",
            name="warehouse_qr_reader_node",
            output="screen",
            parameters=[{
                "image_topic": SIDE_IMAGE_TOPIC,
                "barcode_topic": BARCODE_TOPIC,
                "candidate_topic": BARCODE_CANDIDATE_TOPIC,
                "overlay_topic": BARCODE_OVERLAY_TOPIC,
                "fine_data_topic": FINE_DATA_TOPIC,
                "publish_fine_data": True,
                "stable_count": 2,
                "republish_period_sec": 0.2,
            }],
            condition=IfCondition(use_camera),
        ),
        Node(
            package="activity_control_pkg",
            executable="side_camera_viewer_node",
            name="warehouse_side_camera_viewer_node",
            output="screen",
            parameters=[{
                "image_topic": BARCODE_OVERLAY_TOPIC,
                "candidate_topic": BARCODE_CANDIDATE_TOPIC,
                "barcode_topic": BARCODE_TOPIC,
                "window_name": "Warehouse side camera",
                "display_width": 960,
            }],
            condition=IfCondition(use_viewer),
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
                        "visual_mapping_mode": "right_side_camera",
                        "visual_kp_x": "0.08",
                        "visual_kd_x": "0.01",
                        "visual_kp_z": "0.06",
                        "visual_kd_z": "0.01",
                        "visual_target_offset_x_px": "-75.0",
                        "visual_target_offset_y_px": "20.0",
                        "visual_pixel_deadzone": "5.0",
                        "visual_max_xy_velocity": "20.0",
                        "visual_max_z_velocity": "18.0",
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
                    executable="warehouse_inventory_task_node",
                    name="warehouse_inventory_task_node",
                    output="screen",
                    parameters=[task_params],
                )
            ],
        ),
    ])
