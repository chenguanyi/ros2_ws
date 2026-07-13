import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _start_display(context, *_args, **_kwargs):
    if not _as_bool(LaunchConfiguration("display").perform(context)):
        return []

    if not _as_bool(LaunchConfiguration("publish_debug_image").perform(context)):
        return [LogInfo(msg=(
            "display:=true 需要 publish_debug_image:=true；当前未启动 rqt_image_view，"
            "以避免显示空白窗口。"
        ))]

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        return [LogInfo(msg=(
            "display:=true，但未检测到 DISPLAY/WAYLAND_DISPLAY；跳过 rqt_image_view。"
            "相机和 YOLO 推理将继续运行。请在 Orange Pi 本地图形桌面会话中启动。"
        ))]

    try:
        get_package_prefix("rqt_image_view")
    except PackageNotFoundError:
        return [LogInfo(msg=(
            "未安装 rqt_image_view；跳过实时显示。请安装："
            "sudo apt install ros-$ROS_DISTRO-rqt-image-view"
        ))]

    debug_image_topic = LaunchConfiguration("debug_image_topic").perform(context)
    return [Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rknn_yolo_image_view",
        arguments=[debug_image_topic],
        output="screen",
    )]


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    camera_fps = LaunchConfiguration("camera_fps")
    publish_fps = LaunchConfiguration("publish_fps")
    fourcc = LaunchConfiguration("fourcc")
    retry_period_sec = LaunchConfiguration("retry_period_sec")
    frame_id = LaunchConfiguration("frame_id")
    image_topic = LaunchConfiguration("image_topic")
    model_path = LaunchConfiguration("model_path")
    classes_path = LaunchConfiguration("classes_path")
    num_classes = LaunchConfiguration("num_classes")
    detections_topic = LaunchConfiguration("detections_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    hud_enabled = LaunchConfiguration("hud_enabled")
    hud_stats_window_sec = LaunchConfiguration("hud_stats_window_sec")
    system_metrics_period_sec = LaunchConfiguration("system_metrics_period_sec")
    hud_show_system_metrics = LaunchConfiguration("hud_show_system_metrics")
    hud_show_npu_metrics = LaunchConfiguration("hud_show_npu_metrics")

    return LaunchDescription([
        DeclareLaunchArgument("camera_index", default_value="0", description="[相机] 相机索引。"),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0",
            description="[相机] 设备路径；非空时优先于 camera_index。",
        ),
        DeclareLaunchArgument("width", default_value="640", description="[相机] 图像宽度。"),
        DeclareLaunchArgument("height", default_value="480", description="[相机] 图像高度。"),
        DeclareLaunchArgument("camera_fps", default_value="30", description="[相机] 设备采集帧率。"),
        DeclareLaunchArgument("publish_fps", default_value="30.0", description="[相机] 图像发布帧率。"),
        DeclareLaunchArgument("fourcc", default_value="", description="[相机] 可选 V4L2 像素格式，例如 MJPG/YUYV；空表示驱动默认。"),
        DeclareLaunchArgument("retry_period_sec", default_value="1.0", description="[相机] 打开失败后的重试间隔。"),
        DeclareLaunchArgument("frame_id", default_value="camera", description="[相机] 图像 frame_id。"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw", description="[相机/YOLO] 相机图像话题。"),
        DeclareLaunchArgument("model_path", default_value="", description="[RKNN] .rknn 模型路径，必须传。"),
        DeclareLaunchArgument("classes_path", default_value="", description="[RKNN] 类别名文件路径；为空时需传 num_classes。"),
        DeclareLaunchArgument("num_classes", default_value="0", description="[YOLO] 类别数；0 表示从 classes_path 读取。"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo/detections", description="[YOLO] 检测结果输出话题。"),
        DeclareLaunchArgument("debug_image_topic", default_value="/yolo/debug_image", description="[YOLO/显示] 带检测框图像话题。"),
        DeclareLaunchArgument("publish_debug_image", default_value="true", description="[YOLO] 是否发布画框图像。display:=true 时必须为 true。"),
        DeclareLaunchArgument("display", default_value="false", description="[显示] 是否在 Orange Pi 本地图形桌面启动 rqt_image_view。"),
        DeclareLaunchArgument("hud_enabled", default_value="true", description="[显示] 是否在 debug 图绘制实时 FPS/时延 HUD。"),
        DeclareLaunchArgument("hud_stats_window_sec", default_value="1.0", description="[显示] FPS、DROP 和耗时统计窗口（秒）。"),
        DeclareLaunchArgument("system_metrics_period_sec", default_value="1.0", description="[显示] 系统资源采样周期（秒）。"),
        DeclareLaunchArgument("hud_show_system_metrics", default_value="true", description="[显示] 是否展示 CPU、内存和温度。"),
        DeclareLaunchArgument("hud_show_npu_metrics", default_value="true", description="[显示] 是否展示 NPU load 与频率。"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("visual_pkg"),
                "launch",
                "camera_source.launch.py",
            ])),
            launch_arguments={
                "camera_index": camera_index,
                "camera_device": camera_device,
                "width": width,
                "height": height,
                "camera_fps": camera_fps,
                "publish_fps": publish_fps,
                "fourcc": fourcc,
                "retry_period_sec": retry_period_sec,
                "frame_id": frame_id,
                "image_topic": image_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rknn_yolo_detector_pkg"),
                "launch",
                "rknn_yolo_detector.launch.py",
            ])),
            launch_arguments={
                "model_path": model_path,
                "classes_path": classes_path,
                "num_classes": num_classes,
                "image_topic": image_topic,
                "detections_topic": detections_topic,
                "debug_image_topic": debug_image_topic,
                "publish_debug_image": publish_debug_image,
                "hud_enabled": hud_enabled,
                "hud_stats_window_sec": hud_stats_window_sec,
                "system_metrics_period_sec": system_metrics_period_sec,
                "hud_show_system_metrics": hud_show_system_metrics,
                "hud_show_npu_metrics": hud_show_npu_metrics,
            }.items(),
        ),
        OpaqueFunction(function=_start_display),
    ])
