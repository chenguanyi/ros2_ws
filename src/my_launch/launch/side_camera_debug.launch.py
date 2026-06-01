from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_device = LaunchConfiguration("camera_device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    camera_fps = LaunchConfiguration("camera_fps")
    publish_fps = LaunchConfiguration("publish_fps")
    image_topic = LaunchConfiguration("image_topic")
    barcode_topic = LaunchConfiguration("barcode_topic")
    candidate_topic = LaunchConfiguration("candidate_topic")
    overlay_topic = LaunchConfiguration("overlay_topic")
    display_width = LaunchConfiguration("display_width")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0",
            description="[侧向相机] 设备路径。",
        ),
        DeclareLaunchArgument("width", default_value="640", description="[侧向相机] 图像宽度。"),
        DeclareLaunchArgument("height", default_value="480", description="[侧向相机] 图像高度。"),
        DeclareLaunchArgument("camera_fps", default_value="30", description="[侧向相机] 设备采集帧率。"),
        DeclareLaunchArgument("publish_fps", default_value="30.0", description="[侧向相机] 图像发布帧率。"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/side_camera/image_raw",
            description="[侧向相机] 原始图像话题。",
        ),
        DeclareLaunchArgument(
            "barcode_topic",
            default_value="/plant_protection/barcode_value",
            description="[侧向相机] 稳定条码识别值。",
        ),
        DeclareLaunchArgument(
            "candidate_topic",
            default_value="/plant_protection/barcode_candidate",
            description="[侧向相机] 单帧候选条码识别值。",
        ),
        DeclareLaunchArgument(
            "overlay_topic",
            default_value="/side_camera/barcode_overlay",
            description="[侧向相机] 带识别框的图像话题。",
        ),
        DeclareLaunchArgument(
            "display_width",
            default_value="960",
            description="[侧向相机] OpenCV 显示窗口宽度；0 表示原始尺寸。",
        ),
        Node(
            package="visual_pkg",
            executable="camera_source_node",
            name="side_camera_source",
            output="screen",
            parameters=[{
                "camera_device": camera_device,
                "width": width,
                "height": height,
                "camera_fps": camera_fps,
                "publish_fps": publish_fps,
                "frame_id": "side_camera",
                "image_topic": image_topic,
            }],
        ),
        Node(
            package="activity_control_pkg",
            executable="barcode_reader_node",
            name="barcode_reader_node",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "barcode_topic": barcode_topic,
                "candidate_topic": candidate_topic,
                "overlay_topic": overlay_topic,
                "stable_count": 3,
                "republish_period_sec": 1.0,
            }],
        ),
        Node(
            package="activity_control_pkg",
            executable="side_camera_viewer_node",
            name="side_camera_viewer_node",
            output="screen",
            parameters=[{
                "image_topic": overlay_topic,
                "candidate_topic": candidate_topic,
                "barcode_topic": barcode_topic,
                "window_name": "Side camera barcode",
                "display_width": display_width,
            }],
        ),
    ])
