from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    camera_fps = LaunchConfiguration("camera_fps")
    publish_fps = LaunchConfiguration("publish_fps")
    retry_period_sec = LaunchConfiguration("retry_period_sec")
    frame_id = LaunchConfiguration("frame_id")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription([
        DeclareLaunchArgument("camera_index", default_value="0", description="[相机] 相机索引。"),
        DeclareLaunchArgument("camera_device", default_value="", description="[相机] 设备路径；非空时优先于 camera_index。"),
        DeclareLaunchArgument("width", default_value="640", description="[相机] 图像宽度。"),
        DeclareLaunchArgument("height", default_value="480", description="[相机] 图像高度。"),
        DeclareLaunchArgument("camera_fps", default_value="30", description="[相机] 设备采集帧率。"),
        DeclareLaunchArgument("publish_fps", default_value="30.0", description="[相机] 图像发布帧率。"),
        DeclareLaunchArgument("retry_period_sec", default_value="1.0", description="[相机] 打开失败后的重试间隔。"),
        DeclareLaunchArgument("frame_id", default_value="camera", description="[相机] 图像 frame_id。"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw", description="[相机] 图像输出话题。"),
        Node(
            package="visual_pkg",
            executable="camera_source_node",
            name="camera_source_node",
            output="screen",
            parameters=[{
                "camera_index": camera_index,
                "camera_device": camera_device,
                "width": width,
                "height": height,
                "camera_fps": camera_fps,
                "publish_fps": publish_fps,
                "retry_period_sec": retry_period_sec,
                "frame_id": frame_id,
                "image_topic": image_topic,
            }],
        ),
    ])
