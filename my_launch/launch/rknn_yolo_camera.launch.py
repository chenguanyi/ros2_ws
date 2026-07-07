from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    camera_fps = LaunchConfiguration("camera_fps")
    publish_fps = LaunchConfiguration("publish_fps")
    frame_id = LaunchConfiguration("frame_id")
    image_topic = LaunchConfiguration("image_topic")
    model_path = LaunchConfiguration("model_path")
    classes_path = LaunchConfiguration("classes_path")
    num_classes = LaunchConfiguration("num_classes")
    detections_topic = LaunchConfiguration("detections_topic")
    publish_debug_image = LaunchConfiguration("publish_debug_image")

    return LaunchDescription([
        DeclareLaunchArgument("camera_index", default_value="0", description="[相机] 相机索引。"),
        DeclareLaunchArgument("camera_device", default_value="/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0", description="[相机] 设备路径；非空时优先于 camera_index。"),
        DeclareLaunchArgument("width", default_value="640", description="[相机] 图像宽度。"),
        DeclareLaunchArgument("height", default_value="480", description="[相机] 图像高度。"),
        DeclareLaunchArgument("camera_fps", default_value="30", description="[相机] 设备采集帧率。"),
        DeclareLaunchArgument("publish_fps", default_value="30.0", description="[相机] 图像发布帧率。"),
        DeclareLaunchArgument("frame_id", default_value="camera", description="[相机] 图像 frame_id。"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw", description="[相机/YOLO] 相机图像话题。"),
        DeclareLaunchArgument("model_path", default_value="", description="[RKNN] .rknn 模型路径，必须传。"),
        DeclareLaunchArgument("classes_path", default_value="", description="[RKNN] 类别名文件路径；为空时需传 num_classes。"),
        DeclareLaunchArgument("num_classes", default_value="0", description="[YOLO] 类别数；0 表示从 classes_path 读取。"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo/detections", description="[YOLO] 检测结果输出话题。"),
        DeclareLaunchArgument("publish_debug_image", default_value="false", description="[YOLO] 是否发布画框图像。"),
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
                "publish_debug_image": publish_debug_image,
            }.items(),
        ),
    ])
