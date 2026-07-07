from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration("model_path")
    classes_path = LaunchConfiguration("classes_path")
    image_topic = LaunchConfiguration("image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    input_width = LaunchConfiguration("input_width")
    input_height = LaunchConfiguration("input_height")
    num_classes = LaunchConfiguration("num_classes")
    input_color = LaunchConfiguration("input_color")
    conf_threshold = LaunchConfiguration("conf_threshold")
    nms_threshold = LaunchConfiguration("nms_threshold")
    max_detections = LaunchConfiguration("max_detections")
    anchors = LaunchConfiguration("anchors")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    queue_size = LaunchConfiguration("queue_size")
    log_period_sec = LaunchConfiguration("log_period_sec")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="", description="[RKNN] .rknn 模型路径，必须传。"),
        DeclareLaunchArgument("classes_path", default_value="", description="[RKNN] 类别名文件路径，每行一个类别；为空时需传 num_classes。"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw", description="[YOLO] 输入图像话题。"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo/detections", description="[YOLO] 检测结果 Float32MultiArray 输出话题。"),
        DeclareLaunchArgument("debug_image_topic", default_value="/yolo/debug_image", description="[YOLO] 画框调试图像输出话题。"),
        DeclareLaunchArgument("input_width", default_value="640", description="[YOLO] 模型输入宽度。"),
        DeclareLaunchArgument("input_height", default_value="640", description="[YOLO] 模型输入高度。"),
        DeclareLaunchArgument("num_classes", default_value="0", description="[YOLO] 类别数；0 表示从 classes_path 行数读取。"),
        DeclareLaunchArgument("input_color", default_value="rgb", description="[YOLO] 模型输入颜色顺序：rgb 或 bgr。"),
        DeclareLaunchArgument("conf_threshold", default_value="0.001", description="[YOLO] 置信度阈值。此模型输出值偏低，默认 0.001。"),
        DeclareLaunchArgument("nms_threshold", default_value="0.45", description="[YOLO] NMS IoU 阈值。"),
        DeclareLaunchArgument("max_detections", default_value="100", description="[YOLO] 单帧最多输出目标数。"),
        DeclareLaunchArgument(
            "anchors",
            default_value="10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326",
            description="[YOLO] YOLOv5 anchors，18 个数字。",
        ),
        DeclareLaunchArgument("publish_debug_image", default_value="false", description="[调试] 是否发布画框图像。"),
        DeclareLaunchArgument("queue_size", default_value="3", description="[并行] 三个 NPU worker 共享的待处理帧队列长度，满了丢旧帧。"),
        DeclareLaunchArgument("log_period_sec", default_value="1.0", description="[调试] 推理耗时日志周期。"),
        Node(
            package="rknn_yolo_detector_pkg",
            executable="rknn_yolo_detector_node",
            name="rknn_yolo_detector_node",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "classes_path": classes_path,
                "image_topic": image_topic,
                "detections_topic": detections_topic,
                "debug_image_topic": debug_image_topic,
                "input_width": input_width,
                "input_height": input_height,
                "num_classes": num_classes,
                "input_color": input_color,
                "conf_threshold": conf_threshold,
                "nms_threshold": nms_threshold,
                "max_detections": max_detections,
                "anchors": anchors,
                "publish_debug_image": publish_debug_image,
                "queue_size": queue_size,
                "log_period_sec": log_period_sec,
            }],
        ),
    ])
