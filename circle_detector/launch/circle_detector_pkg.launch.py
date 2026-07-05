from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration('camera_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    camera_fps = LaunchConfiguration('camera_fps')
    show_display = LaunchConfiguration('show_display')
    publish_fine_data = LaunchConfiguration('publish_fine_data')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0', description='[视觉] 相机索引。'),
        DeclareLaunchArgument('width', default_value='640', description='[视觉] 相机图像宽度。'),
        DeclareLaunchArgument('height', default_value='480', description='[视觉] 相机图像高度。'),
        DeclareLaunchArgument('camera_fps', default_value='60', description='[视觉] 相机帧率。'),
        DeclareLaunchArgument('show_display', default_value='true', description='[视觉] 是否显示检测窗口。'),
        DeclareLaunchArgument('publish_fine_data', default_value='true', description='[视觉] 独立运行时是否直接发布 PID 需要的 /fine_data。'),
        Node(
            package='circle_detector_pkg',
            executable='circle_detector_node',
            name='circle_detector',
            output='screen',
            parameters=[{
                'camera_index': camera_index,
                'width': width,
                'height': height,
                'camera_fps': camera_fps,
                'show_display': show_display,
                'publish_fine_data': publish_fine_data,
            }],
        ),
    ])
