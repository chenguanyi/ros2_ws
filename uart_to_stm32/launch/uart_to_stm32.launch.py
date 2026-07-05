from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    height_source = LaunchConfiguration("height_source")
    laser_height_topic = LaunchConfiguration("laser_height_topic")
    forward_height_0x05 = LaunchConfiguration("forward_height_0x05")
    a2_status_enabled = LaunchConfiguration("a2_status_enabled")
    barcode_topic = LaunchConfiguration("barcode_topic")

    uart_params = {
        "update_rate": 100.0,
        "source_frame": "map",
        "target_frame": "laser_link",
        "cruise_height_cm": 130,
        "height_band_cm": 20,
        "min_valid_height_cm": 0,
        "max_valid_height_cm": 500,
        "height_source": height_source,
        "laser_height_topic": laser_height_topic,
        "forward_height_0x05": forward_height_0x05,
        "a2_status_enabled": a2_status_enabled,
        "barcode_topic": barcode_topic,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "height_source",
            default_value="serial_raw",
            description="Mission height source: serial_raw or laser_ground.",
        ),
        DeclareLaunchArgument(
            "laser_height_topic",
            default_value="/laser_array/ground_height",
            description="Laser-derived height topic used when height_source=laser_ground.",
        ),
        DeclareLaunchArgument(
            "forward_height_0x05",
            default_value="false",
            description="Whether to forward the selected mission height downstream using frame ID 0x05.",
        ),
        DeclareLaunchArgument(
            "a2_status_enabled",
            default_value="false",
            description="Whether to send D-task A2 elapsed-time and QR status frames.",
        ),
        DeclareLaunchArgument(
            "barcode_topic",
            default_value="/warehouse_inventory/barcode_value",
            description="QR/barcode topic used by optional A2 status frames.",
        ),
        Node(
            package="uart_to_stm32",
            executable="uart_to_stm32_node",
            name="uart_to_stm32",
            parameters=[uart_params],
            output="screen",
        )
    ])
