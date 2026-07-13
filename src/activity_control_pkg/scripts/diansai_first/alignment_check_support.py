#!/usr/bin/env python3
"""Safe support node for diansai_first bottom-camera alignment checks."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int16, Int32MultiArray
from tf2_ros import TransformBroadcaster


class AlignmentCheckSupport(Node):
    def __init__(self) -> None:
        super().__init__('diansai_first_alignment_check_support')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('laser_link_frame', 'laser_link')
        self.declare_parameter('height_cm', 80)
        self.declare_parameter('target_x_cm', 0.0)
        self.declare_parameter('target_y_cm', 0.0)
        self.declare_parameter('target_z_cm', 80.0)
        self.declare_parameter('target_yaw_deg', 0.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.laser_link_frame = self.get_parameter('laser_link_frame').value
        self.height_cm = int(self.get_parameter('height_cm').value)
        self.target_x_cm = float(self.get_parameter('target_x_cm').value)
        self.target_y_cm = float(self.get_parameter('target_y_cm').value)
        self.target_z_cm = float(self.get_parameter('target_z_cm').value)
        self.target_yaw_deg = float(self.get_parameter('target_yaw_deg').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.height_pub = self.create_publisher(Int16, '/height', 10)
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_position', durable_qos)
        self.takeover_pub = self.create_publisher(Bool, '/visual_takeover_active', durable_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_fine_data: Optional[Int32MultiArray] = None
        self.latest_velocity: Optional[Float32MultiArray] = None
        self.create_subscription(Int32MultiArray, '/fine_data', self.fine_data_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_velocity', self.velocity_callback, 10)

        self.pub_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_fake_inputs)
        self.print_timer = self.create_timer(0.5, self.print_status)
        self.get_logger().info(
            '底部摄像头对准检查支持节点已启动：仅发布假 TF/高度/目标/视觉接管，不启动 UART。'
        )

    def publish_fake_inputs(self) -> None:
        now = self.get_clock().now().to_msg()

        height = Int16()
        height.data = self.height_cm
        self.height_pub.publish(height)

        target = Float32MultiArray()
        target.data = [self.target_x_cm, self.target_y_cm, self.target_z_cm, self.target_yaw_deg]
        self.target_pub.publish(target)

        takeover = Bool()
        takeover.data = True
        self.takeover_pub.publish(takeover)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.laser_link_frame
        transform.transform.translation.x = self.target_x_cm / 100.0
        transform.transform.translation.y = self.target_y_cm / 100.0
        transform.transform.translation.z = 0.0
        yaw = math.radians(self.target_yaw_deg)
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

    def fine_data_callback(self, msg: Int32MultiArray) -> None:
        self.latest_fine_data = msg

    def velocity_callback(self, msg: Float32MultiArray) -> None:
        self.latest_velocity = msg

    def print_status(self) -> None:
        if self.latest_fine_data is None:
            self.get_logger().info('等待 /fine_data：请把红色圆片放到底部摄像头画面中。')
            return
        fine = list(self.latest_fine_data.data)
        velocity = list(self.latest_velocity.data) if self.latest_velocity is not None else []
        vx = velocity[0] if len(velocity) > 0 else 0.0
        vy = velocity[1] if len(velocity) > 1 else 0.0
        vz = velocity[2] if len(velocity) > 2 else 0.0
        vyaw = velocity[3] if len(velocity) > 3 else 0.0
        self.get_logger().info(
            'fine_data=(%+d,%+d) px, target_velocity=(%+.2f,%+.2f,%+.2f,%+.2f). '
            '观察屏幕：目标向右/下移动时，速度方向应让圆片回到中心。'
            % (fine[0], fine[1], vx, vy, vz, vyaw)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignmentCheckSupport()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
