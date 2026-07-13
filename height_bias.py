#!/usr/bin/env python3
"""订阅 /laser_array/ground_height，加 9cm 偏置后发布到 /height"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class HeightBias(Node):
    def __init__(self):
        super().__init__("height_bias_node")
        self.pub = self.create_publisher(Float32, "/height", 10)
        self.sub = self.create_subscription(
            Float32, "/laser_array/ground_height", self.cb, 10
        )
        self.get_logger().info("高度偏置节点已启动：+9cm")

    def cb(self, msg):
        biased = Float32()
        biased.data = msg.data + 0.09  # 加 9cm（单位：米）
        self.pub.publish(biased)

def main():
    rclpy.init()
    node = HeightBias()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
