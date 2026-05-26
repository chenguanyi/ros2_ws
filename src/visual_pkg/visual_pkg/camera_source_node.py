from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_source_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("camera_fps", 60)
        self.declare_parameter("publish_fps", 30.0)
        self.declare_parameter("retry_period_sec", 1.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "/camera/image_raw")

        self._camera_index = int(self.get_parameter("camera_index").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._camera_fps = int(self.get_parameter("camera_fps").value)
        publish_fps = float(self.get_parameter("publish_fps").value)
        self._retry_period = Duration(seconds=float(self.get_parameter("retry_period_sec").value))
        self._frame_id = str(self.get_parameter("frame_id").value)
        image_topic = str(self.get_parameter("image_topic").value)

        self._bridge = CvBridge()
        self._cap: cv2.VideoCapture | None = None
        self._next_retry_time = self.get_clock().now()
        self._camera_ok = self._open_camera()
        if not self._camera_ok:
            self.get_logger().error(
                f"Failed to open camera index {self._camera_index} "
                f"(/dev/video{self._camera_index}). Will retry."
            )

        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._timer = self.create_timer(1.0 / max(1.0, publish_fps), self._timer_callback)

        self.get_logger().info(
            f"CameraSourceNode publishing {image_topic}: camera={self._camera_index} "
            f"{self._width}x{self._height}@{self._camera_fps}, publish_fps={publish_fps:.1f}"
        )

    def _open_camera(self) -> bool:
        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self._camera_index)
        if not cap.isOpened():
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._camera_fps)
        self._cap = cap
        return True

    def _timer_callback(self) -> None:
        if not self._camera_ok or self._cap is None:
            now = self.get_clock().now()
            if now < self._next_retry_time:
                return
            self._next_retry_time = now + self._retry_period
            self._camera_ok = self._open_camera()
            if not self._camera_ok:
                self.get_logger().warn("Camera is unavailable; retrying.", throttle_duration_sec=2.0)
                return
            self.get_logger().info("Camera re-opened successfully.")

        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn("cap.read() failed; will attempt camera re-open next tick.")
            self._camera_ok = False
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)

    def destroy_node(self) -> None:
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
