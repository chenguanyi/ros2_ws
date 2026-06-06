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
        self.declare_parameter("camera_device", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("camera_fps", 60)
        self.declare_parameter("fourcc", "")
        self.declare_parameter("publish_fps", 30.0)
        self.declare_parameter("retry_period_sec", 1.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "/camera/image_raw")

        self._camera_index = int(self.get_parameter("camera_index").value)
        self._camera_device = str(self.get_parameter("camera_device").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._camera_fps = int(self.get_parameter("camera_fps").value)
        self._fourcc = str(self.get_parameter("fourcc").value).strip().upper()
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
                f"Failed to open camera {self._camera_label()}. Will retry."
            )

        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._timer = self.create_timer(1.0 / max(1.0, publish_fps), self._timer_callback)

        self.get_logger().info(
            f"CameraSourceNode publishing {image_topic}: camera={self._camera_label()} "
            f"{self._width}x{self._height}@{self._camera_fps}, fourcc={self._fourcc or 'driver-default'}, "
            f"publish_fps={publish_fps:.1f}"
        )

    def _camera_label(self) -> str:
        if self._camera_device:
            return self._camera_device
        return f"index {self._camera_index} (/dev/video{self._camera_index})"

    def _open_camera(self) -> bool:
        source = self._camera_device if self._camera_device else self._camera_index
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            return False

        if self._fourcc:
            if len(self._fourcc) != 4:
                self.get_logger().warn(
                    f"Ignoring invalid fourcc '{self._fourcc}'; expected exactly 4 characters."
                )
            else:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._camera_fps)
        self._cap = cap
        self._log_actual_camera_format(cap)
        return True

    def _log_actual_camera_format(self, cap: cv2.VideoCapture) -> None:
        fourcc_value = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera actual format: {width:.0f}x{height:.0f}@{fps:.1f}, fourcc={fourcc!r}"
        )

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
