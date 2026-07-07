from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from rknn_yolo_detector_pkg.yolov5_postprocess import Detection, YoloV5PostProcessor, letterbox, parse_anchor_string


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off", ""}:
            return False
    return bool(value)


def _as_int(value: Any) -> int:
    return int(str(value).strip())


def _as_float(value: Any) -> float:
    return float(str(value).strip())


@dataclass(frozen=True)
class FrameJob:
    sequence: int
    stamp: Any
    frame_id: str
    image_bgr: np.ndarray


@dataclass(frozen=True)
class WorkerResult:
    worker_index: int
    sequence: int
    detections: list[Detection]
    debug_image: Image | None
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float


class RknnYoloWorker(threading.Thread):
    def __init__(
        self,
        node: Node,
        worker_index: int,
        core_mask: int,
        core_name: str,
        model_path: str,
        input_width: int,
        input_height: int,
        postprocessor: YoloV5PostProcessor,
        input_color: str,
        job_queue: queue.Queue[FrameJob],
        result_queue: queue.Queue[WorkerResult],
        stop_event: threading.Event,
        publish_debug_image: bool,
        bridge: CvBridge,
    ) -> None:
        super().__init__(name=f"rknn-yolo-{core_name}", daemon=True)
        self._node = node
        self._worker_index = worker_index
        self._core_mask = core_mask
        self._core_name = core_name
        self._model_path = model_path
        self._input_width = input_width
        self._input_height = input_height
        self._postprocessor = postprocessor
        self._input_color = input_color
        self._job_queue = job_queue
        self._result_queue = result_queue
        self._stop_event = stop_event
        self._publish_debug_image = publish_debug_image
        self._bridge = bridge
        self._rknn = None
        self.startup_error: Exception | None = None
        self.ready = threading.Event()

    def run(self) -> None:
        try:
            self._init_rknn_runtime()
        except Exception as exc:  # noqa: BLE001 - startup error is surfaced to the ROS node
            self.startup_error = exc
            self.ready.set()
            return

        self.ready.set()
        while not self._stop_event.is_set():
            try:
                job = self._job_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                result = self._process_job(job)
                self._result_queue.put_nowait(result)
            except Exception as exc:  # noqa: BLE001 - keep worker alive after one bad frame
                self._node.get_logger().error(
                    f"RKNN worker {self._core_name} failed on frame {job.sequence}: {exc}"
                )
            finally:
                self._job_queue.task_done()

        if self._rknn is not None:
            self._rknn.release()
            self._rknn = None

    def _init_rknn_runtime(self) -> None:
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError(
                "Failed to import rknnlite.api. This detector must run on the RK3588 board "
                "with rknn-toolkit-lite2 installed; no CPU fallback is used."
            ) from exc

        rknn = RKNNLite(verbose=False)
        ret = rknn.load_rknn(self._model_path)
        if ret != 0:
            raise RuntimeError(f"RKNN load_rknn failed on {self._core_name}, ret={ret}, model={self._model_path}")

        ret = rknn.init_runtime(core_mask=self._core_mask)
        if ret != 0:
            raise RuntimeError(f"RKNN init_runtime failed on {self._core_name}, ret={ret}")

        self._rknn = rknn
        self._node.get_logger().info(
            f"RKNN worker {self._worker_index} initialized on {self._core_name}: {self._model_path}"
        )

    def _process_job(self, job: FrameJob) -> WorkerResult:
        preprocess_start = time.monotonic()
        input_tensor, letterbox_info = self._prepare_input(job.image_bgr)
        preprocess_ms = (time.monotonic() - preprocess_start) * 1000.0

        inference_start = time.monotonic()
        outputs = self._rknn.inference(inputs=[input_tensor])
        inference_ms = (time.monotonic() - inference_start) * 1000.0
        if outputs is None:
            raise RuntimeError("RKNN inference returned None")

        postprocess_start = time.monotonic()
        detections = self._postprocessor([np.asarray(output) for output in outputs], letterbox_info)
        debug_msg = self._make_debug_image(job, detections) if self._publish_debug_image else None
        postprocess_ms = (time.monotonic() - postprocess_start) * 1000.0

        return WorkerResult(
            worker_index=self._worker_index,
            sequence=job.sequence,
            detections=detections,
            debug_image=debug_msg,
            preprocess_ms=preprocess_ms,
            inference_ms=inference_ms,
            postprocess_ms=postprocess_ms,
        )

    def _prepare_input(self, image_bgr: np.ndarray) -> tuple[np.ndarray, Any]:
        image, letterbox_info = letterbox(image_bgr, self._input_width, self._input_height)
        if self._input_color == "rgb":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif self._input_color != "bgr":
            raise RuntimeError(f"unsupported input_color '{self._input_color}', expected rgb or bgr")
        return np.expand_dims(np.ascontiguousarray(image, dtype=np.uint8), axis=0), letterbox_info

    def _make_debug_image(self, job: FrameJob, detections: list[Detection]) -> Image:
        image = job.image_bgr.copy()
        for det in detections:
            cv2.rectangle(
                image,
                (int(round(det.x1)), int(round(det.y1))),
                (int(round(det.x2)), int(round(det.y2))),
                (0, 255, 0),
                2,
            )
            label = f"{det.class_id}:{det.confidence:.2f}"
            cv2.putText(
                image,
                label,
                (int(round(det.x1)), max(0, int(round(det.y1)) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header.stamp = job.stamp
        msg.header.frame_id = job.frame_id
        return msg


class RknnYoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("rknn_yolo_detector_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("classes_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("debug_image_topic", "/yolo/debug_image")
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("num_classes", 0)
        self.declare_parameter("input_color", "rgb")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("max_detections", 100)
        self.declare_parameter(
            "anchors",
            "10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326",
        )
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("queue_size", 3)
        self.declare_parameter("log_period_sec", 1.0)

        self._model_path = str(self.get_parameter("model_path").value).strip()
        classes_path = str(self.get_parameter("classes_path").value).strip()
        image_topic = str(self.get_parameter("image_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        input_width = _as_int(self.get_parameter("input_width").value)
        input_height = _as_int(self.get_parameter("input_height").value)
        input_color = str(self.get_parameter("input_color").value).strip().lower()
        conf_threshold = _as_float(self.get_parameter("conf_threshold").value)
        nms_threshold = _as_float(self.get_parameter("nms_threshold").value)
        max_detections = _as_int(self.get_parameter("max_detections").value)
        anchors = parse_anchor_string(str(self.get_parameter("anchors").value))
        self._publish_debug_image = _as_bool(self.get_parameter("publish_debug_image").value)
        queue_size = _as_int(self.get_parameter("queue_size").value)
        self._log_period_sec = _as_float(self.get_parameter("log_period_sec").value)

        if not self._model_path:
            raise RuntimeError("model_path is required; pass the .rknn path in launch arguments")
        if input_width <= 0 or input_height <= 0:
            raise RuntimeError("input_width/input_height must be positive")
        if queue_size < 1:
            raise RuntimeError("queue_size must be >= 1")

        num_classes = _as_int(self.get_parameter("num_classes").value)
        self._class_names = self._load_class_names(classes_path)
        if num_classes <= 0:
            num_classes = len(self._class_names)
        if num_classes <= 0:
            raise RuntimeError("num_classes must be set when classes_path is empty")

        self._bridge = CvBridge()
        self._job_queue: queue.Queue[FrameJob] = queue.Queue(maxsize=queue_size)
        self._result_queue: queue.Queue[WorkerResult] = queue.Queue()
        self._stop_event = threading.Event()
        self._sequence = 0
        self._dropped_frames = 0
        self._last_log_time = time.monotonic()
        self._last_output_shapes: tuple[tuple[int, ...], ...] | None = None

        postprocessor = YoloV5PostProcessor(
            input_width=input_width,
            input_height=input_height,
            num_classes=num_classes,
            conf_threshold=conf_threshold,
            nms_threshold=nms_threshold,
            anchors=anchors,
            max_detections=max_detections,
        )

        self._detections_pub = self.create_publisher(Float32MultiArray, detections_topic, 10)
        self._debug_pub = self.create_publisher(Image, debug_image_topic, 10) if self._publish_debug_image else None
        self._workers = self._start_workers(
            model_path=self._model_path,
            input_width=input_width,
            input_height=input_height,
            postprocessor=postprocessor,
            input_color=input_color,
        )
        self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self._result_timer = self.create_timer(0.01, self._publish_ready_results)

        self.get_logger().info(
            "RKNN YOLO detector started: "
            f"image_topic={image_topic}, detections_topic={detections_topic}, "
            f"input={input_width}x{input_height}, classes={num_classes}, workers=3, "
            "NPU cores=CORE_0/CORE_1/CORE_2, no CPU fallback"
        )

    def _load_class_names(self, classes_path: str) -> list[str]:
        if not classes_path:
            return []
        with open(classes_path, "r", encoding="utf-8") as file:
            return [line.strip() for line in file if line.strip()]

    def _start_workers(
        self,
        model_path: str,
        input_width: int,
        input_height: int,
        postprocessor: YoloV5PostProcessor,
        input_color: str,
    ) -> list[RknnYoloWorker]:
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError(
                "Failed to import rknnlite.api. This detector must run on the RK3588 board "
                "with rknn-toolkit-lite2 installed; no CPU fallback is used."
            ) from exc

        core_specs = [
            (RKNNLite.NPU_CORE_0, "NPU_CORE_0"),
            (RKNNLite.NPU_CORE_1, "NPU_CORE_1"),
            (RKNNLite.NPU_CORE_2, "NPU_CORE_2"),
        ]
        workers: list[RknnYoloWorker] = []
        for index, (core_mask, core_name) in enumerate(core_specs):
            worker = RknnYoloWorker(
                node=self,
                worker_index=index,
                core_mask=core_mask,
                core_name=core_name,
                model_path=model_path,
                input_width=input_width,
                input_height=input_height,
                postprocessor=postprocessor,
                input_color=input_color,
                job_queue=self._job_queue,
                result_queue=self._result_queue,
                stop_event=self._stop_event,
                publish_debug_image=self._publish_debug_image,
                bridge=self._bridge,
            )
            worker.start()
            workers.append(worker)

        for worker in workers:
            worker.ready.wait(timeout=15.0)
            if not worker.ready.is_set():
                self._stop_event.set()
                raise RuntimeError(f"RKNN worker {worker.name} startup timed out")
            if worker.startup_error is not None:
                self._stop_event.set()
                raise worker.startup_error

        return workers

    def _image_callback(self, msg: Image) -> None:
        try:
            image_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - ROS image conversion errors should be logged
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        self._sequence += 1
        job = FrameJob(
            sequence=self._sequence,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
            image_bgr=image_bgr,
        )
        if self._job_queue.full():
            try:
                self._job_queue.get_nowait()
                self._job_queue.task_done()
                self._dropped_frames += 1
            except queue.Empty:
                pass
        try:
            self._job_queue.put_nowait(job)
        except queue.Full:
            self._dropped_frames += 1

    def _publish_ready_results(self) -> None:
        while True:
            try:
                result = self._result_queue.get_nowait()
            except queue.Empty:
                break

            self._publish_detections(result)
            if result.debug_image is not None and self._debug_pub is not None:
                self._debug_pub.publish(result.debug_image)
            self._maybe_log_result(result)
            self._result_queue.task_done()

    def _publish_detections(self, result: WorkerResult) -> None:
        msg = Float32MultiArray()
        msg.data = []
        for det in result.detections:
            msg.data.extend([
                float(det.class_id),
                float(det.confidence),
                float(det.x1),
                float(det.y1),
                float(det.x2),
                float(det.y2),
            ])
        self._detections_pub.publish(msg)

    def _maybe_log_result(self, result: WorkerResult) -> None:
        now = time.monotonic()
        if now - self._last_log_time < self._log_period_sec:
            return
        self._last_log_time = now
        self.get_logger().info(
            f"frame={result.sequence} worker={result.worker_index} detections={len(result.detections)} "
            f"preprocess={result.preprocess_ms:.1f}ms inference={result.inference_ms:.1f}ms "
            f"postprocess={result.postprocess_ms:.1f}ms dropped={self._dropped_frames}"
        )

    def destroy_node(self) -> None:
        self._stop_event.set()
        for worker in getattr(self, "_workers", []):
            worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RknnYoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
