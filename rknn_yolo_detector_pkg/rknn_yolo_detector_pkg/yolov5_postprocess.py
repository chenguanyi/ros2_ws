from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class LetterboxInfo:
    scale: float
    pad_x: float
    pad_y: float
    original_width: int
    original_height: int


def letterbox(image: np.ndarray, width: int, height: int, color: tuple[int, int, int] = (114, 114, 114)) -> tuple[np.ndarray, LetterboxInfo]:
    src_h, src_w = image.shape[:2]
    scale = min(width / src_w, height / src_h)
    resized_w = int(round(src_w * scale))
    resized_h = int(round(src_h * scale))

    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((height, width, 3), color, dtype=image.dtype)
    pad_x = (width - resized_w) / 2.0
    pad_y = (height - resized_h) / 2.0
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    canvas[top:top + resized_h, left:left + resized_w] = resized

    return canvas, LetterboxInfo(
        scale=scale,
        pad_x=float(left),
        pad_y=float(top),
        original_width=src_w,
        original_height=src_h,
    )


def parse_anchor_string(anchor_text: str) -> np.ndarray:
    values = [float(item.strip()) for item in anchor_text.replace(";", ",").split(",") if item.strip()]
    if len(values) != 18:
        raise ValueError("anchors must contain 18 numbers for YOLOv5 3 scales x 3 anchors x width/height")
    return np.asarray(values, dtype=np.float32).reshape(3, 3, 2)


class YoloV5PostProcessor:
    def __init__(
        self,
        input_width: int,
        input_height: int,
        num_classes: int,
        conf_threshold: float,
        nms_threshold: float,
        anchors: np.ndarray,
        strides: Iterable[int] = (8, 16, 32),
        max_detections: int = 100,
    ) -> None:
        self.input_width = input_width
        self.input_height = input_height
        self.num_classes = num_classes
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors.astype(np.float32)
        self.strides = tuple(int(v) for v in strides)
        self.max_detections = max_detections

        if self.anchors.shape != (3, 3, 2):
            raise ValueError(f"anchors must have shape (3, 3, 2), got {self.anchors.shape}")
        if len(self.strides) != 3:
            raise ValueError("YOLOv5 postprocess expects three strides")

    def __call__(self, outputs: list[np.ndarray], letterbox_info: LetterboxInfo) -> list[Detection]:
        decoded = self._decode_outputs(outputs)
        if decoded.size == 0:
            return []

        boxes = decoded[:, 0:4]
        if decoded.shape[1] == 6:
            scores = decoded[:, 4]
            class_ids = decoded[:, 5].astype(np.int32)
        else:
            objectness = decoded[:, 4]
            class_scores = decoded[:, 5:]
            class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
            class_conf = class_scores[np.arange(class_scores.shape[0]), class_ids]
            scores = objectness * class_conf

        keep = scores >= self.conf_threshold
        if not np.any(keep):
            return []

        boxes = boxes[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]

        detections: list[Detection] = []
        for class_id in np.unique(class_ids):
            indexes = np.where(class_ids == class_id)[0]
            class_boxes = boxes[indexes]
            class_scores_keep = scores[indexes]
            kept_indexes = self._nms(class_boxes, class_scores_keep)
            for kept in kept_indexes:
                src_index = indexes[kept]
                x1, y1, x2, y2 = self._scale_box_to_original(boxes[src_index], letterbox_info)
                detections.append(Detection(
                    class_id=int(class_id),
                    confidence=float(scores[src_index]),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                ))

        detections.sort(key=lambda det: det.confidence, reverse=True)
        return detections[:self.max_detections]

    def _decode_outputs(self, outputs: list[np.ndarray]) -> np.ndarray:
        if len(outputs) == 1:
            flat = np.asarray(outputs[0])
            if flat.ndim == 3 and flat.shape[0] == 1:
                flat = flat[0]
            if flat.ndim == 2 and flat.shape[-1] >= 6:
                return self._decode_flat_output(flat)

        if len(outputs) != 3:
            raise RuntimeError(f"standard YOLOv5 RKNN output expects 3 tensors, got {len(outputs)}")

        decoded_parts = []
        expected_channels = 3 * (self.num_classes + 5)
        for scale_index, raw in enumerate(outputs):
            output = np.asarray(raw)
            if output.ndim == 4 and output.shape[0] == 1:
                output = output[0]

            if output.ndim != 3:
                raise RuntimeError(f"YOLO output[{scale_index}] must be 3D after batch removal, got {output.shape}")

            if output.shape[0] == expected_channels:
                output = output.reshape(3, self.num_classes + 5, output.shape[1], output.shape[2])
                output = output.transpose(0, 2, 3, 1)
            elif output.shape[-1] == expected_channels:
                output = output.reshape(output.shape[0], output.shape[1], 3, self.num_classes + 5)
                output = output.transpose(2, 0, 1, 3)
            else:
                raise RuntimeError(
                    f"YOLO output[{scale_index}] shape {output.shape} does not match "
                    f"channels={expected_channels}; check num_classes/anchors"
                )

            decoded_parts.append(self._decode_scale(output, scale_index))

        return np.concatenate(decoded_parts, axis=0)

    def _decode_flat_output(self, output: np.ndarray) -> np.ndarray:
        flat = output.astype(np.float32, copy=False)
        cx = flat[:, 0:1]
        cy = flat[:, 1:2]
        w = flat[:, 2:3]
        h = flat[:, 3:4]
        boxes = np.concatenate([cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0], axis=1)
        # obj is constant for this model (post-export flattened format)
        # use class max directly as confidence
        cls_probs = np.clip(flat[:, 5:5 + self.num_classes], 0.0, 1.0)
        cls_ids = cls_probs.argmax(axis=1).astype(np.float32)
        cls_max = cls_probs.max(axis=1)
        return np.column_stack([boxes, cls_max, cls_ids])

    def _decode_scale(self, output: np.ndarray, scale_index: int) -> np.ndarray:
        anchors = self.anchors[scale_index]
        stride = self.strides[scale_index]
        _, grid_h, grid_w, _ = output.shape

        prediction = self._sigmoid(output.astype(np.float32, copy=False))
        grid_x, grid_y = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        grid = np.stack((grid_x, grid_y), axis=-1).astype(np.float32)

        xy = (prediction[..., 0:2] * 2.0 - 0.5 + grid[None, :, :, :]) * stride
        wh = np.square(prediction[..., 2:4] * 2.0) * anchors[:, None, None, :]
        objectness = prediction[..., 4:5]
        class_scores = prediction[..., 5:]

        x1y1 = xy - wh / 2.0
        x2y2 = xy + wh / 2.0
        boxes = np.concatenate((x1y1, x2y2), axis=-1)
        decoded = np.concatenate((boxes, objectness, class_scores), axis=-1)
        return decoded.reshape(-1, self.num_classes + 5)

    def _scale_box_to_original(self, box: np.ndarray, info: LetterboxInfo) -> tuple[float, float, float, float]:
        x1 = (float(box[0]) - info.pad_x) / info.scale
        y1 = (float(box[1]) - info.pad_y) / info.scale
        x2 = (float(box[2]) - info.pad_x) / info.scale
        y2 = (float(box[3]) - info.pad_y) / info.scale
        x1 = min(max(x1, 0.0), float(info.original_width - 1))
        y1 = min(max(y1, 0.0), float(info.original_height - 1))
        x2 = min(max(x2, 0.0), float(info.original_width - 1))
        y2 = min(max(y2, 0.0), float(info.original_height - 1))
        return x1, y1, x2, y2

    def _nms(self, boxes: np.ndarray, scores: np.ndarray) -> list[int]:
        if boxes.size == 0:
            return []

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep: list[int] = []

        while order.size > 0 and len(keep) < self.max_detections:
            current = int(order[0])
            keep.append(current)
            if order.size == 1:
                break

            xx1 = np.maximum(x1[current], x1[order[1:]])
            yy1 = np.maximum(y1[current], y1[order[1:]])
            xx2 = np.minimum(x2[current], x2[order[1:]])
            yy2 = np.minimum(y2[current], y2[order[1:]])
            inter_w = np.maximum(0.0, xx2 - xx1)
            inter_h = np.maximum(0.0, yy2 - yy1)
            inter = inter_w * inter_h
            union = areas[current] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-9)
            order = order[1:][iou <= self.nms_threshold]

        return keep

    @staticmethod
    def _sigmoid(value: np.ndarray) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-value))
