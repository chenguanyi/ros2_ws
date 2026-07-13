#!/usr/bin/env python3
"""
Batch inference script for RKNN YOLOv5 on Orange Pi RK3588.
Runs all test images through the FP16 model and reports detection statistics.
"""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


# ── Postprocessing (copied from yolov5_postprocess.py, no ROS deps) ──────────

@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    cx: float
    cy: float
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


def letterbox(image: np.ndarray, width: int, height: int,
              color: tuple[int, int, int] = (114, 114, 114)) -> tuple[np.ndarray, LetterboxInfo]:
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
    return canvas, LetterboxInfo(scale=scale, pad_x=float(left), pad_y=float(top),
                                 original_width=src_w, original_height=src_h)


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
        fusion_enabled: bool = False,
        fusion_iou_threshold: float = 0.5,
    ) -> None:
        self.input_width = input_width
        self.input_height = input_height
        self.num_classes = num_classes
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors.astype(np.float32)
        self.strides = tuple(int(v) for v in strides)
        self.max_detections = max_detections
        self.fusion_enabled = fusion_enabled
        self.fusion_iou_threshold = fusion_iou_threshold

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
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                detections.append(Detection(
                    class_id=int(class_id),
                    confidence=float(scores[src_index]),
                    cx=cx, cy=cy,
                    x1=x1, y1=y1, x2=x2, y2=y2,
                ))

        if self.fusion_enabled and detections:
            detections = self._fuse_per_class(detections)

        detections.sort(key=lambda det: det.confidence, reverse=True)
        return detections[:self.max_detections]

    def _fuse_per_class(self, detections: list[Detection]) -> list[Detection]:
        """Weighted box fusion per class: cluster overlapping boxes, produce one per class."""
        by_class: dict[int, list[Detection]] = {}
        for det in detections:
            by_class.setdefault(det.class_id, []).append(det)

        fused: list[Detection] = []
        for class_id, class_dets in by_class.items():
            class_dets.sort(key=lambda d: d.confidence, reverse=True)
            used = [False] * len(class_dets)
            best_cluster: list[Detection] | None = None
            best_cluster_conf = 0.0

            for i, seed in enumerate(class_dets):
                if used[i]:
                    continue
                cluster: list[Detection] = [seed]
                used[i] = True
                for j in range(i + 1, len(class_dets)):
                    if used[j]:
                        continue
                    if self._box_iou(seed, class_dets[j]) >= self.fusion_iou_threshold:
                        cluster.append(class_dets[j])
                        used[j] = True

                cluster_peak = max(d.confidence for d in cluster)
                if cluster_peak > best_cluster_conf:
                    best_cluster = cluster
                    best_cluster_conf = cluster_peak

            if best_cluster is None or not best_cluster:
                continue

            total_w = sum(d.confidence for d in best_cluster)
            if total_w <= 0:
                best = max(best_cluster, key=lambda d: d.confidence)
                cx, cy = best.cx, best.cy
                x1, y1, x2, y2 = best.x1, best.y1, best.x2, best.y2
                conf = best.confidence
            else:
                cx = sum(d.confidence * d.cx for d in best_cluster) / total_w
                cy = sum(d.confidence * d.cy for d in best_cluster) / total_w
                x1 = sum(d.confidence * d.x1 for d in best_cluster) / total_w
                y1 = sum(d.confidence * d.y1 for d in best_cluster) / total_w
                x2 = sum(d.confidence * d.x2 for d in best_cluster) / total_w
                y2 = sum(d.confidence * d.y2 for d in best_cluster) / total_w
                conf = max(d.confidence for d in best_cluster)

            fused.append(Detection(
                class_id=class_id, confidence=conf,
                cx=cx, cy=cy, x1=x1, y1=y1, x2=x2, y2=y2,
            ))

        return fused

    def _box_iou(self, a: Detection, b: Detection) -> float:
        ax1, ay1, ax2, ay2 = a.x1, a.y1, a.x2, a.y2
        bx1, by1, bx2, by2 = b.x1, b.y1, b.x2, b.y2
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter = inter_w * inter_h
        area_a = (ax2 - ax1) * (ay2 - ay1)
        area_b = (bx2 - bx1) * (by2 - by1)
        union = area_a + area_b - inter
        if union <= 0:
            return 0.0
        return inter / union

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
        cls_probs = np.clip(flat[:, 5:5 + self.num_classes], 0.0, 1.0)
        cls_ids = cls_probs.argmax(axis=1).astype(np.float32)
        cls_max = cls_probs.max(axis=1)
        # Standard YOLO confidence: objectness × class_prob.
        # Even if objectness is low/constant (~0.0018), multiplying it
        # suppresses background grid cells where cls_max is spuriously high.
        obj = np.clip(flat[:, 4], 0.0, 1.0)
        scores = cls_max * obj
        return np.column_stack([boxes, scores, cls_ids])

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


# ── Inference ──────────────────────────────────────────────────────────────

def load_model(model_path: str, core_mask: int):
    from rknnlite.api import RKNNLite
    rknn = RKNNLite(verbose=False)
    ret = rknn.load_rknn(model_path)
    if ret != 0:
        raise RuntimeError(f"load_rknn failed, ret={ret}")
    ret = rknn.init_runtime(core_mask=core_mask)
    if ret != 0:
        raise RuntimeError(f"init_runtime failed, ret={ret}")
    return rknn


def run_inference(
    rknn,
    postprocessor: YoloV5PostProcessor,
    image_bgr: np.ndarray,
    input_width: int,
    input_height: int,
    input_color: str = "rgb",
) -> tuple[list[Detection], float, float, float]:
    t0 = time.monotonic()
    input_img, letterbox_info = letterbox(image_bgr, input_width, input_height)
    if input_color == "rgb":
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
    input_tensor = np.expand_dims(np.ascontiguousarray(input_img, dtype=np.uint8), axis=0)
    t1 = time.monotonic()
    outputs = rknn.inference(inputs=[input_tensor])
    t2 = time.monotonic()
    if outputs is None:
        raise RuntimeError("inference returned None")
    detections = postprocessor([np.asarray(output) for output in outputs], letterbox_info)
    t3 = time.monotonic()
    return detections, (t1 - t0) * 1000, (t2 - t1) * 1000, (t3 - t2) * 1000


# ── Stats ───────────────────────────────────────────────────────────────────

@dataclass
class ImageResult:
    filename: str
    detections: list[Detection]
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float
    annotated_path: str | None = None


CLASS_NAMES = ["elephant", "monkey", "peacock", "tiger", "wolf"]


def draw_detections(image: np.ndarray, detections: list[Detection], class_names: list[str]) -> np.ndarray:
    vis = image.copy()
    for det in detections:
        c = (0, 255, 0)
        cv2.rectangle(vis, (int(det.x1), int(det.y1)), (int(det.x2), int(det.y2)), c, 2)
        label = f"{class_names[det.class_id]}:{det.confidence:.3f}"
        cv2.putText(vis, label, (int(det.x1), max(5, int(det.y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
    return vis


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Batch RKNN YOLO inference")
    parser.add_argument("--model", default="/home/orangepi/models/h2025_yolov5s_fp.rknn",
                        help="RKNN model path")
    parser.add_argument("--classes", default="/home/orangepi/models/classes.txt",
                        help="Classes text file")
    parser.add_argument("--images", default="./test_images",
                        help="Directory of test images")
    parser.add_argument("--output", default="./results",
                        help="Output directory for annotated images and stats")
    parser.add_argument("--input-width", type=int, default=640)
    parser.add_argument("--input-height", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.001,
                        help="Confidence threshold")
    parser.add_argument("--nms", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=100)
    parser.add_argument("--num-classes", type=int, default=5)
    parser.add_argument("--anchors",
                        default="10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326")
    parser.add_argument("--core", type=int, default=0,
                        help="NPU core: 0, 1, 2, or 3 for all")
    parser.add_argument("--fusion", action="store_true",
                        help="Enable per-class weighted box fusion")
    parser.add_argument("--fusion-iou", type=float, default=0.5,
                        help="IoU threshold for fusion clustering")
    args = parser.parse_args()

    # Load class names
    with open(args.classes, "r", encoding="utf-8") as f:
        class_names = [l.strip() for l in f if l.strip()]
    print(f"Classes ({len(class_names)}): {class_names}", flush=True)

    # Setup postprocessor
    anchors = parse_anchor_string(args.anchors)
    postprocessor = YoloV5PostProcessor(
        input_width=args.input_width,
        input_height=args.input_height,
        num_classes=args.num_classes,
        conf_threshold=args.conf,
        nms_threshold=args.nms,
        anchors=anchors,
        max_detections=args.max_det,
        fusion_enabled=args.fusion,
        fusion_iou_threshold=args.fusion_iou,
    )

    # Map core mask
    from rknnlite.api import RKNNLite
    core_map = {0: RKNNLite.NPU_CORE_0, 1: RKNNLite.NPU_CORE_1,
                2: RKNNLite.NPU_CORE_2, 3: RKNNLite.NPU_CORE_0_1_2}
    core_mask = core_map.get(args.core, RKNNLite.NPU_CORE_0)

    # Load model
    print(f"Loading model: {args.model}", flush=True)
    rknn = load_model(args.model, core_mask)
    print(f"Model loaded. Running inference...", flush=True)

    # Collect images
    img_dir = Path(args.images)
    image_paths = sorted(img_dir.glob("*.[jJ][pP][gG]")) + sorted(img_dir.glob("*.[pP][nN][gG]"))
    if not image_paths:
        print(f"No images found in {img_dir}", flush=True)
        return

    print(f"Found {len(image_paths)} images", flush=True)

    # Output dirs
    out_dir = Path(args.output)
    annotated_dir = out_dir / "annotated"
    annotated_dir.mkdir(parents=True, exist_ok=True)

    # Run inference
    results: list[ImageResult] = []
    total_preproc = total_infer = total_postproc = 0.0

    for img_path in image_paths:
        image_bgr = cv2.imread(str(img_path))
        if image_bgr is None:
            print(f"  SKIP (cannot read): {img_path.name}", flush=True)
            continue

        dets, prep_ms, inf_ms, post_ms = run_inference(
            rknn, postprocessor, image_bgr,
            args.input_width, args.input_height,
        )

        total_preproc += prep_ms
        total_infer += inf_ms
        total_postproc += post_ms

        # Annotate if there are detections
        annotated = None
        if dets:
            vis = draw_detections(image_bgr, dets, class_names)
            out_path = str(annotated_dir / img_path.name)
            cv2.imwrite(out_path, vis)
            annotated = out_path

        results.append(ImageResult(
            filename=img_path.name,
            detections=dets,
            preprocess_ms=prep_ms,
            inference_ms=inf_ms,
            postprocess_ms=post_ms,
            annotated_path=annotated,
        ))

        log = f"  {img_path.name}: {len(dets)} dets "
        log += f"(prep={prep_ms:.1f}ms infer={inf_ms:.1f}ms post={post_ms:.1f}ms)"
        if dets:
            top = dets[0]
            log += f" top={class_names[top.class_id]}:{top.confidence:.4f}"
        print(log, flush=True)

    rknn.release()

    # ── Statistics ──────────────────────────────────────────────────────
    n = len(results)
    print(f"\n{'='*60}", flush=True)
    print(f"SUMMARY: {n} images processed", flush=True)
    n_det = sum(1 for r in results if r.detections)
    print(f"Images with detections: {n_det}/{n} ({100*n_det/n:.1f}%)", flush=True)
    total_dets = sum(len(r.detections) for r in results)
    print(f"Total detections: {total_dets}", flush=True)
    print(f"Avg preprocess: {total_preproc/n:.1f}ms  "
          f"Avg inference: {total_infer/n:.1f}ms  "
          f"Avg postprocess: {total_postproc/n:.1f}ms", flush=True)

    # Per-class
    class_counts: dict[int, int] = {}
    class_confidences: dict[int, list[float]] = {}
    for r in results:
        for det in r.detections:
            class_counts[det.class_id] = class_counts.get(det.class_id, 0) + 1
            class_confidences.setdefault(det.class_id, []).append(det.confidence)

    print(f"\nPer-class detection counts:", flush=True)
    for cid in sorted(class_counts.keys()):
        confs = class_confidences.get(cid, [])
        avg_conf = sum(confs) / len(confs) if confs else 0.0
        max_conf = max(confs) if confs else 0.0
        name = class_names[cid] if cid < len(class_names) else f"class-{cid}"
        print(f"  {name}({cid}): {class_counts[cid]} boxes  "
              f"avg_conf={avg_conf:.4f}  max_conf={max_conf:.4f}", flush=True)

    # Per-image detection distribution
    det_counts = [len(r.detections) for r in results]
    if det_counts:
        import statistics
        print(f"\nDetections per image:", flush=True)
        print(f"  min={min(det_counts)}  max={max(det_counts)}  "
              f"mean={statistics.mean(det_counts):.1f}  "
              f"median={statistics.median(det_counts):.1f}", flush=True)

    # Ground-truth label parsing from filenames
    # Format: NN_动物名_组_augNN.jpg
    print(f"\nBy ground-truth label (from filename):", flush=True)
    gt_results: dict[str, dict] = {}  # animal -> list of (detected_class_ids, tot_dets)
    for r in results:
        parts = r.filename.split("_")
        if len(parts) >= 2:
            gt_animal = parts[1]
            gt_results.setdefault(gt_animal, {"count": 0, "with_any_det": 0,
                                              "correct": 0, "gt_idx": None})
    # Map GT animal names to class ids
    gt_to_cid: dict[str, int] = {}
    for i, name in enumerate(class_names):
        gt_to_cid[name] = i
    # Fill
    for r in results:
        parts = r.filename.split("_")
        if len(parts) < 2:
            continue
        gt_animal = parts[1]
        if gt_animal not in gt_results:
            continue
        gt_results[gt_animal]["count"] += 1
        if r.detections:
            gt_results[gt_animal]["with_any_det"] += 1
        gt_cid = gt_to_cid.get(gt_animal)
        if gt_cid is not None:
            correct = sum(1 for d in r.detections if d.class_id == gt_cid)
            if correct > 0:
                gt_results[gt_animal]["correct"] += 1

    for animal, stats in gt_results.items():
        if stats["count"] == 0:
            continue
        pct_det = 100 * stats["with_any_det"] / stats["count"]
        pct_correct = 100 * stats["correct"] / stats["count"] if gt_to_cid.get(animal) is not None else 0
        print(f"  {animal}: {stats['count']} images, "
              f"any_det={stats['with_any_det']}/{stats['count']} ({pct_det:.0f}%), "
              f"correct_class={stats['correct']}/{stats['count']} ({pct_correct:.0f}%)", flush=True)

    # Save JSON report
    report = {
        "total_images": n,
        "images_with_detections": n_det,
        "total_detections": total_dets,
        "avg_preprocess_ms": total_preproc / n,
        "avg_inference_ms": total_infer / n,
        "avg_postprocess_ms": total_postproc / n,
        "per_class": {
            class_names[cid] if cid < len(class_names) else f"class-{cid}": {
                "count": class_counts[cid],
                "avg_confidence": float(np.mean(class_confidences[cid])) if cid in class_confidences else 0,
                "max_confidence": float(np.max(class_confidences[cid])) if cid in class_confidences else 0,
            }
            for cid in sorted(class_counts.keys())
        },
        "per_image": [
            {"filename": r.filename, "detections": len(r.detections),
             "top_class": class_names[r.detections[0].class_id] if r.detections else None,
             "top_conf": r.detections[0].confidence if r.detections else 0}
            for r in results
        ],
    }
    report_path = out_dir / "report.json"
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\nReport saved to {report_path}", flush=True)
    print(f"Annotated images in {annotated_dir}", flush=True)


if __name__ == "__main__":
    main()
