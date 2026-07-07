import numpy as np

from rknn_yolo_detector_pkg.yolov5_postprocess import (
    Detection,
    LetterboxInfo,
    YoloV5PostProcessor,
    parse_anchor_string,
)


ANCHORS = parse_anchor_string("10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326")


def make_postprocessor(conf_threshold=0.001, fusion_enabled=False):
    return YoloV5PostProcessor(
        input_width=640,
        input_height=640,
        num_classes=5,
        conf_threshold=conf_threshold,
        nms_threshold=0.45,
        anchors=ANCHORS,
        max_detections=100,
        fusion_enabled=fusion_enabled,
        fusion_iou_threshold=0.5,
    )


def assert_close(actual, expected):
    assert abs(actual - expected) < 1e-6


def test_flat_output_objectness_multiplied_confidence():
    """Flat output uses objectness * cls_max as confidence.
    High-objectness cell with non-max cls produces correct score.
    Low-objectness cell is filtered out."""
    postprocessor = make_postprocessor()
    # Format: [cx, cy, w, h, obj, cls0, cls1, cls2, cls3, cls4]
    # Row 0: real detection with high objectness, class 1 (monkey) highest
    output = np.zeros((1, 2, 10), dtype=np.float32)
    output[0, 0] = [320.0, 320.0, 100.0, 80.0, 0.6, 0.01, 0.12, 0.02, 0.03, 0.04]
    # Row 1: background noise with low objectness
    output[0, 1] = [100.0, 100.0, 30.0, 40.0, 0.0018, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005]

    detections = postprocessor(
        [output],
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0, original_width=640, original_height=640),
    )

    # Only high-objectness cell should remain
    assert len(detections) == 1
    assert detections[0].class_id == 1
    # Expected score: 0.12 * 0.6 = 0.072
    assert_close(detections[0].confidence, 0.072)
    assert_close(detections[0].x1, 270.0)
    assert_close(detections[0].y1, 280.0)
    assert_close(detections[0].x2, 370.0)
    assert_close(detections[0].y2, 360.0)
    assert_close(detections[0].cx, 320.0)
    assert_close(detections[0].cy, 320.0)


def test_flat_output_low_objectness_filtered():
    """Flat output: cell with low objectness is filtered out even if cls_max is high."""
    postprocessor = make_postprocessor(conf_threshold=0.001)
    output = np.zeros((1, 1, 10), dtype=np.float32)
    # obj=0.0018 (background), cls_max=0.5 (spuriously high class peak)
    # score = 0.5 * 0.0018 = 0.0009 < 0.001 → filtered
    output[0, 0] = [320.0, 320.0, 100.0, 80.0, 0.0018, 0.01, 0.02, 0.50, 0.03, 0.04]

    detections = postprocessor(
        [output],
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0, original_width=640, original_height=640),
    )

    assert len(detections) == 0, "Low-objectness cell should be filtered"


def test_flat_output_scales_letterboxed_boxes_back_to_original_image():
    postprocessor = make_postprocessor()
    output = np.zeros((1, 1, 10), dtype=np.float32)
    # Row: [cx, cy, w, h, obj, cls0, cls1, cls2, cls3, cls4]
    # class 2 (peacock) with realistic objectness
    output[0, 0] = [320.0, 320.0, 100.0, 80.0, 0.5, 0.01, 0.02, 0.13, 0.03, 0.04]

    detections = postprocessor(
        [output],
        LetterboxInfo(scale=0.5, pad_x=0.0, pad_y=140.0, original_width=1280, original_height=720),
    )

    assert len(detections) == 1
    assert detections[0].class_id == 2
    # Expected score: 0.13 * 0.5 = 0.065
    assert_close(detections[0].confidence, 0.065)
    assert_close(detections[0].x1, 540.0)
    assert_close(detections[0].y1, 280.0)
    assert_close(detections[0].x2, 740.0)
    assert_close(detections[0].y2, 440.0)
    assert_close(detections[0].cx, 640.0)
    assert_close(detections[0].cy, 360.0)


def make_detection(class_id, confidence, cx, cy, w, h):
    """Helper to create a Detection dataclass for fusion tests."""
    return Detection(
        class_id=class_id,
        confidence=confidence,
        cx=cx, cy=cy,
        x1=cx - w / 2.0,
        y1=cy - h / 2.0,
        x2=cx + w / 2.0,
        y2=cy + h / 2.0,
    )


def test_fusion_merges_overlapping_boxes_same_class():
    """Multiple overlapping boxes of same class should fuse into one."""
    postprocessor = make_postprocessor(fusion_enabled=True)
    # Simulate the output of NMS: 3 overlapping boxes of class 1 (monkey)
    dets = [
        make_detection(1, 0.95, 320, 240, 100, 80),   # center
        make_detection(1, 0.85, 330, 245, 110, 75),   # shifted right
        make_detection(1, 0.70, 315, 235, 95, 85),    # shifted up-left
    ]
    fused = postprocessor._fuse_per_class(dets)
    assert len(fused) == 1, f"Expected 1 fused detection, got {len(fused)}"
    assert fused[0].class_id == 1
    # Center should be weighted toward the highest confidence box
    assert 315 < fused[0].cx < 330
    assert 235 < fused[0].cy < 245
    # Confidence should be the max of the cluster
    assert_close(fused[0].confidence, 0.95)


def test_fusion_keeps_separate_objects_different_class():
    """Boxes of different classes should remain separate after fusion."""
    postprocessor = make_postprocessor(fusion_enabled=True)
    dets = [
        make_detection(1, 0.95, 320, 240, 100, 80),   # class 1
        make_detection(2, 0.90, 150, 150, 80, 60),    # class 2 (non-overlapping)
    ]
    fused = postprocessor._fuse_per_class(dets)
    assert len(fused) == 2, f"Expected 2 fused detections (different classes), got {len(fused)}"


def test_fusion_keeps_far_boxes_separate_same_class():
    """Same-class boxes with low IoU that form separate clusters — only the best cluster is kept."""
    postprocessor = make_postprocessor(fusion_enabled=True)
    dets = [
        make_detection(3, 0.95, 500, 300, 100, 80),   # cluster A (high conf)
        make_detection(3, 0.85, 510, 310, 90, 70),    # cluster A (overlaps with seed, IOU > 0.5)
        make_detection(3, 0.60, 100, 100, 80, 60),    # cluster B (far away, low IoU)
    ]
    fused = postprocessor._fuse_per_class(dets)
    assert len(fused) == 1, f"Expected 1 fused detection (cluster A only), got {len(fused)}"
    assert fused[0].class_id == 3
    # Fused box should be centered on cluster A (around 505, 305)
    assert 490 < fused[0].cx < 520
    assert 295 < fused[0].cy < 315
    # Confidence is max of cluster A
    assert_close(fused[0].confidence, 0.95)
