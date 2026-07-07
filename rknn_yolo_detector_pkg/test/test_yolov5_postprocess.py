import numpy as np

from rknn_yolo_detector_pkg.yolov5_postprocess import LetterboxInfo, YoloV5PostProcessor, parse_anchor_string


ANCHORS = parse_anchor_string("10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326")


def make_postprocessor(conf_threshold=0.001):
    return YoloV5PostProcessor(
        input_width=640,
        input_height=640,
        num_classes=5,
        conf_threshold=conf_threshold,
        nms_threshold=0.45,
        anchors=ANCHORS,
        max_detections=100,
    )


def assert_close(actual, expected):
    assert abs(actual - expected) < 1e-6


def test_flat_output_uses_class_max_confidence_when_objectness_is_low():
    postprocessor = make_postprocessor()
    output = np.zeros((1, 2, 10), dtype=np.float32)
    output[0, 0] = [320.0, 320.0, 100.0, 80.0, 0.0018, 0.01, 0.12, 0.02, 0.03, 0.04]
    output[0, 1] = [100.0, 100.0, 30.0, 40.0, 0.0018, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005]

    detections = postprocessor(
        [output],
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0, original_width=640, original_height=640),
    )

    assert len(detections) == 1
    assert detections[0].class_id == 1
    assert_close(detections[0].confidence, 0.12)
    assert_close(detections[0].x1, 270.0)
    assert_close(detections[0].y1, 280.0)
    assert_close(detections[0].x2, 370.0)
    assert_close(detections[0].y2, 360.0)


def test_flat_output_scales_letterboxed_boxes_back_to_original_image():
    postprocessor = make_postprocessor()
    output = np.zeros((1, 1, 10), dtype=np.float32)
    output[0, 0] = [320.0, 320.0, 100.0, 80.0, 0.0018, 0.01, 0.02, 0.13, 0.03, 0.04]

    detections = postprocessor(
        [output],
        LetterboxInfo(scale=0.5, pad_x=0.0, pad_y=140.0, original_width=1280, original_height=720),
    )

    assert len(detections) == 1
    assert detections[0].class_id == 2
    assert_close(detections[0].confidence, 0.13)
    assert_close(detections[0].x1, 540.0)
    assert_close(detections[0].y1, 280.0)
    assert_close(detections[0].x2, 740.0)
    assert_close(detections[0].y2, 440.0)
