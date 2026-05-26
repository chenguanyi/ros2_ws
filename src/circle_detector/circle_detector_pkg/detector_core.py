from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectorConfig:
    # Tracking params (defaults chosen conservatively)
    track_pred_decay: float = 0.9
    track_pred_max_step_ratio: float = 0.5
    track_gate_base: float = 10.0
    track_gate_miss_gain: float = 1.0
    track_gate_speed_gain: float = 1.0
    track_gate_speed_norm: float = 1.0
    track_size_ratio_min: float = 0.01
    track_size_ratio_max: float = 0.8
    track_size_ratio_min_miss: float = 0.005
    track_size_ratio_max_miss: float = 1.0
    track_velocity_blend_old: float = 0.8
    track_switch_required_hits: int = 3
    track_pending_iou_min: float = 0.1

    # Ratio filter params
    circle_ratio_ema_alpha: float = 0.3
    ratio_outlier_percent: float = 0.25
    ratio_window_size: int = 5
    ratio_confidence_min: float = 0.2

    # Detection tolerance
    circle_detect_miss_tolerance: int = 3
    circle_hold_frames: int = 5

    # Visualization / threshold
    threshold_method: str = 'otsu'
    # Hough / circle detection params
    hough_dp: float = 1.2
    hough_min_dist: float = 20.0
    hough_param1: float = 100.0
    hough_param2: float = 30.0
    hough_min_radius: int = 4
    hough_max_radius: int = 200


class RobustRatioFilter:
    def __init__(self, alpha=0.3, outlier_percent=0.25, window_size=5, confidence_min=0.2):
        self.alpha = alpha
        self.window_size = max(1, int(window_size))
        self.confidence_min = confidence_min
        self._ema: Optional[float] = None

    def update(self, ratio: float, confidence: float) -> Optional[float]:
        if confidence < self.confidence_min:
            return None
        if self._ema is None:
            self._ema = float(ratio)
        else:
            self._ema = (1 - self.alpha) * self._ema + self.alpha * float(ratio)
        return self._ema

    def reset(self) -> None:
        self._ema = None


class DetectorState:
    def __init__(self):
        self.polygon: Optional[np.ndarray] = None
        self.misses: int = 0
        self.stable_hits: int = 0


class StableRectangleTracker:
    def __init__(
        self,
        pred_decay: float = 0.9,
        pred_max_step_ratio: float = 0.5,
        gate_base: float = 10.0,
        gate_miss_gain: float = 1.0,
        gate_speed_gain: float = 1.0,
        gate_speed_norm: float = 1.0,
        size_ratio_min: float = 0.01,
        size_ratio_max: float = 0.8,
        size_ratio_min_miss: float = 0.005,
        size_ratio_max_miss: float = 1.0,
        velocity_blend_old: float = 0.8,
        switch_required_hits: int = 3,
        pending_iou_min: float = 0.1,
    ) -> None:
        self.required_hits = max(1, int(switch_required_hits))
        self._state = DetectorState()

    def update(self, detection: Any) -> Tuple[DetectorState, bool]:
        # detection is expected to be a dict-like with optional 'polygon'
        poly = None
        if isinstance(detection, dict):
            poly = detection.get('polygon')
        if poly is not None:
            self._state.polygon = np.asarray(poly)
            self._state.stable_hits = min(self._state.stable_hits + 1, self.required_hits)
            self._state.misses = 0
            return self._state, True
        else:
            self._state.misses += 1
            if self._state.misses > 0:
                self._state.stable_hits = max(0, self._state.stable_hits - 1)
            return self._state, False


def preprocess(frame: np.ndarray, config: DetectorConfig) -> Tuple[np.ndarray, np.ndarray]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if config.threshold_method == 'otsu':
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    return gray, (binary > 0).astype(np.uint8)


def build_black_mask(frame: np.ndarray, config: DetectorConfig) -> np.ndarray:
    h, w = frame.shape[:2]
    return np.zeros((h, w), dtype=np.uint8)


def fuse_binary_masks(primary: np.ndarray, black: np.ndarray, config: DetectorConfig) -> np.ndarray:
    return np.clip(primary.astype(np.uint8) | black.astype(np.uint8), 0, 1)


def detect_rectangle_with_fallback(binary_fused: np.ndarray, binary_primary: np.ndarray, config: DetectorConfig) -> dict:
    # Find contours on the primary binary (prefer stronger edges)
    img = (binary_primary * 255).astype(np.uint8)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {'polygon': None}

    h, w = img.shape[:2]
    frame_area = float(max(1, h * w))
    min_area = config.track_size_ratio_min * frame_area
    max_area = config.track_size_ratio_max * frame_area

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            # ensure reasonable rectangle shape
            candidates.append((area, approx.reshape(-1, 2)))

    # if no quad found, try largest contour fallback
    if not candidates:
        contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
        for c in contours_sorted:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) >= 4:
                hull = cv2.convexHull(approx)
                if hull is not None and hull.shape[0] >= 4:
                    # take 4 extreme points via bounding rect
                    x, y, ww, hh = cv2.boundingRect(hull)
                    poly = np.array([[x, y], [x + ww, y], [x + ww, y + hh], [x, y + hh]], dtype=np.int32)
                    return {'polygon': poly}
        return {'polygon': None}

    # choose largest quad candidate
    candidates.sort(key=lambda t: t[0], reverse=True)
    poly = np.asarray(candidates[0][1], dtype=np.int32)
    return {'polygon': poly}


def detect_circle_in_square(binary_black: np.ndarray, polygon: np.ndarray, config: DetectorConfig) -> Optional[dict]:
    if polygon is None:
        return None
    # create mask of polygon
    h, w = binary_black.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)

    # apply mask to input binary to focus search
    search = (binary_black * 255).astype(np.uint8)
    masked = cv2.bitwise_and(search, mask)

    # find contours inside polygon
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # prefer the largest contour inside polygon
        c = max(contours, key=cv2.contourArea)
        area_c = cv2.contourArea(c)
        if area_c <= 0:
            return None
        # approximate circle via minEnclosingCircle
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        circle_area = np.pi * (radius ** 2)
        poly_area = abs(cv2.contourArea(polygon))
        if poly_area <= 0:
            return None
        ratio = float(circle_area / poly_area)
        # circularity metric
        peri = cv2.arcLength(c, True)
        circularity = 0.0
        if peri > 0:
            circularity = float((4.0 * np.pi * area_c) / (peri * peri))
        confidence = float(np.clip(circularity, 0.0, 1.0))
        return {
            'center': (float(cx), float(cy)),
            'radius': float(radius),
            'ratio': float(ratio),
            'confidence': confidence,
        }

    # fallback: use HoughCircles on the cropped polygon bounding box
    x, y, ww, hh = cv2.boundingRect(polygon.astype(np.int32))
    if ww <= 0 or hh <= 0:
        return None
    cropped = masked[y:y + hh, x:x + ww]
    if cropped.size == 0:
        return None
    gray = cv2.GaussianBlur(cropped, (5, 5), 0)
    # Hough expects 8-bit image
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=config.hough_dp,
        minDist=config.hough_min_dist,
        param1=config.hough_param1,
        param2=config.hough_param2,
        minRadius=config.hough_min_radius,
        maxRadius=config.hough_max_radius,
    )
    if circles is None:
        return None
    circles = np.round(circles[0, :]).astype(int)
    # pick the circle with largest radius
    best = max(circles, key=lambda c: c[2])
    cx = float(best[0] + x)
    cy = float(best[1] + y)
    radius = float(best[2])
    circle_area = np.pi * (radius ** 2)
    poly_area = abs(cv2.contourArea(polygon))
    if poly_area <= 0:
        return None
    ratio = float(circle_area / poly_area)
    # approximate confidence from radius vs box size
    confidence = float(np.clip(radius / max(1.0, min(ww, hh) / 2.0), 0.0, 1.0))
    return {
        'center': (cx, cy),
        'radius': radius,
        'ratio': ratio,
        'confidence': confidence,
    }


def draw_overlay(vis: np.ndarray, state: DetectorState, confirmed: bool, method: str, a: float, b: float, circle_for_pub: Optional[dict], circle_hold: bool) -> None:
    # Draw detected polygon
    if state is not None and state.polygon is not None:
        try:
            pts = state.polygon.reshape(-1, 2).astype(int)
            cv2.polylines(vis, [pts], True, (0, 255, 0) if confirmed else (0, 160, 255), 2)
        except Exception:
            pass
    # Draw circle if available
    if circle_for_pub is not None:
        cx, cy = int(round(circle_for_pub.get('center', (0, 0))[0])), int(round(circle_for_pub.get('center', (0, 0))[1]))
        r = int(round(circle_for_pub.get('radius', 0)))
        cv2.circle(vis, (cx, cy), r, (255, 0, 0), 2)
        cv2.circle(vis, (cx, cy), 2, (0, 0, 255), -1)
        # draw ratio text
        ratio_text = f"ratio={circle_for_pub.get('ratio', 0):.3f} conf={circle_for_pub.get('confidence', 0):.2f}"
        cv2.putText(vis, ratio_text, (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return
