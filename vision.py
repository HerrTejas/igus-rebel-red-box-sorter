"""
Box detection and pixel -> robot coordinate mapping.

Detection (config vision.method = "color"): find boxes of a known colour (red)
by HSV hue.
    1. BGR -> HSV   2. inRange (red band, wrap-aware)   3. morphology
    4. contours   5. area + rectangularity filter   6. minAreaRect -> centre

Coordinate mapping:
    calibrate.py fits a 2x3 affine matrix M such that
        [X_robot, Y_robot]^T = M @ [u_pixel, v_pixel, 1]^T
    Camera looks straight down at a flat table, so image plane and robot XY
    plane are related by an affine map. Z is configured, not measured.
"""
import json

import cv2
import numpy as np


def apply_roi(mask, roi):
    """Zero out everything outside the ROI polygon so only the table area is
    searched. `roi` is a list of [x, y] pixel corners, or None (whole frame)."""
    if not roi:
        return mask
    roi_mask = np.zeros(mask.shape[:2], dtype=np.uint8)
    cv2.fillPoly(roi_mask, [np.array(roi, dtype=np.int32)], 255)
    return cv2.bitwise_and(mask, roi_mask)


class Box:
    """One detected box: pixel center, rotated rect (for drawing), angle and,
    once transformed, robot coordinates in mm."""

    def __init__(self, cx, cy, rect, angle):
        self.cx = float(cx)          # pixel center
        self.cy = float(cy)
        self.rect = rect             # cv2.minAreaRect result, for drawing
        self.angle = float(angle)    # rotation of the box in the image (deg)
        self.X = None                # robot coords (mm), set by to_robot()
        self.Y = None
        self.box_id = None

    def dist_to_robot(self):
        """Distance from the robot base (origin of the robot frame) — used to
        pick the nearest box first."""
        return float(np.hypot(self.X, self.Y))


class BoxDetector:
    """Detect boxes of a KNOWN colour (currently red) by HSV hue.

    Robust to shadow and ground because it keys on HUE + SATURATION, not
    brightness: a shadow on the box keeps the same hue (we allow a wide V
    range so shadowed box pixels still match), while the ground and shadows on
    the ground aren't the target hue (and dull reddish wood is rejected by
    the saturation floor). So neither shadow nor ground colour matters."""

    def __init__(self, cfg):
        v = cfg["vision"]
        self.lower = np.array(v["hsv_lower"], dtype=np.uint8)
        self.upper = np.array(v["hsv_upper"], dtype=np.uint8)
        self.min_area = v["min_area_px"]
        self.max_area = v["max_area_px"]
        self.rect_min = v.get("rectangularity_min", 0.80)
        self.roi = v.get("roi")
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.last_mask = None

    def _hue_mask(self, hsv):
        """inRange, but handles the RED wrap-around at the 0/180 hue seam.

        OpenCV hue is 0..179 in a circle, and red lives on both ends (~0-10
        and ~170-179). If hsv_lower's hue > hsv_upper's hue we read it as a
        wrap (e.g. H 170 -> 10) and OR two sub-ranges. Any non-wrapping colour
        (e.g. yellow 17->32) has lower hue <= upper hue and takes the single
        range exactly as before."""
        lo, hi = self.lower, self.upper
        if lo[0] <= hi[0]:
            return cv2.inRange(hsv, lo, hi)
        # wrap: [0 .. hi_hue]  OR  [lo_hue .. 179], same S/V bounds on both
        low_band = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], np.uint8),
                               np.array([hi[0], hi[1], hi[2]], np.uint8))
        high_band = cv2.inRange(hsv, np.array([lo[0], lo[1], lo[2]], np.uint8),
                                np.array([179, hi[1], hi[2]], np.uint8))
        return cv2.bitwise_or(low_band, high_band)

    def detect(self, frame) -> list:
        """Detect boxes in a single frame. Returns a list of Box (pixel coords only)."""
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = self._hue_mask(hsv)                           # coloured, saturated pixels
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = apply_roi(mask, self.roi)
        self.last_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if not (self.min_area <= area <= self.max_area):
                continue
            rect = cv2.minAreaRect(c)          # ((cx, cy), (w, h), angle)
            (cx, cy), (w, h), angle = rect
            rect_area = w * h
            if rect_area <= 0 or area / rect_area < self.rect_min:
                continue                       # reject non-rectangular red blobs
            boxes.append(Box(cx, cy, rect, angle))
        return boxes


def make_detector(cfg):
    """Build the box detector. We use the HSV colour detector (red boxes)."""
    return BoxDetector(cfg)


def aggregate_detections(samples, match_radius=40.0, min_hit_ratio=0.6):
    """Turn a list of per-frame detections (list of list of Box) into one
    stable set of boxes.

    Boxes from different frames are matched by proximity (< match_radius px)
    and grouped; a group must appear in at least min_hit_ratio of the frames
    to count (kills single-frame flickers). The reported center is the mean
    over the group — this averages out pixel noise.
    """
    groups = []   # each group: list of Box
    for frame_boxes in samples:
        for b in frame_boxes:
            for g in groups:
                gx = np.mean([m.cx for m in g])
                gy = np.mean([m.cy for m in g])
                if np.hypot(b.cx - gx, b.cy - gy) < match_radius:
                    g.append(b)
                    break
            else:
                groups.append([b])

    n_frames = max(len(samples), 1)
    stable = []
    for g in groups:
        if len(g) / n_frames < min_hit_ratio:
            continue
        cx = float(np.mean([m.cx for m in g]))
        cy = float(np.mean([m.cy for m in g]))
        # take rect/angle from the member closest to the mean center
        best = min(g, key=lambda m: np.hypot(m.cx - cx, m.cy - cy))
        stable.append(Box(cx, cy, best.rect, best.angle))
    return stable


class PixelToRobot:
    """Loads the affine transform produced by calibrate.py and applies it in
    both directions (pixel->robot for picking, robot->pixel for drawing the
    storage spot)."""

    def __init__(self, calibration_file):
        with open(calibration_file, "r") as f:
            data = json.load(f)
        self.M = np.array(data["M"], dtype=np.float64)          # 2x3
        self.M_inv = cv2.invertAffineTransform(self.M)          # robot -> pixel

    def to_robot(self, u, v):
        X, Y = self.M @ np.array([u, v, 1.0])
        return float(X), float(Y)

    def to_pixel(self, X, Y):
        u, v = self.M_inv @ np.array([X, Y, 1.0])
        return int(round(u)), int(round(v))


# ---------------------------------------------------------------------- #
# drawing helpers for the live visualization window
# ---------------------------------------------------------------------- #
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)


def draw_box(frame, box, color, label=None):
    pts = cv2.boxPoints(box.rect).astype(np.int32)
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
    cv2.drawMarker(frame, (int(box.cx), int(box.cy)), YELLOW,
                   cv2.MARKER_CROSS, 20, 2)
    if label:
        cv2.putText(frame, label, (int(box.cx) + 12, int(box.cy) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


def storage_quad(transform, X, Y, size_mm=50.0):
    """Pixel corners of a size_mm x size_mm square centred on the robot position
    (X, Y). Each corner is projected separately through the calibration, so the
    square stays a true 50 mm footprint on the table whatever the camera's
    scale and rotation — unlike a fixed pixel size."""
    h = size_mm / 2.0
    return [transform.to_pixel(X + dx, Y + dy)
            for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]


def _draw_dashed_poly(frame, pts, color, thickness=2, dash_px=8):
    """Closed polygon outline drawn as dashes (cv2 has no dashed line style)."""
    for i in range(len(pts)):
        p = np.array(pts[i], dtype=np.float64)
        q = np.array(pts[(i + 1) % len(pts)], dtype=np.float64)
        length = np.linalg.norm(q - p)
        if length < 1e-6:
            continue
        step = dash_px / length
        t = 0.0
        while t < 1.0:
            a = p + (q - p) * t
            b = p + (q - p) * min(t + step, 1.0)
            cv2.line(frame, tuple(a.astype(np.int32)), tuple(b.astype(np.int32)),
                     color, thickness, cv2.LINE_AA)
            t += 2 * step


def draw_storage(frame, quad):
    """Dashed blue square marking the storage spot, from storage_quad()."""
    _draw_dashed_poly(frame, quad, BLUE, 2)
    u = min(p[0] for p in quad)
    v = min(p[1] for p in quad)
    cv2.putText(frame, "STORAGE", (u, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2, cv2.LINE_AA)


def draw_status(frame, text):
    cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 1, cv2.LINE_AA)
