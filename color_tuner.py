"""
Live HSV tuner for the 'color' detector (red boxes).

Left = camera with detected boxes outlined; right = the mask (white =
"matches the colour"). Slide the 6 HSV bounds until ONLY the red boxes are
white and solid, then press 's' to save into config.json. 'q' quits.

For RED, remember hue wraps the 0/180 seam: set H min HIGH (~170) and H max
LOW (~10) and the detector reads it as the red band on both ends. Keep S min
fairly high (vivid red) so dull reddish wood/ground is rejected, and V min low
so red in shadow is still caught. Hue in OpenCV is 0-179.
"""
import json

import cv2
import numpy as np

import config_io
import vision

CONFIG_FILE = "config.json"
WIN = "COLOR TUNER (red)"


def nothing(_):
    pass


def save_vision_settings(lower, upper, min_area, path=CONFIG_FILE):
    """Write ONLY this tuner's own three keys into config.json."""
    return config_io.update("vision", {
        "hsv_lower": [int(x) for x in lower],
        "hsv_upper": [int(x) for x in upper],
        "min_area_px": int(min_area),
    }, path=path)


def main():
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)

    cam = cfg["camera"]
    cap = cv2.VideoCapture(cam["source"], cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera source {cam['source']}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])

    lo = cfg["vision"]["hsv_lower"]
    hi = cfg["vision"]["hsv_upper"]
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H min", WIN, lo[0], 179, nothing)
    cv2.createTrackbar("H max", WIN, hi[0], 179, nothing)
    cv2.createTrackbar("S min", WIN, lo[1], 255, nothing)
    cv2.createTrackbar("S max", WIN, hi[1], 255, nothing)
    cv2.createTrackbar("V min", WIN, lo[2], 255, nothing)
    cv2.createTrackbar("V max", WIN, hi[2], 255, nothing)
    cv2.createTrackbar("min area", WIN, int(cfg["vision"]["min_area_px"]), 30000, nothing)

    detector = vision.BoxDetector(cfg)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        detector.lower = np.array([cv2.getTrackbarPos("H min", WIN),
                                   cv2.getTrackbarPos("S min", WIN),
                                   cv2.getTrackbarPos("V min", WIN)], dtype=np.uint8)
        detector.upper = np.array([cv2.getTrackbarPos("H max", WIN),
                                   cv2.getTrackbarPos("S max", WIN),
                                   cv2.getTrackbarPos("V max", WIN)], dtype=np.uint8)
        detector.min_area = cv2.getTrackbarPos("min area", WIN)
        boxes = detector.detect(frame)

        shown = frame.copy()
        for b in boxes:
            vision.draw_box(shown, b, vision.RED)
        mask_bgr = cv2.cvtColor(detector.last_mask, cv2.COLOR_GRAY2BGR)
        side = cv2.resize(np.hstack([shown, mask_bgr]), None, fx=0.5, fy=0.5)
        cv2.putText(side, f"{len(boxes)} boxes   s=save  q=quit", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(WIN, side)

        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            save_vision_settings(detector.lower, detector.upper, detector.min_area)
            print(f"saved hsv_lower={list(detector.lower)} hsv_upper={list(detector.upper)} "
                  f"min_area_px={detector.min_area}")
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
