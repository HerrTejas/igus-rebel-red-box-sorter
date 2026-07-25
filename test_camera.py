"""
Camera test — no robot needed, nothing moves.

Shows the live feed with resolution/FPS overlay so you can verify you're
looking at the RIGHT camera (overhead, whole workspace visible, in focus).

Keys:
    n   try the next camera index (cycles 0-4) — for finding the overhead cam
    d   toggle detection overlay (uses the HSV values currently in config.json)
    s   save the currently shown index into config.json as camera.source
    q   quit

Run me before calibrating. If the boxes don't light up under 'd', run
hsv_tuner.py next.
"""
import json
import time

import cv2

import config_io
import vision

CONFIG_FILE = "config.json"
WIN = "CAMERA TEST"
MAX_INDEX = 4


def open_cam(idx, cfg):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
    return cap


def main():
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)

    idx = cfg["camera"]["source"]
    cap = open_cam(idx, cfg)
    detector = vision.make_detector(cfg)
    show_detection = False

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    print(f"showing camera index {idx} — n=next  d=detection  s=save  q=quit")

    t_last, fps = time.time(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            frame = None

        if frame is not None:
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
            t_last = now

            if show_detection:
                for b in detector.detect(frame):
                    vision.draw_box(frame, b, vision.RED)

            h, w = frame.shape[:2]
            saved = " (saved)" if idx == cfg["camera"]["source"] else ""
            vision.draw_status(frame,
                               f"index {idx}{saved}  {w}x{h}  {fps:.0f} fps  "
                               f"detection {'ON' if show_detection else 'off'}  "
                               f"(n/d/s/q)")
            cv2.imshow(WIN, frame)
        else:
            # black placeholder so the window stays responsive
            import numpy as np
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            vision.draw_status(blank, f"index {idx}: NO IMAGE  (n=next  q=quit)")
            cv2.imshow(WIN, blank)

        key = cv2.waitKey(15) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('n'):
            cap.release()
            idx = (idx + 1) % (MAX_INDEX + 1)
            print(f"trying camera index {idx} ...")
            cap = open_cam(idx, cfg)
        elif key == ord('d'):
            show_detection = not show_detection
        elif key == ord('s'):
            config_io.update("camera", {"source": idx})
            print(f"saved camera.source = {idx} to config.json")
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
