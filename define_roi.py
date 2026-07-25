"""
Define the detection region of interest (ROI) = the box-table area.

Because the robot arm is dark like the boxes, the contour detector would treat
the arm as a box. The fix: draw a polygon around ONLY the table where the boxes
sit. Detection then ignores everything outside it, including the arm.

How to use:
    1. Move the robot to your detection pose (L pose) so you see the real scene.
    2. Click the corners of the box table area (4 clicks is typical; go around
       the region you want to search). Keep the arm's resting area OUTSIDE this
       polygon.
    3. Press 's' to save the polygon into config.json (vision.roi).
       'r' resets the points, 'u' undoes the last point, 'q' quits.

The green outline is your ROI; red boxes show what the detector currently finds
INSIDE it. Adjust until only the real box(es) are outlined and the arm is not.
"""
import json

import cv2
import numpy as np

import config_io
import vision

CONFIG_FILE = "config.json"
WIN = "DEFINE ROI (click table corners)"

points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print(f"point {len(points)}: ({x}, {y})")


def main():
    global points
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    if cfg["vision"].get("roi"):
        points = [list(p) for p in cfg["vision"]["roi"]]
        print(f"loaded existing ROI with {len(points)} points")

    cam = cfg["camera"]
    cap = cv2.VideoCapture(cam["source"], cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera source {cam['source']}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])

    detector = vision.make_detector(cfg)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, on_mouse)
    print("click the table corners; s=save  u=undo  r=reset  q=quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        # live detection using the polygon drawn so far
        detector.roi = points if len(points) >= 3 else None
        boxes = detector.detect(frame)

        shown = frame.copy()
        if len(points) >= 1:
            for i, p in enumerate(points):
                cv2.circle(shown, tuple(p), 6, (0, 255, 0), -1)
                cv2.putText(shown, str(i + 1), (p[0] + 8, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if len(points) >= 2:
            cv2.polylines(shown, [np.array(points, np.int32)],
                          isClosed=len(points) >= 3, color=(0, 255, 0), thickness=2)
        for b in boxes:
            vision.draw_box(shown, b, vision.RED)
        vision.draw_status(shown, f"{len(points)} corners  |  {len(boxes)} boxes inside  "
                                  f"(s=save u=undo r=reset q=quit)")
        cv2.imshow(WIN, shown)

        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('u') and points:
            points.pop()
        elif key == ord('r'):
            points = []
        elif key == ord('s'):
            if len(points) < 3:
                print("need at least 3 corners to save")
            else:
                config_io.update("vision", {"roi": points})
                print(f"saved ROI with {len(points)} corners to config.json")
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
