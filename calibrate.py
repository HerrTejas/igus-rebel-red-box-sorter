"""
Camera -> robot calibration (run this FIRST, before main.py).

Improved version of the course calib.py: the solved transform is SAVED to
calibration.json so main.py can load it — you calibrate once per camera setup,
not once per program run.

How it works
------------
We need a mapping from image pixels (u, v) to robot table coordinates
(X, Y) in mm. Camera above the table + flat table  =>  the two planes are
related by a 2D affine transform:

    [X]   [m11 m12 m13]   [u]
    [Y] = [m21 m22 m23] @ [v]
                          [1]

6 unknowns -> 3 point pairs minimum; we require 4+ and solve least squares
with RANSAC (cv2.estimateAffine2D) so one sloppy click doesn't ruin it.
The residual printout tells you how good the fit is (aim for < 3 mm mean).

Procedure
---------
1. put a small marker (e.g. a box corner or pen tip) somewhere on the table
2. click that spot in the camera window
3. jog the robot tip to the same spot using the iRC teach pendant / software
   and type the robot X and Y (mm) into this console
4. repeat for 4+ points spread over the whole workspace (corners!)
5. press 's' -> transform is solved and written to calibration.json
6. AIM mode: click anywhere; if the robot is connected it drives there —
   perfect sanity check before the real run.

Run without robot (only solve + save):  python calibrate.py --no-robot
"""
import json
import sys

import cv2
import numpy as np

import igus

CONFIG_FILE = "config.json"
WIN = "CALIBRATION"


class Calibrator:
    def __init__(self, use_robot=True):
        with open(CONFIG_FILE, "r") as f:
            self.cfg = json.load(f)

        cam = self.cfg["camera"]
        self.cap = cv2.VideoCapture(cam["source"], cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open camera source {cam['source']}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
        print("connected to camera")

        # hover height for AIM mode: use safe_z so the tool stays well above
        # the (possibly raised) table instead of a hardcoded value
        self.aim_z = self.cfg["task"]["safe_z"]

        self.mode = "calibrate"
        self.pixel_pts = []
        self.robot_pts = []
        self.transform = None
        self.frame = None
        self.aim_marker = None

        self.robot = None
        if use_robot:
            r = self.cfg["robot"]
            self.robot = igus.IGUS(r["host"], r["port"])
            self.robot.connect()

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WIN, self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.frame is None:
            return
        if self.mode == "calibrate":
            print(f"\nselected pixel ({x}, {y})")
            try:
                rx = float(input("robot X (mm): "))
                ry = float(input("robot Y (mm): "))
            except ValueError:
                print("not a number — point cancelled")
                return
            self.pixel_pts.append((x, y))
            self.robot_pts.append((rx, ry))
            n = len(self.pixel_pts)
            print(f"point {n} added." + (" ready to solve (press 's')" if n >= 4 else ""))
        else:
            X, Y = self.pixel_to_robot(x, y)
            self.aim_marker = (x, y)
            print(f"[AIM] pixel=({x},{y}) -> robot X={X:.1f} Y={Y:.1f} mm")
            if self.robot:
                tool = self.cfg["task"]["tool_down"]
                self.robot.go_to(igus.Cart(X, Y, self.aim_z, tool["A"], tool["B"], tool["C"]),
                                 vel=self.cfg["robot"]["vel_travel"])

    def solve(self):
        n = len(self.pixel_pts)
        if n < 4:
            print(f"need at least 4 points (have {n})")
            return
        src = np.array(self.pixel_pts, dtype=np.float32)
        dst = np.array(self.robot_pts, dtype=np.float32)
        M, _ = cv2.estimateAffine2D(src, dst, method=cv2.RANSAC)
        if M is None:
            print("error: could not fit a transform")
            return
        self.transform = M

        # residuals: how far off each calibration point is after the fit
        proj = (M @ np.hstack([src, np.ones((n, 1))]).T).T
        err = np.linalg.norm(proj - dst, axis=1)
        print(f"\n[SOLVE] affine fit on {n} points | "
              f"residual {err.mean():.2f} mm mean, {err.max():.2f} mm max")

        out = self.cfg["calibration_file"]
        with open(out, "w") as f:
            json.dump({
                "M": M.tolist(),
                "pixel_pts": self.pixel_pts,
                "robot_pts": self.robot_pts,
                "residual_mean_mm": float(err.mean()),
                "residual_max_mm": float(err.max()),
            }, f, indent=4)
        print(f"saved transform to {out}")
        print("AIM mode: click a spot to send the robot there (q=quit)\n")
        self.mode = "aim"
        if self.robot:
            self.robot.go_to_L()

    def pixel_to_robot(self, u, v):
        X, Y = self.transform @ np.array([u, v, 1.0])
        return float(X), float(Y)

    def undo(self):
        if self.mode == "calibrate" and self.pixel_pts:
            self.pixel_pts.pop()
            self.robot_pts.pop()
            print(f"removed last point, {len(self.pixel_pts)} left")

    def draw(self, frame):
        for i, (u, v) in enumerate(self.pixel_pts):
            cv2.drawMarker(frame, (int(u), int(v)), (45, 45, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.circle(frame, (int(u), int(v)), 9, (45, 45, 255), 2)
            cv2.putText(frame, str(i + 1), (int(u) + 12, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (45, 45, 255), 1, cv2.LINE_AA)
        if self.mode == "aim" and self.aim_marker is not None:
            cv2.circle(frame, self.aim_marker, 13, (106, 191, 95), 2)
            cv2.drawMarker(frame, self.aim_marker, (106, 191, 95), cv2.MARKER_CROSS, 26, 2)
        label = (f"CALIBRATE  {len(self.pixel_pts)} pts  (s=solve+save  u=undo  q=quit)"
                 if self.mode == "calibrate" else "AIM  click a position  (q=quit)")
        cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    def run(self):
        print("CALIBRATE: click a point in the image, then type its robot X/Y "
              "in this console. Place 4+ points, then press 's'.")
        while True:
            ok, frame = self.cap.read()
            if ok:
                self.frame = frame
            if self.frame is not None:
                shown = self.frame.copy()
                self.draw(shown)
                cv2.imshow(WIN, shown)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s') and self.mode == "calibrate":
                self.solve()
            elif key == ord('u'):
                self.undo()
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
        self.cap.release()
        cv2.destroyAllWindows()
        if self.robot:
            self.robot.disconnect()


if __name__ == "__main__":
    Calibrator(use_robot="--no-robot" not in sys.argv).run()
