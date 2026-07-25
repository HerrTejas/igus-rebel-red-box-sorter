"""
Pre-flight check — run this FIRST, before any test or the main app.

Moves NOTHING. Touches no motors. It only inspects:
    1. python packages (cv2, numpy, paho-mqtt)
    2. config.json is valid and complete
    3. calibration.json exists + how good the fit was
    4. which camera indices actually open, and what the configured one delivers
    5. robot controller reachable over TCP (plain socket connect, no enable)
    6. MQTT broker reachable over TCP

Usage:  python check_setup.py            (full check)
        python check_setup.py --no-net   (skip robot + broker, e.g. at home)
"""
import json
import socket
import sys

CONFIG_FILE = "config.json"

PASS = "  [ OK ] "
FAIL = "  [FAIL] "
WARN = "  [WARN] "

failures = 0


def ok(msg):
    print(PASS + msg)


def fail(msg):
    global failures
    failures += 1
    print(FAIL + msg)


def warn(msg):
    print(WARN + msg)


# ------------------------------------------------------------------ #
print("\n1) python packages")
try:
    import cv2
    ok(f"opencv {cv2.__version__}")
except ImportError as e:
    fail(f"opencv missing: {e} — activate the env: conda activate xr-igus")
    sys.exit(1)
try:
    import numpy
    ok(f"numpy {numpy.__version__}")
except ImportError as e:
    fail(f"numpy missing: {e}")
    sys.exit(1)
try:
    import paho.mqtt.client as mqtt
    ok("paho-mqtt importable")
except ImportError as e:
    fail(f"paho-mqtt missing: {e}")

# ------------------------------------------------------------------ #
print("\n2) config.json")
try:
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    ok("parses as valid JSON")
except (OSError, ValueError) as e:
    fail(f"cannot read config.json: {e}")
    sys.exit(1)

for section, keys in {
    "robot": ["host", "port", "vel_travel", "vel_approach", "detect_pose_joint",
              "gripper_open_dout", "gripper_close_dout", "gripper_delay"],
    "camera": ["source", "width", "height"],
    "vision": ["hsv_lower", "hsv_upper", "min_area_px", "stable_samples"],
    "task": ["safe_z", "pick_z", "place_z_first", "box_height", "tool_down",
             "expected_boxes", "default_storage"],
    "mqtt": ["host", "port", "topic_sort", "topic_status"],
}.items():
    missing = [k for k in keys if k not in cfg.get(section, {})]
    if missing:
        fail(f"config section '{section}' missing keys: {missing}")
    else:
        ok(f"section '{section}' complete")

if cfg["task"]["safe_z"] <= cfg["task"]["pick_z"]:
    fail(f"safe_z ({cfg['task']['safe_z']}) must be well above pick_z "
         f"({cfg['task']['pick_z']})")
else:
    ok(f"heights sane: safe_z={cfg['task']['safe_z']} > pick_z={cfg['task']['pick_z']}")

# ------------------------------------------------------------------ #
print("\n3) calibration")
try:
    with open(cfg["calibration_file"], "r") as f:
        cal = json.load(f)
    n = len(cal.get("pixel_pts", []))
    res = cal.get("residual_mean_mm", -1)
    ok(f"calibration.json found: {n} points, residual {res:.2f} mm mean")
    if res > 5:
        warn("residual > 5 mm — consider re-calibrating with wider-spread points")
except OSError:
    warn("calibration.json not found — run calibrate.py before main.py "
         "(fine if you haven't calibrated yet)")
except (ValueError, KeyError) as e:
    fail(f"calibration.json unreadable: {e}")

# ------------------------------------------------------------------ #
print("\n4) cameras attached (probing indices 0-4, a few seconds)")
found = []
for idx in range(5):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        got, frame = cap.read()
        if got:
            h, w = frame.shape[:2]
            found.append(idx)
            marker = "  <-- configured" if idx == cfg["camera"]["source"] else ""
            print(f"         index {idx}: {w}x{h}{marker}")
    cap.release()
if not found:
    fail("no camera opened on indices 0-4 — is it plugged in?")
elif cfg["camera"]["source"] in found:
    ok(f"configured source {cfg['camera']['source']} works")
else:
    fail(f"configured source {cfg['camera']['source']} did not open — "
         f"working indices: {found}. Fix camera.source in config.json "
         f"or run test_camera.py to pick one.")

# ------------------------------------------------------------------ #
def tcp_reachable(host, port, timeout=3):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


if "--no-net" in sys.argv:
    print("\n5+6) network checks skipped (--no-net)")
else:
    print("\n5) robot controller")
    r = cfg["robot"]
    if tcp_reachable(r["host"], r["port"]):
        ok(f"TCP connect to {r['host']}:{r['port']} works (no commands sent)")
    else:
        fail(f"cannot reach {r['host']}:{r['port']} — robot on? same network? "
             f"right IP in config.json? (simulator uses localhost:3921)")

    print("\n6) MQTT broker")
    m = cfg["mqtt"]
    if tcp_reachable(m["host"], m["port"], timeout=5):
        ok(f"TCP connect to {m['host']}:{m['port']} works")
        print(f"         topics: sort={m['topic_sort']}  status={m['topic_status']}")
    else:
        fail(f"cannot reach broker {m['host']}:{m['port']} — internet up? "
             f"(you can still dry-run with the 't' key in main.py)")

# ------------------------------------------------------------------ #
print()
if failures:
    print(f"RESULT: {failures} problem(s) found — fix these before the next step.")
    sys.exit(1)
print("RESULT: all checks passed. Test order: test_camera.py -> test_robot.py "
      "-> test_gripper.py -> calibrate.py -> main.py")
