# IGUS ReBeL - Autonomous Red-Box Sorter

Course project (New Industrial Technologies - XR / Autonomous Robotics, SRH).
A downward-facing camera detects **red boxes** on a table; on an MQTT start
message the IGUS ReBeL robot detects them, picks the closest first, and stacks
them at a storage location - fully autonomously, with a live OpenCV view.

Flow: **candle pose** (detect, arm out of view) → **L pose** → pick nearest →
place/stack → back to **candle**.

## Demo video

https://github.com/user-attachments/assets/d3e6acc5-1288-49dd-a2eb-49f110200a37

*Accelerated autonomous red-box sorting demo (18 seconds).*

## Acknowledgements

This project was developed as part of the New Industrial Technologies course
(XR / Autonomous Robotics) at SRH. The original `igus.py` robot-control module
was developed by Lukasz Rojek for the course and was extended for this project
with gripper control and more robust motion-completion handling.

---

## 1. Requirements

- **Windows** with a webcam mounted above the table
- **Miniconda / Anaconda**
- **IGUS ReBeL** reachable over the network (default `192.168.3.11:3920`), with
  **iRC** (igus Robot Control) installed
- A vacuum (suction) gripper on the robot tool

## 2. Setup

### 2a. Get the code
```bash
git clone https://github.com/HerrTejas/igus-rebel-red-box-sorter.git
cd igus-rebel-red-box-sorter
```

### 2b. Create the conda environment

**Option A - from the environment file (recommended):**
```bash
conda env create -f environment.yml
conda activate xr-igus
```

**Option B - manually:**
```bash
conda create -n xr-igus python=3.11
conda activate xr-igus
pip install -r requirements.txt
```

Packages installed: `numpy`, `opencv-python`, `paho-mqtt==1.6.1`
(paho pinned to 1.x - the code uses the v1 callback API).

### 2c. Verify everything (no robot motion)
```bash
python check_setup.py
```
Checks packages, `config.json`, the camera, and robot/broker reachability.

## 3. Configure - `config.json`

Everything tunable lives here. Key fields:

| Section | Key | Meaning |
|---|---|---|
| `robot` | `host` / `port` | robot controller address (`192.168.3.11:3920`) |
| `robot` | `gripper_vacuum_dout` / `gripper_blowoff_dout` | suction ON channel / blow-off release channel |
| `robot` | `gripper_delay` | seconds to wait for seal / release |
| `camera` | `source` | camera index (0, 1, 2 …) - the overhead cam |
| `vision` | `method` | `"color"` (red HSV detection) |
| `vision` | **`hsv_lower` / `hsv_upper`** | **the red colour range (see §5)** |
| `vision` | `min_area_px` | smallest blob accepted as a box |
| `task` | `pick_z` / `place_z_first` / `safe_z` | jog-measured heights (mm, robot frame) |
| `task` | `box_height` | stack step per level |
| `mqtt` | `host` / `port` / topics | broker + start/status topics |

## 4. Run - step by step

Do the one-time tuning/calibration first, then run.

```bash
# 1. confirm the camera + detection see the red boxes
python color_tuner.py        # tune the red HSV range, press 's' to save

# 2. teach the camera <-> robot mapping (needed once per camera setup)
python calibrate.py          # click 4+ points, jog robot, type X/Y, 's' saves

# 3. optional subsystem checks
python test_camera.py        # find/verify the overhead camera
python test_robot.py         # slow staged arm moves
python test_gripper.py       # test grip + blow-off release

# 4. run the autonomous sorter
python main.py
```

In `main.py`: **click the camera window**, then press **`t`** for a local test
run (places each box back where picked), or publish an MQTT start message
`{"X": 337, "Y": 263}` to the sort topic to stack at that spot.

**Keys** (with the OpenCV window focused): `t` start · `o`/`c` gripper open/close
· `q` quit.

## 5. The red colour - where it's defined

Detection keys on **hue**, which makes it robust to shadow and ground:

- **`config.json` → `vision.hsv_lower` / `hsv_upper`** is the red definition.
  - **H** (hue) - red wraps the 0/180 seam, so H min is HIGH (~170) and H max
    LOW (~10); the detector reads that as the red band on both ends
  - **S min** (saturation floor) - set high so only *vivid* red passes; this
    is what makes the **ground not matter** (dull wood is low-saturation)
  - **V** (value/brightness) - kept wide so **red in shadow** still counts
- `vision.method: "color"` selects the colour detector (`BoxDetector` in
  `vision.py`), which applies the range with `cv2.inRange`.

Retune anytime with `python color_tuner.py` - it writes back to those keys, so
you never edit code. To detect a **different** colour, just center the H sliders
on that colour's hue.

## 6. Files

| File | Purpose |
|---|---|
| `main.py` | The autonomous app: MQTT + state machine + visualization |
| `igus.py` | Robot driver: CRI protocol over TCP, moves, gripper (vacuum + blow-off) |
| `vision.py` | Detection + pixel↔robot transform + drawing |
| `config.json` | All settings (IP, camera, red range, heights, MQTT) |
| `config_io.py` | Small helper to update single config keys safely |
| `color_tuner.py` | Live HSV tuner for the red boxes |
| `calibrate.py` | Camera→robot calibration; saves `calibration.json` |
| `define_roi.py` | Optional: restrict detection to a table region |
| `check_setup.py` | Pre-flight checks (no motion) |
| `test_camera.py` / `test_robot.py` / `test_gripper.py` | Subsystem tests |
| `mqtt_test.py` | Standalone MQTT round-trip test |
| `environment.yml` / `requirements.txt` | Environment definition |

## 7. Notes & troubleshooting

- **MQTT ports:** the Python code uses plain MQTT on **1883**; the HiveMQ
  *browser* client uses WebSocket **8884**. Both reach the same broker - keep
  each on its own port. (The code auto-corrects 8884→1883 for safety.)
- **Robot unreachable / connection timeout:** check the robot is powered and on
  the same network; `ping 192.168.3.11`. Keep Ethernet on the robot LAN and
  Wi-Fi for internet if you need both.
- **Cup lands off-centre on far boxes:** re-calibrate with points spread to the
  corners; click box *tops* to cancel parallax.
- **Box won't release:** the vacuum holds via a check valve - release is a
  blow-off pulse on `gripper_blowoff_dout`.
- **Keys do nothing:** click the OpenCV window first (not the terminal).
