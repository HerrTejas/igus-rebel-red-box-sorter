"""
Final Project — autonomous box sorting with the IGUS ReBeL.

Program flow (state machine):

    IDLE  --(MQTT start message on IGUS/robotX/sort with storage {X, Y})-->
    MOVE_AWAY   robot turns out of the camera view
    DETECT      stable box detection over N frames, publish "<n> have been found!"
    SORT        for each box, nearest to the robot base first:
                    publish {box_id, pos_old, pos_new}
                    open gripper -> above box -> down -> close -> up
                    -> above storage -> down to stack height -> open -> up
    DONE        publish "COMPLETE", robot back to L pose, return to IDLE

The OpenCV window runs the whole time in the main thread:
    red rectangles  = all currently detected boxes
    green rectangle = the box being picked right now
    blue rectangle  = storage spot (robot coords projected back into the image)

Keys:  q = quit   t = test start without MQTT (uses default_storage from
config)   o / c = manually open / close the gripper (handy for setup).

Run AFTER calibrating:   python calibrate.py   ->   python main.py
"""
import json
import threading
import time

import cv2
import paho.mqtt.client as mqtt

import igus
import vision

CONFIG_FILE = "config.json"
WIN = "BOX SORTER"


class SharedState:
    """Data shared between the main (camera/GUI) thread, the MQTT thread and
    the robot worker thread. Always access under `lock`."""

    def __init__(self):
        self.lock = threading.Lock()
        self.live_boxes = []        # detections of the latest frame (Box, px only)
        self.frame_counter = 0      # increases with every processed frame
        self.selected_box = None    # Box currently being picked (drawn green)
        self.storage_quad = None    # 4 pixel corners of the storage square
        self.status_text = "IDLE - waiting for MQTT start message ('t' = test start)"
        self.start_request = None   # (X, Y) storage position, set by MQTT
        self.busy = False           # True while the sort sequence runs
        self.quit = False


def load_config():
    with open(CONFIG_FILE, "r") as f:
        return json.load(f)


# ---------------------------------------------------------------------- #
# robot sort sequence (runs in a worker thread)
# ---------------------------------------------------------------------- #
def collect_stable_boxes(state, cfg, transform):
    """Sample the live detections for `stable_samples` frames and aggregate
    them into one reliable set of boxes with robot coordinates."""
    v = cfg["vision"]
    samples = []
    with state.lock:
        last_counter = state.frame_counter
    t0 = time.time()
    while len(samples) < v["stable_samples"] and time.time() - t0 < 15:
        time.sleep(0.03)
        with state.lock:
            if state.frame_counter != last_counter:
                last_counter = state.frame_counter
                samples.append(list(state.live_boxes))

    boxes = vision.aggregate_detections(samples,
                                        match_radius=v["match_radius_px"],
                                        min_hit_ratio=v["min_hit_ratio"])
    for b in boxes:
        b.X, b.Y = transform.to_robot(b.cx, b.cy)
    return boxes


def sort_sequence(state, cfg, robot, transform, client, storage):
    """The complete autonomous pick-and-place run for one start message."""
    task = cfg["task"]
    r = cfg["robot"]
    tool = task["tool_down"]
    topic_status = cfg["mqtt"]["topic_status"]

    def set_status(text):
        with state.lock:
            state.status_text = text
        print(text)

    def cart(X, Y, Z):
        return igus.Cart(X, Y, Z, tool["A"], tool["B"], tool["C"])

    def publish(payload):
        if client is not None:            # no-op when running without a broker
            client.publish(topic_status, payload)

    # "inplace" = no storage pose given (e.g. 't' test): place each box back
    # where it was picked. Otherwise stack at the storage X/Y (from MQTT).
    inplace = storage.get("inplace", False)

    try:
        # storage spot for the blue rectangle in the visualization
        with state.lock:
            state.storage_quad = (None if inplace else vision.storage_quad(
                transform, storage["X"], storage["Y"],
                task.get("storage_square_mm", 50.0)))

        # 1. go to candle pose (arm straight up, OUT of the camera view so the
        #    dark arm is not mistaken for a box) and detect ONCE from there.
        #    Positions are stored below and never re-detected during motion.
        set_status("moving to candle pose to read box positions...")
        robot.go_to_zero(vel=r["vel_travel"])
        time.sleep(1.0)  # let the image settle (auto exposure, vibrations)

        # 2. detect boxes ONCE, store them
        set_status("detecting boxes...")
        boxes = collect_stable_boxes(state, cfg, transform)
        publish(f"{len(boxes)} have been found!")
        set_status(f"{len(boxes)} boxes found")
        if len(boxes) != task["expected_boxes"]:
            print(f"WARNING: expected {task['expected_boxes']} boxes, "
                  f"found {len(boxes)} — continuing with what we have")
        if not boxes:
            set_status("no boxes found - back to IDLE")
            return

        # 3. nearest box (to the robot base = origin of robot frame) first
        boxes.sort(key=lambda b: b.dist_to_robot())
        for i, b in enumerate(boxes):
            b.box_id = i + 1

        # 4. into the sorting workspace
        robot.go_to_L(vel=r["vel_travel"])
        robot.gripper_open()

        # 5. pick and place, stacking upwards
        for i, box in enumerate(boxes):
            # where to place: back at the pick spot (inplace test) or on the
            # stack at the storage pose (Z rises by box_height per level)
            if inplace:
                place_x, place_y, place_z = box.X, box.Y, task["pick_z"]
            else:
                place_x, place_y = storage["X"], storage["Y"]
                place_z = task["place_z_first"] + i * task["box_height"]
            with state.lock:
                state.selected_box = box

            publish(json.dumps({
                "box_id": box.box_id,
                "pos_old": {"X": round(box.X, 1), "Y": round(box.Y, 1),
                            "Z": task["pick_z"]},
                "pos_new": {"X": round(place_x, 1), "Y": round(place_y, 1),
                            "Z": place_z},
            }))

            set_status(f"picking box {box.box_id} at X={box.X:.0f} Y={box.Y:.0f} "
                       f"(dist {box.dist_to_robot():.0f} mm)")
            if task.get("transit_via_L", False):
                robot.go_to_L(vel=r["vel_travel"])          # L pose before every pick
            robot.go_to(cart(box.X, box.Y, task["safe_z"]), vel=r["vel_travel"])
            robot.go_to(cart(box.X, box.Y, task["pick_z"]), vel=r["vel_approach"])
            robot.gripper_close()                           # suction ON, on the box
            robot.go_to(cart(box.X, box.Y, task["safe_z"]), vel=r["vel_approach"])

            place_desc = ("back at pick spot" if inplace
                          else f"on stack level {i + 1} (Z={place_z:.0f})")
            set_status(f"placing box {box.box_id} {place_desc}")
            # Go straight from the lifted box to above the place spot (at safe_z),
            # NOT back to the central L pose. The arm is already high and holding
            # the box; a direct move to the place location is what you want.
            robot.go_to(cart(place_x, place_y, task["safe_z"]), vel=r["vel_travel"])
            robot.go_to(cart(place_x, place_y, place_z), vel=r["vel_approach"])
            robot.gripper_open()                            # suction OFF + blow-off release
            robot.go_to(cart(place_x, place_y, task["safe_z"]), vel=r["vel_approach"])

            with state.lock:
                state.selected_box = None

        # 6. done -> back to candle pose (all joints 0, arm straight up)
        publish("COMPLETE")
        robot.go_to_zero(vel=r["vel_travel"])
        set_status("COMPLETE - at candle pose, waiting for next start message")

    except Exception as e:
        set_status(f"ERROR during sort sequence: {e}")
    finally:
        # safety: make sure the vacuum is OFF if we aborted (never leave it on)
        try:
            robot.set_dout(robot.gripper_vacuum_dout, False)
        except (OSError, ConnectionError):
            pass
        with state.lock:
            state.selected_box = None
            state.busy = False


# ---------------------------------------------------------------------- #
# MQTT
# ---------------------------------------------------------------------- #
def setup_mqtt(cfg, state, robot):
    m = cfg["mqtt"]
    client = mqtt.Client(client_id=f"igus-sorter-{int(time.time())}",
                         clean_session=True)

    # topic prefix (e.g. "igus/tejas/robot0") -> build the manual-control topics
    # from the same prefix as the sort topic, mirroring the course app.py.
    prefix = m["topic_sort"].rsplit("/", 1)[0]
    topic_zero = prefix + "/zero"
    topic_L = prefix + "/L"
    topic_joint = prefix + "/joint"
    topic_base = prefix + "/base"
    manual_topics = [topic_zero, topic_L, topic_joint, topic_base]

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"mqtt connected to {m['host']}:{m['port']}")
            for t in [m["topic_sort"]] + manual_topics:
                client.subscribe(t)
            print(f"subscribed to {m['topic_sort']} (+ manual zero/L/joint/base)")
        else:
            print(f"mqtt connection refused, rc={rc}")

    def on_message(client, userdata, msg):
        raw = msg.payload.decode("utf-8")            # string message (like the course code)
        print(f"MQTT message on '{msg.topic}': {raw}")
        topic = msg.topic

        # --- manual control (same as the course app.py), only when idle ---
        if topic in manual_topics:
            with state.lock:
                if state.busy:
                    print("  manual command ignored - sort sequence running")
                    return
            try:
                if topic == topic_zero:
                    robot.go_to_zero()
                elif topic == topic_L:
                    robot.go_to_L()
                elif topic == topic_joint:
                    robot.go_to(igus.Joint(**json.loads(raw)))
                elif topic == topic_base:
                    robot.go_to(igus.Cart(**json.loads(raw)))
            except Exception as e:
                print(f"  manual command error: {e}")
            return

        # --- the autonomous sort start message: {"X":.., "Y":..} ---
        try:
            data = json.loads(raw)
            storage = {"X": float(data["X"]), "Y": float(data["Y"])}
        except (ValueError, KeyError) as e:
            print(f"  ignoring malformed start message ({e})")
            return
        with state.lock:
            if state.busy:
                print("  start ignored - sequence already running")
                return
            state.start_request = storage
        print(f"  START accepted: storage at {storage}")

    client.on_connect = on_connect
    client.on_message = on_message

    # The Python (paho) client speaks plain MQTT over TCP = port 1883. Ports
    # 8884/8000/8083 are WebSocket ports for the BROWSER client only; paho
    # cannot use them here, so fall back to 1883 to avoid a silent no-connect.
    port = m["port"]
    if port in (8884, 8000, 8083, 443):
        print(f"NOTE: port {port} is a browser/WebSocket port; using TCP 1883 for Python")
        port = 1883

    try:
        client.connect(m["host"], port)
        client.loop_start()   # MQTT network loop in its own thread
        return client
    except OSError as e:
        # broker unreachable (no internet / broker offline) -> run without it.
        # You can still start the sequence locally with the 't' key.
        print(f"WARNING: could not reach MQTT broker {m['host']}:{port} ({e})")
        print("         running WITHOUT MQTT - use the 't' key to start a run.")
        return None


# ---------------------------------------------------------------------- #
# main: camera + visualization loop
# ---------------------------------------------------------------------- #
def main():
    cfg = load_config()
    state = SharedState()

    transform = vision.PixelToRobot(cfg["calibration_file"])
    detector = vision.make_detector(cfg)

    cam = cfg["camera"]
    cap = cv2.VideoCapture(cam["source"], cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera source {cam['source']}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    print("camera ready")

    r = cfg["robot"]
    robot = igus.IGUS(r["host"], r["port"],
                      gripper_open_dout=r["gripper_open_dout"],
                      gripper_close_dout=r["gripper_close_dout"],
                      gripper_delay=r["gripper_delay"],
                      gripper_mode=r.get("gripper_mode", "dual"),
                      gripper_vacuum_dout=r.get("gripper_vacuum_dout", 32),
                      gripper_blowoff_dout=r.get("gripper_blowoff_dout"))
    robot.connect()

    client = setup_mqtt(cfg, state, robot)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    print("running - waiting for start message "
          f"on {cfg['mqtt']['topic_sort']} ('t' for a test run)")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            # live detection on every frame — this feeds both the red boxes in
            # the visualization and the worker's stable detection sampling
            boxes = detector.detect(frame)
            with state.lock:
                state.live_boxes = boxes
                state.frame_counter += 1
                selected = state.selected_box
                storage_quad = state.storage_quad
                status_text = state.status_text
                start_request = state.start_request
                busy = state.busy

            # start the worker if a start message arrived
            if start_request and not busy:
                with state.lock:
                    state.start_request = None
                    state.busy = True
                threading.Thread(
                    target=sort_sequence,
                    args=(state, cfg, robot, transform, client, start_request),
                    daemon=True).start()

            # ---- draw the visualization ----
            for b in boxes:
                vision.draw_box(frame, b, vision.RED)
            if selected is not None:
                vision.draw_box(frame, selected, vision.GREEN,
                                label=f"box {selected.box_id}")
            if storage_quad is not None:
                vision.draw_storage(frame, storage_quad)
            vision.draw_status(frame, status_text)
            cv2.imshow(WIN, frame)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('t'):   # test start without MQTT: place = pick spot
                with state.lock:
                    if not state.busy:
                        state.start_request = {"inplace": True}
                        print("test start triggered (place each box back at its pick spot)")
            elif key == ord('o'):
                robot.gripper_open()
            elif key == ord('c'):
                robot.gripper_close()
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if client is not None:
            client.loop_stop()
        robot.disconnect()
        print("shut down cleanly")


if __name__ == "__main__":
    main()
