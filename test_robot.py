"""
Arm motion test — one small, slow step at a time.

Every step prints WHAT the robot is about to do and waits for you to press
ENTER before it moves (type 's' + ENTER to skip a step, 'q' + ENTER to quit).
All moves run at a crawl (vel 15). Keep a hand near the E-stop anyway.

Steps:
    0  connect + print current position        (NO motion)
    1  L pose (A3=90, A5=90)                   joint move, the working home
    2  wrist spin A6 +20 deg and back          smallest safest joint motion
    3  lift tool 40 mm straight up and back    first cartesian move (relative)
    4  slide tool 40 mm in +X and back         cartesian in the table plane
    5  back to L pose

If step 3 or 4 misbehaves (weird detour, wrong direction), you are near a
singularity or the pose in iRC disagrees — stop and check before calibrating.
"""
import json

import igus

CONFIG_FILE = "config.json"
VEL = 40.0     # moderate; the iRC speed-override slider scales this further


def ask(step_text):
    answer = input(f"\nNEXT: {step_text}\n      ENTER=go  s=skip  q=quit > ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt
    return answer != "s"


def show_position(robot):
    print(f"      joints: {robot.current_joint}")
    print(f"      cart:   {robot.current_cart}")


def main():
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    r = cfg["robot"]

    print(f"connecting to {r['host']}:{r['port']} — this ENABLES THE MOTORS.")
    input("ENTER to connect (ctrl+c to abort) > ")

    robot = igus.IGUS(r["host"], r["port"])
    robot.connect()

    try:
        print("\nSTEP 0 — current position (no motion):")
        show_position(robot)

        if ask("STEP 1 — move to L pose (A3=90, A5=90), joint move"):
            robot.go_to_L(vel=VEL)
            show_position(robot)

        if ask("STEP 2 — wrist spin: A6 to +20 deg and back"):
            robot.go_to(igus.Joint(A3=90.0, A5=90.0, A6=20.0), vel=VEL)
            robot.go_to(igus.Joint(A3=90.0, A5=90.0, A6=0.0), vel=VEL)
            show_position(robot)

        # cartesian steps move RELATIVE to wherever the tool is right now.
        # We move DOWN (toward the table) not up: from the L pose, lifting the
        # tool higher drives wrist axis A5 into its 95 deg limit, whereas
        # descending is both reachable and the direction picking actually uses.
        if ask("STEP 3 — cartesian: lower tool 40 mm straight down, then back"):
            here = robot.current_cart
            down = igus.Cart(**{**here.get_dict(), "Z": here.Z - 40.0})
            robot.go_to(down, vel=VEL)
            robot.go_to(here, vel=VEL)
            show_position(robot)

        if ask("STEP 4 — cartesian: slide tool 40 mm in +X, then back"):
            here = robot.current_cart
            out = igus.Cart(**{**here.get_dict(), "X": here.X + 40.0})
            robot.go_to(out, vel=VEL)
            robot.go_to(here, vel=VEL)
            show_position(robot)

        if ask("STEP 5 — back to L pose"):
            robot.go_to_L(vel=VEL)
            show_position(robot)

        print("\nARM TEST DONE. If every step looked right, run test_gripper.py next.")

    except KeyboardInterrupt:
        print("\naborted by user")
    finally:
        robot.disconnect()
        print("disconnected")


if __name__ == "__main__":
    main()
