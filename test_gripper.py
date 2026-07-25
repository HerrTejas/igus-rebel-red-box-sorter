"""
Gripper test — the arm does NOT move, only digital outputs switch.

Also your tool for resolving the DOUT off-by-one question: the XMLs name
DOut31 (open) / DOut32 (close), but the CRI manual (p.19) warns the CPRog UI
numbering can be offset by 1 — so the wire numbers might be 30/31. This
script lets you fire single channels until you see the gripper react, then
saves the working pair to config.json.

Commands (type in this console):
    t             guided suction test: hold box -> vacuum ON -> box stays?
                  -> vacuum OFF -> box drops?  (start with this one)
    o             open / vacuum OFF  (uses the pair currently in config)
    c             close / vacuum ON  (uses the pair currently in config)
    f <n>         fire single channel n ON  (e.g. "f 31")
    x <n>         switch single channel n OFF
    alloff        PANIC: switch channels 20-40 OFF (stops a chattering valve)
    scan          fire channels 28-35 one at a time, ~1.5 s each, all off after
                  -> watch/listen for which single channel turns suction ON
    setvac <n>    set the single vacuum channel (suction gripper) e.g. "setvac 32"
    pair <o> <c>  set open/close channels for a DUAL/parallel gripper
    save          write the current gripper config to config.json
    q             quit (switches tested channels off first)
"""
import json
import time

import config_io
import igus

CONFIG_FILE = "config.json"


def guided_suction_test(robot):
    """Interactive box-hold test for the vacuum gripper.
    'close' = vacuum ON (grip), 'open' = vacuum OFF (release) — same channel
    semantics as the course GripperClose.xml / GripperOpen.xml."""
    print("\n--- GUIDED SUCTION TEST ---")
    if robot.gripper_mode == "single":
        print(f"single-channel mode: vacuum on DOut{robot.gripper_vacuum_dout} "
              f"(ON=grip, OFF=release)")
    else:
        print(f"dual mode: ON=DOut{robot.gripper_close_dout}  "
              f"OFF=DOut{robot.gripper_open_dout}")

    input("\n1) Hold a box flat against the suction cup, then press ENTER "
          "-> vacuum ON ... ")
    robot.gripper_close()
    answer = input("2) Let go of the box. Does it STAY on the gripper? (y/n) > ").strip().lower()
    if answer != "y":
        robot.gripper_open()   # don't leave the valve on while troubleshooting
        print("\nBox did not hold. Checklist:")
        print("  - wrong channel? type 'scan' to find which single output gives")
        print("    suction, then 'setvac <n>' and 'save'")
        print("  - compressed air / vacuum supply connected and on?")
        print("  - cup sealing? box surface must be flat and clean")
        return

    input("3) Press ENTER -> vacuum OFF, box should drop ... ")
    robot.gripper_open()
    answer = input("4) Did the box DROP? (y/n) > ").strip().lower()
    if answer == "y":
        print("\nGRIPPER TEST PASSED — channels are correct.")
        print("If this pair isn't saved yet, type 'save' now.")
    else:
        print("\nBox stuck after vacuum OFF: residual vacuum in the line.")
        print("If it always lingers, increase robot.gripper_delay in config.json")
        print("so main.py waits longer after releasing before moving up.")


def main():
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    r = cfg["robot"]

    print(f"connecting to {r['host']}:{r['port']} (motors enable, arm will NOT move)")
    robot = igus.IGUS(r["host"], r["port"],
                      gripper_open_dout=r["gripper_open_dout"],
                      gripper_close_dout=r["gripper_close_dout"],
                      gripper_delay=r["gripper_delay"],
                      gripper_mode=r.get("gripper_mode", "dual"),
                      gripper_vacuum_dout=r.get("gripper_vacuum_dout", 32),
                      gripper_blowoff_dout=r.get("gripper_blowoff_dout"))
    robot.connect()

    touched = set()
    print(f"\ncurrent pair: open=DOut{robot.gripper_open_dout} "
          f"close=DOut{robot.gripper_close_dout}")
    print("Commands" + __doc__.split("Commands")[1])
    print(">>> type 't' to start the guided box-hold test <<<")

    try:
        while True:
            parts = input("gripper> ").strip().lower().split()
            if not parts:
                continue
            cmd = parts[0]

            if cmd == "q":
                break
            elif cmd == "t":
                touched.update([robot.gripper_open_dout, robot.gripper_close_dout])
                guided_suction_test(robot)
            elif cmd == "o":
                print(f"opening (DOut{robot.gripper_close_dout} off, "
                      f"DOut{robot.gripper_open_dout} on)")
                robot.gripper_open()
                touched.update([robot.gripper_open_dout, robot.gripper_close_dout])
            elif cmd == "c":
                print(f"closing (DOut{robot.gripper_open_dout} off, "
                      f"DOut{robot.gripper_close_dout} on)")
                robot.gripper_close()
                touched.update([robot.gripper_open_dout, robot.gripper_close_dout])
            elif cmd == "alloff":
                for ch in range(20, 41):
                    robot.set_dout(ch, False)
                print("channels 20-40 switched OFF")
            elif cmd == "scan":
                print("scanning channels 28-35, one at a time. Watch the gripper "
                      "and the iRC Input/Output tab. Ctrl+C to abort.")
                for ch in range(28, 36):
                    for c in range(20, 41):
                        robot.set_dout(c, False)   # ensure only one on at a time
                    touched.add(ch)
                    print(f"  -> DOut{ch} ON (suction now?)")
                    robot.set_dout(ch, True)
                    time.sleep(1.5)
                    robot.set_dout(ch, False)
                print("scan done, all channels 20-40 off. Which channel gave suction? "
                      "set it with: setvac <n>  then  save")
            elif cmd == "f" and len(parts) == 2 and parts[1].isdigit():
                ch = int(parts[1])
                print(f"DOut{ch} -> ON")
                robot.set_dout(ch, True)
                touched.add(ch)
            elif cmd == "x" and len(parts) == 2 and parts[1].isdigit():
                ch = int(parts[1])
                print(f"DOut{ch} -> OFF")
                robot.set_dout(ch, False)
            elif cmd == "setvac" and len(parts) == 2 and parts[1].isdigit():
                robot.gripper_vacuum_dout = int(parts[1])
                robot.gripper_mode = "single"
                print(f"session vacuum channel set: DOut{parts[1]} "
                      f"(single mode, use 'save' to persist)")
            elif cmd == "pair" and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
                robot.gripper_open_dout = int(parts[1])
                robot.gripper_close_dout = int(parts[2])
                robot.gripper_mode = "dual"
                print(f"session pair set: open=DOut{parts[1]} close=DOut{parts[2]} "
                      f"(dual mode, use 'save' to persist)")
            elif cmd == "save":
                config_io.update("robot", {
                    "gripper_mode": robot.gripper_mode,
                    "gripper_vacuum_dout": robot.gripper_vacuum_dout,
                    "gripper_open_dout": robot.gripper_open_dout,
                    "gripper_close_dout": robot.gripper_close_dout,
                })
                print(f"saved gripper_mode={robot.gripper_mode}, "
                      f"vacuum=DOut{robot.gripper_vacuum_dout} to config.json")
            else:
                print("unknown command — t, o, c, f <n>, x <n>, alloff, scan, "
                      "setvac <n>, pair <o> <c>, save, q")
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        for ch in touched:
            try:
                robot.set_dout(ch, False)
            except (OSError, ConnectionError):
                break
        robot.disconnect()
        print("all tested outputs switched off, disconnected")


if __name__ == "__main__":
    main()
