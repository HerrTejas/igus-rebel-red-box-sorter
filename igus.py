"""
This module provides a Python interface for controlling the IGUS robot.
It includes classes for representing joint positions, generating commands, and managing the connection to the robot.

It allows users to connect to the robot, send commands to move joints, and receive status updates.

It is just a basic implementation and can be extended with more features as needed.

Author: Lukasz Rojek (lukasz.rojek@srh.de)
"""

# The original module was extended for this project with gripper control and
# more robust motion-completion handling.

import socket
import threading
import time


def _ang_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two angles in degrees.
    Needed because the controller may report +180 while we commanded -180
    (same physical orientation)."""
    d = (a - b) % 360.0
    if d > 180.0:
        d = 360.0 - d
    return abs(d)


class Joint:
    """A joint-space position: 6 robot axes A1..A6 plus 3 external axes E1..E3 (deg)."""
    ATTRIBUTES = ["A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3"]

    def __init__(self, *args, **kwargs):
        for attr in Joint.ATTRIBUTES:
            setattr(self, attr, 0.0)
        for attr, value in zip(Joint.ATTRIBUTES, args):
            setattr(self, attr, float(value))
        for attr in Joint.ATTRIBUTES:
            if attr in kwargs:
                setattr(self, attr, float(kwargs[attr]))

    def get_dict(self) -> dict:
        return {attr: getattr(self, attr) for attr in Joint.ATTRIBUTES}

    def close_to(self, other, tol: float = 0.5) -> bool:
        """True if all axes match within `tol` degrees."""
        if other is None:
            return False
        return all(abs(getattr(self, a) - getattr(other, a)) <= tol
                   for a in ["A1", "A2", "A3", "A4", "A5", "A6"])

    def __str__(self):
        return "Joint(" + ", ".join(f"{a}={getattr(self, a)}" for a in Joint.ATTRIBUTES) + ")"


class Cart:
    """A cartesian pose: position X,Y,Z in mm and orientation A,B,C as Euler
    angles in degrees (plus external axes E1..E3). The controller solves the
    inverse kinematics for us when we command a cartesian move."""
    ATTRIBUTES = ["X", "Y", "Z", "A", "B", "C", "E1", "E2", "E3"]

    def __init__(self, *args, **kwargs):
        for attr in Cart.ATTRIBUTES:
            setattr(self, attr, 0.0)
        for attr, value in zip(Cart.ATTRIBUTES, args):
            setattr(self, attr, float(value))
        for attr in Cart.ATTRIBUTES:
            if attr in kwargs:
                setattr(self, attr, float(kwargs[attr]))

    def get_dict(self) -> dict:
        return {attr: getattr(self, attr) for attr in Cart.ATTRIBUTES}

    def close_to(self, other, pos_tol: float = 1.0, ang_tol: float = 1.0) -> bool:
        """True if position matches within pos_tol mm and orientation within
        ang_tol degrees (with +/-180 wrap-around handling)."""
        if other is None:
            return False
        for a in ["X", "Y", "Z"]:
            if abs(getattr(self, a) - getattr(other, a)) > pos_tol:
                return False
        for a in ["A", "B", "C"]:
            if _ang_diff(getattr(self, a), getattr(other, a)) > ang_tol:
                return False
        return True

    def __str__(self):
        return "Cart(" + ", ".join(f"{a}={getattr(self, a)}" for a in Cart.ATTRIBUTES) + ")"


class CommandID:
    """Rolling message id 1..9999 — every CRI message needs a unique id so
    commands can be tracked by the controller."""

    def __init__(self, start_id: int = 1):
        self.__id = start_id
        self.__lock = threading.Lock()

    def get_id(self) -> int:
        with self.__lock:
            self.__id = 1 if self.__id >= 9999 else self.__id + 1
            return self.__id


class Command:
    """Builders for the CRI command strings we need."""
    __command_id = CommandID()

    @staticmethod
    def move_joint(A1=0.0, A2=0.0, A3=0.0, A4=0.0, A5=0.0, A6=0.0,
                   E1=0.0, E2=0.0, E3=0.0, vel=50.0):
        return (f"CRISTART {Command.__command_id.get_id()} CMD Move Joint "
                f"{A1} {A2} {A3} {A4} {A5} {A6} {E1} {E2} {E3} {vel} CRIEND")

    @staticmethod
    def move_cart(X=0.0, Y=0.0, Z=0.0, A=0.0, B=0.0, C=0.0,
                  E1=0.0, E2=0.0, E3=0.0, vel=50.0):
        return (f"CRISTART {Command.__command_id.get_id()} CMD Move Cart "
                f"{X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {vel} CRIEND")

    @staticmethod
    def dout(channel: int, state: bool):
        """Set a digital output (0..63). The gripper valves/fingers hang on
        two of these channels (see GripperOpen.xml / GripperClose.xml)."""
        return (f"CRISTART {Command.__command_id.get_id()} CMD DOUT "
                f"{int(channel)} {'true' if state else 'false'} CRIEND")

    @staticmethod
    def alive_jog():
        return (f"CRISTART {Command.__command_id.get_id()} ALIVEJOG "
                f"0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND")

    @staticmethod
    def connect():
        return f"CRISTART {Command.__command_id.get_id()} CMD Connect CRIEND"

    @staticmethod
    def enable():
        return f"CRISTART {Command.__command_id.get_id()} CMD Enable CRIEND"

    @staticmethod
    def disconnect():
        return f"CRISTART {Command.__command_id.get_id()} CMD Disconnect CRIEND"

    @staticmethod
    def active_multiple_clients():
        return f"CRISTART {Command.__command_id.get_id()} CMD SetActive true CRIEND"


class IGUS:
    """Connection manager + high level motion API for the ReBeL.

    Usage:
        robot = IGUS("192.168.3.11", 3920)
        robot.connect()
        robot.go_to(Joint(A3=90, A5=90))                  # joint move
        robot.go_to(Cart(300, 0, 250, 180, 0, 180))       # cartesian move
        robot.gripper_close()
        robot.disconnect()
    """

    def __init__(self, host="192.168.3.11", port=3920,
                 gripper_open_dout=31, gripper_close_dout=32,
                 gripper_delay=1.5, gripper_mode="dual", gripper_vacuum_dout=32,
                 gripper_blowoff_dout=None):
        self.__host = host
        self.__port = port
        self.__sock = None
        self.__send_lock = threading.Lock()
        self.__current_joint = None
        self.__current_cart = None
        self.wait = True                 # block go_to() until motion finished
        self.move_timeout = 60.0         # give up waiting after this many s
        # gripper_mode "single": suction gripper, one output (vacuum on/off).
        # gripper_mode "dual":   parallel gripper, two complementary outputs.
        self.gripper_mode = gripper_mode
        self.gripper_vacuum_dout = gripper_vacuum_dout
        # optional blow-off output: pulsed on release to actively break the
        # vacuum (needed when the cup holds via a check valve). None = no blow-off.
        self.gripper_blowoff_dout = gripper_blowoff_dout
        self.gripper_open_dout = gripper_open_dout
        self.gripper_close_dout = gripper_close_dout
        self.gripper_delay = gripper_delay
        self.callback_read_msg = None    # optional hook, gets Joint/Cart updates

    # ------------------------------------------------------------------ #
    # connection handling
    # ------------------------------------------------------------------ #
    def connect(self):
        self.__sock = socket.create_connection((self.__host, self.__port), timeout=5)
        self.__sock.settimeout(None)
        print(f"connected to IGUS at {self.__host}:{self.__port}")

        self.__keep_alive()      # controller drops us without ALIVEJOG spam
        self.__keep_reading()    # parse STATUS messages in the background
        time.sleep(1)

        self.send(Command.active_multiple_clients())
        time.sleep(1)
        self.send(Command.connect())
        time.sleep(1)
        self.send(Command.enable())
        time.sleep(3)

        # wait until we have received at least one position report
        t0 = time.time()
        while self.__current_joint is None and self.__current_cart is None:
            if time.time() - t0 > 10:
                raise TimeoutError("no position status received from robot")
            time.sleep(0.01)
        print("robot is ready")

    def disconnect(self):
        if self.__sock:
            try:
                self.send(Command.disconnect())
            except OSError:
                pass
            self.__sock.close()
            self.__sock = None

    def send(self, command: str):
        if not self.__sock:
            raise ConnectionError("socket is not connected, call connect() first")
        with self.__send_lock:
            self.__sock.sendall(command.encode("utf-8"))
        time.sleep(0.05)

    # ------------------------------------------------------------------ #
    # motion
    # ------------------------------------------------------------------ #
    @property
    def current_joint(self):
        return self.__current_joint

    @property
    def current_cart(self):
        return self.__current_cart

    def go_to(self, pos, vel=50.0):
        """Move to a Joint or Cart target. Blocks until the robot reports the
        target position (within tolerance) if self.wait is True.

        `vel` means two different things (CRI spec 4.5): for a Joint move it is
        percent of max velocity [1..100]; for a Cart move it is mm/s, and is NOT
        capped at 100. The iRC override slider does not scale either — the spec
        applies it to jog motion and program replay only, and CMD Move is
        neither."""
        if isinstance(pos, Joint):
            self.send(Command.move_joint(**pos.get_dict(), vel=vel))
            if self.wait:
                self.__wait_until(lambda: pos.close_to(self.__current_joint))
        elif isinstance(pos, Cart):
            self.send(Command.move_cart(**pos.get_dict(), vel=vel))
            if self.wait:
                self.__wait_until(lambda: pos.close_to(self.__current_cart))
        else:
            raise TypeError(f"pos must be Joint or Cart, got {type(pos)}")

    def go_to_zero(self, vel=50.0):
        self.go_to(Joint(), vel=vel)

    def go_to_L(self, vel=50.0):
        """The 'L' pose (A3=90, A5=90): elbow and wrist bent so the tool points
        down. Good sorting start pose and keeps the arm away from the
        stretched-out singularity of the all-zero pose."""
        self.go_to(Joint(A3=90.0, A5=90.0), vel=vel)

    def __wait_until(self, reached, poll=0.02):
        t0 = time.time()
        while not reached():
            if time.time() - t0 > self.move_timeout:
                # Do NOT continue silently — if the arm never reached the target
                # (robot disabled, speed override 0, joint limit, error state),
                # continuing would fire the gripper mid-air. Abort instead.
                raise TimeoutError(
                    f"move did not finish within {self.move_timeout:.0f}s. "
                    f"Is the robot enabled, error-free, and speed override > 0?")
            time.sleep(poll)

    # ------------------------------------------------------------------ #
    # gripper (two digital outputs, see GripperOpen.xml / GripperClose.xml)
    # ------------------------------------------------------------------ #
    def set_dout(self, channel: int, state: bool):
        self.send(Command.dout(channel, state))

    def gripper_open(self):
        """Release. Suction gripper (single): vacuum OFF, then a blow-off pulse
        to actively break the vacuum (a check-valve cup won't let go otherwise).
        Parallel (dual): open."""
        if self.gripper_mode == "single":
            self.set_dout(self.gripper_vacuum_dout, False)      # stop the vacuum
            if self.gripper_blowoff_dout is not None:
                self.set_dout(self.gripper_blowoff_dout, True)  # blow-off to release
                time.sleep(self.gripper_delay)
                self.set_dout(self.gripper_blowoff_dout, False) # stop the blow-off
            else:
                time.sleep(self.gripper_delay)
        else:
            self.set_dout(self.gripper_close_dout, False)
            self.set_dout(self.gripper_open_dout, True)
            time.sleep(self.gripper_delay)

    def gripper_close(self):
        """Grip. Suction gripper (single): vacuum ON. Parallel (dual): close.
        The DOUT is sent twice with a short gap so an occasionally-dropped
        command can't leave the gripper un-triggered on a pick."""
        if self.gripper_mode == "single":
            self.set_dout(self.gripper_vacuum_dout, True)
            time.sleep(0.1)
            self.set_dout(self.gripper_vacuum_dout, True)   # resend for reliability
        else:
            self.set_dout(self.gripper_open_dout, False)
            self.set_dout(self.gripper_close_dout, True)
            time.sleep(0.1)
            self.set_dout(self.gripper_close_dout, True)
        time.sleep(self.gripper_delay)

    # ------------------------------------------------------------------ #
    # background workers
    # ------------------------------------------------------------------ #
    def __update_status(self, status: str):
        data = status.split(" ")

        if "POSJOINTCURRENT" in data:
            idx = data.index("POSJOINTCURRENT") + 1
            try:
                self.__current_joint = Joint(*data[idx:idx + 9])
            except (ValueError, IndexError):
                pass
            else:
                if self.callback_read_msg:
                    self.callback_read_msg(self.__current_joint)

        if "POSCARTROBOT" in data:
            idx = data.index("POSCARTROBOT") + 1
            try:
                self.__current_cart = Cart(*data[idx:idx + 6])
            except (ValueError, IndexError):
                pass
            else:
                if self.callback_read_msg:
                    self.callback_read_msg(self.__current_cart)

    def __keep_reading(self):
        def read_thread():
            while self.__sock:
                try:
                    buff = self.__sock.recv(4096).decode("utf-8", errors="ignore")
                except OSError:
                    return
                if not buff:
                    return
                # take the last complete CRISTART..CRIEND block in the buffer
                block = buff[buff.find("CRISTART"): buff.rfind("CRIEND") + 6]
                if block:
                    self.__update_status(block)

        threading.Thread(target=read_thread, daemon=True).start()

    def __keep_alive(self, interval: float = 0.1):
        def keep_alive_thread():
            while self.__sock:
                try:
                    self.send(Command.alive_jog())
                except (OSError, ConnectionError):
                    return
                time.sleep(interval)

        threading.Thread(target=keep_alive_thread, daemon=True).start()


if __name__ == "__main__":
    # tiny smoke test: connect, go to L pose, toggle gripper, disconnect
    robot = IGUS("192.168.3.11", 3920)
    try:
        robot.connect()
        robot.go_to_L(vel=50.0)
        robot.gripper_open()
        robot.gripper_close()
        robot.gripper_open()
    finally:
        robot.disconnect()
        print("disconnected")
