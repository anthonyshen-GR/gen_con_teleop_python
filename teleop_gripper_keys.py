#!/usr/bin/env python3
"""
teleop_gripper_curses.py — Keyboard teleoperation demo for the DAS Gripper
Controller, using curses for input instead of pynput.

Why curses: pynput needs an X server for its global key hook, which a
plain WSL2/SSH terminal usually doesn't have. curses reads keys straight
from the terminal itself, so it works anywhere you have a shell — no
display required. This is a straight swap of the input layer; the
control/encoder logic is identical to teleop_gripper.py.

IMPORTANT: run only this ONE script. Do not also have start_gripper.py
(or any other script) open on the same serial port at the same time —
two processes reading/writing the same port will corrupt the protocol
stream (you'll see "ValueError: 0 is not a valid RecordType" or
"multiple access on port" errors, which is a symptom of exactly that).

Controls (must have this terminal focused — that's how curses reads keys):
    Right / Up    -> open (hold to keep moving)
    Left  / Down  -> close (hold to keep moving)
    Space         -> stop / hold current target
    Home          -> jump to full open  (0.103 m)
    End           -> jump to full close (0.000 m)
    R             -> re-center to 0.050 m
    Q             -> disable motor and quit

Usage:
    python3 teleop_gripper.py left
    python3 teleop_gripper.py right --port /dev/ttyUSB0

Requires: pyserial   (pip install pyserial --break-system-packages)
No pynput needed.
"""

import argparse
import curses
import os
import struct
import sys
import threading
import time

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

from scripts.databus import DataBus  # noqa: E402


MIN_DIST = 0.0
MAX_DIST = 0.103  # confirmed range from databus.py / system.py

SIDE_PORTS = {
    "left": "/dev/ttyDeviceLeft",
    "right": "/dev/ttyDeviceRight",
}


class TeleopState:
    def __init__(self, start_pos: float, step_rate: float):
        self.lock = threading.Lock()
        self.target = start_pos
        self.direction = 0
        self.step_rate = step_rate
        self.encoder_value = None
        self.encoder_ts = 0.0
        self.running = True
        self.last_key_ts = 0.0  # used to auto-release direction if no key repeats arrive

    def set_direction(self, d: int):
        with self.lock:
            self.direction = d
            self.last_key_ts = time.time()

    def nudge(self, dt: float):
        with self.lock:
            # Auto-stop if no key event has refreshed direction recently.
            # Terminals send repeat key-press events while held, with gaps
            # between them, so a short grace window (~0.35s) keeps motion
            # smooth without it running away after you release the key.
            if self.direction != 0 and (time.time() - self.last_key_ts) > 0.35:
                self.direction = 0
            if self.direction != 0:
                self.target += self.direction * self.step_rate * dt
                self.target = max(MIN_DIST, min(MAX_DIST, self.target))
            return self.target

    def jump_to(self, value: float):
        with self.lock:
            self.target = max(MIN_DIST, min(MAX_DIST, value))
            self.direction = 0
            return self.target

    def update_encoder(self, value: float):
        with self.lock:
            self.encoder_value = value
            self.encoder_ts = time.time()

    def snapshot(self):
        with self.lock:
            return self.target, self.encoder_value, self.encoder_ts


def encoder_callback_factory(state: TeleopState):
    def _cb(record_data: bytes):
        try:
            value = struct.unpack(">f", record_data)[0]
            state.update_encoder(value)
        except Exception:
            pass  # avoid printing from a background thread while curses owns the screen
    return _cb


def control_loop(databus: DataBus, state: TeleopState, hz: float):
    interval = 1.0 / hz
    last = time.time()
    while state.running:
        now = time.time()
        dt = now - last
        last = now
        target = state.nudge(dt)
        databus.set_target_distance(target)
        elapsed = time.time() - now
        time.sleep(max(0.0, interval - elapsed))


def run_ui(stdscr, databus: DataBus, state: TeleopState, side: str, port: str):
    curses.curs_set(0)
    stdscr.nodelay(True)   # non-blocking getch
    stdscr.timeout(50)     # ~20Hz redraw / key-poll rate
    stdscr.keypad(True)

    bar_width = 30
    quit_flag = False

    while not quit_flag:
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1

        if key != -1:
            if key in (curses.KEY_RIGHT, curses.KEY_UP):
                state.set_direction(+1)
            elif key in (curses.KEY_LEFT, curses.KEY_DOWN):
                state.set_direction(-1)
            elif key == ord(' '):
                state.set_direction(0)
            elif key == curses.KEY_HOME:
                state.jump_to(MAX_DIST)
            elif key == curses.KEY_END:
                state.jump_to(MIN_DIST)
            elif key in (ord('r'), ord('R')):
                state.jump_to(0.05)
            elif key in (ord('q'), ord('Q')):
                quit_flag = True

        target, encoder, ts = state.snapshot()
        age = time.time() - ts if ts else None
        if encoder is None:
            enc_str = "waiting..."
        elif age is not None and age > 1.0:
            enc_str = f"{encoder:.4f} m (stale {age:.1f}s)"
        else:
            enc_str = f"{encoder:.4f} m"

        filled = int((target - MIN_DIST) / (MAX_DIST - MIN_DIST) * bar_width)
        bar = "#" * filled + "-" * (bar_width - filled)

        stdscr.erase()
        stdscr.addstr(0, 0, "DAS Gripper Controller — Keyboard Teleop Demo")
        stdscr.addstr(1, 0, f"Side: {side}   Port: {port}")
        stdscr.addstr(2, 0, "Arrows=open/close  Space=stop  Home/End=full open/close  R=center  Q=quit")
        stdscr.addstr(4, 0, f"[{bar}]")
        stdscr.addstr(5, 0, f"target:  {target:.4f} m")
        stdscr.addstr(6, 0, f"encoder: {enc_str}")
        stdscr.refresh()

    return


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop demo for DAS Gripper Controller (curses input)")
    parser.add_argument("side", choices=["left", "right"])
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--gripper-type", type=str, default="default_gripper",
                         choices=["default_gripper", "tactile_gripper", "soft_gripper"])
    parser.add_argument("--step-rate", type=float, default=0.05)
    parser.add_argument("--start", type=float, default=0.05)
    parser.add_argument("--hz", type=float, default=30.0)
    args = parser.parse_args()

    port = args.port or SIDE_PORTS[args.side]

    print(f"Connecting to {port} ...")
    state = TeleopState(start_pos=max(MIN_DIST, min(MAX_DIST, args.start)),
                         step_rate=args.step_rate)
    try:
        databus = DataBus(
            tty_port=port,
            baudrate=921600,
            encoder_freq=30,
            encoder_callback=encoder_callback_factory(state),
            gripper_type=args.gripper_type,
        )
    except Exception as e:
        print(f"Failed to open driver on {port}: {e}")
        print("If start_gripper.py or another instance of this script is already")
        print("running against this port, close it first — only one process can")
        print("hold the serial port at a time.")
        sys.exit(1)

    ctrl_thread = threading.Thread(target=control_loop, args=(databus, state, args.hz), daemon=True)
    ctrl_thread.start()

    try:
        curses.wrapper(run_ui, databus, state, args.side, port)
    finally:
        print("\nShutting down: disabling motor and closing serial port...")
        state.running = False
        time.sleep(0.1)
        try:
            databus.disable_motor()
            time.sleep(0.1)
        except Exception:
            pass
        databus.stop()
        print("Done.")


if __name__ == "__main__":
    main()
