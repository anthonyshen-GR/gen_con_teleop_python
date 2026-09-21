#!/usr/bin/env python3
"""
teleop_client.py — [LINUX NATIVE VERSION] hand-pinch teleop CLIENT.

This process never opens the serial port. It only runs the webcam +
MediaPipe hand-pinch tracking, computes a target distance, and sends it
over UDP to a driver process (e.g. start_gripper.py) that owns the gripper
hardware and prints its own encoder readings. That split lets the
tracking/UI process and the hardware-owning process live on different
machines, or just keeps camera/UI churn off the process that's talking
to the serial port. Need to modify start_gripper.py to accept teleop CLI. 

Built on the native-Linux display model (see teleop_gripper.py):
a real cv2 window + cv2.waitKey() for hotkeys, no HTTP MJPEG server and
no termios/tty raw-mode terminal reading. The old client/driver split
was written around an HTTP-stream-to-Windows-browser flow for WSL, but
WSL is a bad fit for this exact configuration anyway — a driver process
holding the serial port in one WSL instance/terminal, plus a client
process independently opening the webcam and pushing a UDP stream, runs
into the same device-passthrough and multi-process-per-port limitations
this whole client/driver split exists to avoid. On native Linux none of
that applies: the webcam just opens, the window just shows up, and UDP
is UDP.

Run the driver(s) first, each in its own terminal:
    python3 start_gripper.py left
    python3 start_gripper.py right      # only needed for dual-gripper ('both') mode

Then run this client:
    python3 teleop_client.py left
    python3 teleop_client.py both   # dual-hand: left hand -> left driver,
                                     # right hand -> right driver

Controls (window must be focused):
    n        -> capture current pinch as CLOSED reference (per visible hand)
    f        -> capture current pinch as OPEN reference (per visible hand)
    SPACE    -> freeze target(s) (ignore hand tracking until pressed again)
    q / ESC  -> quit (does not touch the driver's motor — the driver keeps
                whatever target it last received; stop it there if needed)
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        HandLandmarker,
        HandLandmarkerOptions,
        RunningMode,
    )
except ImportError:
    print("Missing dependency: mediapipe. Install with: pip install mediapipe")
    sys.exit(1)

_here = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_here, "hand_landmarker.task")

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]

MIN_DIST = 0.0
MAX_DIST_BY_TYPE = {
    "default_gripper": 0.103,
    "tactile_gripper": 0.0998,
    "soft_gripper": 0.103,
}
THUMB_TIP = 4
INDEX_TIP = 8
WRIST = 0
MIDDLE_MCP = 9

DEFAULT_DRIVER_PORT = 5566       # single-gripper / left driver, in 'both' mode
DEFAULT_RIGHT_DRIVER_PORT = 5567  # right driver, 'both' mode only

WINDOW_NAME = "DAS Gripper Teleop (client)"


class TeleopState:
    def __init__(self, smoothing: float, max_dist: float):
        self.lock = threading.Lock()
        self.max_dist = max_dist
        self.target = min(0.05, max_dist)
        self.running = True
        self.frozen = False
        self.smoothing = smoothing
        self.smoothed_ratio = None
        self.calib_near = None
        self.calib_far = None

    def set_target(self, value):
        with self.lock:
            if not self.frozen:
                self.target = max(MIN_DIST, min(self.max_dist, value))

    def toggle_freeze(self):
        with self.lock:
            self.frozen = not self.frozen
            return self.frozen

    def get_target(self):
        with self.lock:
            return self.target


class DriverLink:
    """Sends target-distance commands to a driver process over UDP."""

    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_target(self, value: float):
        try:
            self.sock.sendto(struct.pack(">f", value), self.addr)
        except OSError as e:
            print(f"Failed to send target to driver at {self.addr}: {e}")


def control_loop(link: DriverLink, state: TeleopState, hz: float):
    interval = 1.0 / hz
    while state.running:
        link.send_target(state.get_target())
        time.sleep(interval)


def _draw_hand(frame, pts):
    for a, b in HAND_CONNECTIONS:
        cv2.line(
            frame,
            tuple(map(int, pts[a])),
            tuple(map(int, pts[b])),
            (0, 200, 0),
            2,
        )
    for x, y in pts:
        cv2.circle(frame, (int(x), int(y)), 3, (0, 200, 0), -1)


def _update_ratio_from_landmarks(frame, pts, state: TeleopState):
    thumb = np.array(pts[THUMB_TIP])
    index = np.array(pts[INDEX_TIP])
    wrist = np.array(pts[WRIST])
    mid_mcp = np.array(pts[MIDDLE_MCP])
    pinch_dist = np.linalg.norm(thumb - index)
    hand_scale = np.linalg.norm(wrist - mid_mcp)

    if hand_scale > 1e-3:
        ratio = pinch_dist / hand_scale
        if state.smoothed_ratio is None:
            state.smoothed_ratio = ratio
        else:
            a = state.smoothing
            state.smoothed_ratio = a * state.smoothed_ratio + (1 - a) * ratio

    cv2.line(
        frame,
        tuple(thumb.astype(int)),
        tuple(index.astype(int)),
        (0, 255, 0),
        2,
    )
    cv2.circle(frame, tuple(thumb.astype(int)), 6, (255, 0, 0), -1)
    cv2.circle(frame, tuple(index.astype(int)), 6, (0, 0, 255), -1)


def _apply_calibration(state: TeleopState):
    if (
        state.calib_near is not None
        and state.calib_far is not None
        and state.smoothed_ratio is not None
    ):
        near, far = state.calib_near, state.calib_far
        if abs(far - near) > 1e-6:
            frac = (state.smoothed_ratio - near) / (far - near)
            frac = max(0.0, min(1.0, frac))
            state.set_target(MIN_DIST + frac * (state.max_dist - MIN_DIST))


def _calib_status(state: TeleopState) -> str:
    if state.calib_near and state.calib_far:
        return f"CALIBRATED (near={state.calib_near:.2f}, far={state.calib_far:.2f})"
    return "NOT CALIBRATED (press 'n' then 'f')"


def _handle_key(key: int, states: dict) -> bool:
    """Apply a cv2.waitKey() result to one or more labeled TeleopState
    objects (e.g. {"L": state_left, "R": state_right}, or {"": state} for
    single-gripper mode). Returns False if quit was requested."""
    if key == -1:
        return True
    ch = chr(key & 0xFF) if 0 <= (key & 0xFF) < 128 else ""
    if ch == "q" or key == 27:  # q or ESC
        for st in states.values():
            st.running = False
        print("\n[Quit requested]")
        return False
    elif ch == "n":
        msg = []
        for label, st in states.items():
            if st.smoothed_ratio is not None:
                st.calib_near = st.smoothed_ratio
                msg.append(f"{label + '=' if label else ''}{st.calib_near:.3f}")
        if msg:
            print(f"\n>> Calibrated CLOSED: {', '.join(msg)}")
    elif ch == "f":
        msg = []
        for label, st in states.items():
            if st.smoothed_ratio is not None:
                st.calib_far = st.smoothed_ratio
                msg.append(f"{label + '=' if label else ''}{st.calib_far:.3f}")
        if msg:
            print(f"\n>> Calibrated OPEN: {', '.join(msg)}")
    elif ch == " ":
        frozen = None
        for st in states.values():
            frozen = st.toggle_freeze()
        print("\n>> FROZEN" if frozen else "\n>> LIVE")
    return True


def run_single(args):
    """Single-gripper hand-pinch teleop client: one webcam hand drives one
    driver process over UDP."""
    max_dist = MAX_DIST_BY_TYPE[args.gripper_type]
    state = TeleopState(smoothing=args.smoothing, max_dist=max_dist)
    link = DriverLink(host=args.driver_host, port=args.driver_port)

    threading.Thread(target=control_loop, args=(link, state, args.hz), daemon=True).start()

    cap = cv2.VideoCapture(args.webcam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.webcam_index)

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    landmarker = HandLandmarker.create_from_options(
        HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print("=" * 60)
    print(f"Window '{WINDOW_NAME}' is open — click it to focus.")
    print(f"Sending commands to driver at {args.driver_host}:{args.driver_port}")
    print("(Encoder readings are printed in the DRIVER terminal)")
    print("  'n'   -> Calibrate Pinch CLOSED")
    print("  'f'   -> Calibrate Pinch OPEN")
    print("  SPACE -> Freeze / Unfreeze")
    print("  'q'   -> Quit")
    print("=" * 60)

    frame_timestamp_ms = 0
    try:
        while state.running:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            frame_timestamp_ms += 33
            result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

            if result.hand_landmarks:
                lm = result.hand_landmarks[0]
                h, w, _ = frame.shape
                pts = [(p.x * w, p.y * h) for p in lm]
                _draw_hand(frame, pts)
                _update_ratio_from_landmarks(frame, pts, state)

            _apply_calibration(state)

            target = state.get_target()
            overlay = [
                f"Target: {target:.4f} m  (encoder is in the DRIVER terminal)",
                f"Calib: {_calib_status(state)}",
                f"{'FROZEN' if state.frozen else 'LIVE'}  |  n=near, f=far, space=freeze, q=quit",
            ]
            for i, line in enumerate(overlay):
                cv2.putText(
                    frame,
                    line,
                    (10, 25 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1)
            if not _handle_key(key, {"": state}):
                break

    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        time.sleep(0.1)
        cap.release()
        cv2.destroyAllWindows()
        try:
            landmarker.close()
        except Exception:
            pass
        print("Shutdown complete.")


def run_dual(args):
    """Dual-gripper hand-pinch teleop client: your left hand's pinch drives
    the left driver over UDP, your right hand drives the right driver,
    both from one webcam feed. Handedness routing is the same as
    teleop_gripper_native.py's run_dual()."""
    left_host = args.left_driver_host or args.driver_host
    right_host = args.right_driver_host or args.driver_host
    left_port = args.left_driver_port
    right_port = args.right_driver_port

    max_dist = MAX_DIST_BY_TYPE[args.gripper_type]
    state_left = TeleopState(smoothing=args.smoothing, max_dist=max_dist)
    state_right = TeleopState(smoothing=args.smoothing, max_dist=max_dist)
    link_left = DriverLink(host=left_host, port=left_port)
    link_right = DriverLink(host=right_host, port=right_port)

    threading.Thread(
        target=control_loop, args=(link_left, state_left, args.hz), daemon=True
    ).start()
    threading.Thread(
        target=control_loop, args=(link_right, state_right, args.hz), daemon=True
    ).start()

    cap = cv2.VideoCapture(args.webcam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.webcam_index)

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    landmarker = HandLandmarker.create_from_options(
        HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print("=" * 60)
    print(f"Window '{WINDOW_NAME}' is open — click it to focus.")
    print(f"  Left hand  -> left driver  at {left_host}:{left_port}")
    print(f"  Right hand -> right driver at {right_host}:{right_port}")
    print("(Encoder readings are printed in each DRIVER's own terminal)")
    print("  'n'   -> Calibrate Pinch CLOSED (per visible hand)")
    print("  'f'   -> Calibrate Pinch OPEN (per visible hand)")
    print("  SPACE -> Freeze / Unfreeze both")
    print("  'q'   -> Quit")
    if args.invert_hands:
        print("  (--invert-hands is ON: mediapipe Left/Right swapped)")
    print("=" * 60)

    frame_timestamp_ms = 0
    try:
        while state_left.running and state_right.running:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            frame_timestamp_ms += 33
            result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

            h, w, _ = frame.shape
            seen = {"Left": False, "Right": False}

            if result.hand_landmarks:
                for idx, lm in enumerate(result.hand_landmarks):
                    # MediaPipe classifies handedness assuming a mirrored
                    # (selfie-view) input, which matches the cv2.flip()
                    # above, so "Left"/"Right" here already correspond to
                    # the user's own left/right hand. If yours come out
                    # swapped, rerun with --invert-hands.
                    label = "Left"
                    if result.handedness and idx < len(result.handedness):
                        label = result.handedness[idx][0].category_name
                    if args.invert_hands:
                        label = "Right" if label == "Left" else "Left"

                    state = state_left if label == "Left" else state_right
                    seen[label] = True

                    pts = [(p.x * w, p.y * h) for p in lm]
                    _draw_hand(frame, pts)
                    _update_ratio_from_landmarks(frame, pts, state)

            _apply_calibration(state_left)
            _apply_calibration(state_right)

            overlay = [
                f"L target: {state_left.get_target():.4f} m"
                f"{'  [tracking]' if seen['Left'] else ''}",
                f"L calib: {_calib_status(state_left)}",
                f"R target: {state_right.get_target():.4f} m"
                f"{'  [tracking]' if seen['Right'] else ''}",
                f"R calib: {_calib_status(state_right)}",
                f"{'FROZEN' if (state_left.frozen or state_right.frozen) else 'LIVE'}  |  "
                "n=near, f=far, space=freeze, q=quit",
            ]
            for i, line in enumerate(overlay):
                cv2.putText(
                    frame,
                    line,
                    (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1)
            if not _handle_key(key, {"L": state_left, "R": state_right}):
                break

    except KeyboardInterrupt:
        pass
    finally:
        state_left.running = False
        state_right.running = False
        time.sleep(0.1)
        cap.release()
        cv2.destroyAllWindows()
        try:
            landmarker.close()
        except Exception:
            pass
        print("Shutdown complete.")


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Hand-pinch teleop client (native Linux; sends commands to separately running driver(s))"
    )
    parser.add_argument("side", choices=["left", "right", "both"],
                         help="Which gripper to drive. 'both' teleops left+right "
                              "grippers at once with your left/right hands, each "
                              "talking to its own driver process over UDP.")
    parser.add_argument("--gripper-type", type=str, default="default_gripper",
                         choices=["default_gripper", "tactile_gripper", "soft_gripper"],
                         help="Must match the driver's(s') --gripper-type so range clamping agrees")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--smoothing", type=float, default=0.4)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--driver-host", type=str, default="127.0.0.1",
                         help="Driver host for single-gripper mode; also the default "
                              "host for both drivers in 'both' mode.")
    parser.add_argument("--driver-port", type=int, default=DEFAULT_DRIVER_PORT,
                         help="Driver port for single-gripper mode.")
    parser.add_argument("--left-driver-host", type=str, default=None,
                         help="'both' mode only: override host for the left driver.")
    parser.add_argument("--left-driver-port", type=int, default=DEFAULT_DRIVER_PORT,
                         help="'both' mode only: port for the left driver.")
    parser.add_argument("--right-driver-host", type=str, default=None,
                         help="'both' mode only: override host for the right driver.")
    parser.add_argument("--right-driver-port", type=int, default=DEFAULT_RIGHT_DRIVER_PORT,
                         help="'both' mode only: port for the right driver.")
    parser.add_argument("--invert-hands", action="store_true",
                         help="'both' mode only: swap which detected hand "
                              "(Left/Right) drives which driver.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    if args.side == "both":
        run_dual(args)
    else:
        run_single(args)


if __name__ == "__main__":
    main()