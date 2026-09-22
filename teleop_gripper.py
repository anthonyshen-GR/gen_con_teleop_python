#!/usr/bin/env python3
"""
teleop_gripper.py — [LINUX NATIVE VERSION] hand-pinch teleop for DAS Gripper. Cretaes OpenCV window called 
"DAS Gripper Teleop" showing your webcam feed (mirrored) with the hand skeleton drawn on it. 
The overlay text shows the target distance, the encoder reading, calibration status, and whether it's LIVE or FROZEN.
Standalone script, do not run with pre-existing scripts (start_gripper.py).

Modes:
    single gripper:
        python3 teleop_gripper.py left
        python3 teleop_gripper.py right

    dual gripper: plug in both gripper controllers and drive them with
    both hands at once — your left hand drives the left gripper, your
    right hand drives the right gripper.
        python3 teleop_gripper.py dual

Controls (window must be focused):
    n        -> capture current pinch as CLOSED reference (only for hand(s)
                actually visible in frame right now — see visible_this_frame)
    f        -> capture current pinch as OPEN reference (same rule)
    SPACE    -> freeze target(s) (ignore hand tracking until pressed again)
    q / ESC  -> disable motor(s) and quit

Calibration in dual mode can be done either simultaneously (both hands in
frame, pinch both closed, press 'n', spread both open, press 'f') or
sequentially, one hand at a time (pinch left, press 'n' while your right
hand types — right hand doesn't need to be in frame; move on to right hand
next). 'n'/'f' only ever touch hands that are visible in the CURRENT frame,
so calibrating one hand never overwrites the other hand's calibration with
a stale reading.

Usage: python teleop_gripper.py left
       python3 teleop_gripper.py dual
       python3 teleop_gripper.py dual --left-port /dev/ttyUSB0 --right-port /dev/ttyUSB1
"""

import argparse
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

from scripts.databus import DataBus  # noqa: E402

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
MAX_DIST = 0.103
THUMB_TIP = 4
INDEX_TIP = 8
WRIST = 0
MIDDLE_MCP = 9

SIDE_PORTS = {
    "left": "/dev/ttyDeviceLeft",
    "right": "/dev/ttyDeviceRight",
}

WINDOW_NAME = "DAS Gripper Teleop"


class HandTeleopState:
    def __init__(self, smoothing: float):
        self.lock = threading.Lock()
        self.target = 0.05
        self.encoder_value = None
        self.encoder_ts = 0.0
        self.running = True
        self.frozen = False
        self.smoothing = smoothing
        self.smoothed_ratio = None
        self.calib_near = None
        self.calib_far = None
        # True only for the frame(s) in which this hand was actually
        # detected — reset to False every frame before detection runs, and
        # set True only if this hand shows up in that frame's results.
        # 'n'/'f' calibration checks this so calibrating one hand can never
        # silently re-use a stale reading from a hand that isn't currently
        # in view.
        self.visible_this_frame = False

    def set_target(self, value):
        with self.lock:
            if not self.frozen:
                self.target = max(MIN_DIST, min(MAX_DIST, value))

    def toggle_freeze(self):
        with self.lock:
            self.frozen = not self.frozen
            return self.frozen

    def get_target(self):
        with self.lock:
            return self.target

    def update_encoder(self, value):
        with self.lock:
            self.encoder_value = value
            self.encoder_ts = time.time()

    def snapshot_encoder(self):
        with self.lock:
            return self.encoder_value, self.encoder_ts


def control_loop(databus: DataBus, state: HandTeleopState, hz: float):
    interval = 1.0 / hz
    while state.running:
        databus.set_target_distance(state.get_target())
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


def _update_ratio_from_landmarks(frame, pts, state: HandTeleopState):
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


def _apply_calibration(state: HandTeleopState):
    if (
        state.calib_near is not None
        and state.calib_far is not None
        and state.smoothed_ratio is not None
    ):
        near, far = state.calib_near, state.calib_far
        if abs(far - near) > 1e-6:
            frac = (state.smoothed_ratio - near) / (far - near)
            frac = max(0.0, min(1.0, frac))
            state.set_target(MIN_DIST + frac * (MAX_DIST - MIN_DIST))


def _calib_status(state: HandTeleopState) -> str:
    # Explicit None-checks, not truthiness — a calibrated ratio of exactly
    # 0.0 (unlikely but not impossible) would otherwise read as uncalibrated.
    if state.calib_near is not None and state.calib_far is not None:
        return f"CALIBRATED (near={state.calib_near:.2f}, far={state.calib_far:.2f})"
    return "NOT CALIBRATED (press 'n' then 'f')"


def _encoder_str(state: HandTeleopState) -> str:
    encoder, enc_ts = state.snapshot_encoder()
    enc_age = time.time() - enc_ts if enc_ts else None
    if encoder is None:
        return "waiting..."
    if enc_age is not None and enc_age <= 1.0:
        return f"{encoder:.4f} m"
    return f"{encoder:.4f} m (stale)"


def _handle_key(key: int, states: dict) -> bool:
    """Apply a cv2.waitKey() result to one or more labeled HandTeleopState
    objects (e.g. {"L": state_left, "R": state_right}, or just {"": state}
    for single-gripper mode). Returns False if quit was requested.

    'n'/'f' only touch a state if BOTH its smoothed_ratio is set AND it was
    actually visible in the current frame (visible_this_frame) — this is
    what makes sequential one-hand-at-a-time calibration safe in dual mode:
    calibrating the second hand never silently re-touches the first hand's
    calibration using an old, no-longer-current reading.
    """
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
            if st.visible_this_frame and st.smoothed_ratio is not None:
                st.calib_near = st.smoothed_ratio
                msg.append(f"{label + '=' if label else ''}{st.calib_near:.3f}")
        if msg:
            print(f"\n>> Calibrated CLOSED: {', '.join(msg)}")
        else:
            print("\n>> No visible hand to calibrate — make sure it's in frame.")
    elif ch == "f":
        msg = []
        for label, st in states.items():
            if st.visible_this_frame and st.smoothed_ratio is not None:
                st.calib_far = st.smoothed_ratio
                msg.append(f"{label + '=' if label else ''}{st.calib_far:.3f}")
        if msg:
            print(f"\n>> Calibrated OPEN: {', '.join(msg)}")
        else:
            print("\n>> No visible hand to calibrate — make sure it's in frame.")
    elif ch == " ":
        frozen = None
        for st in states.values():
            frozen = st.toggle_freeze()
        print("\n>> FROZEN" if frozen else "\n>> LIVE")
    return True


def run_single(args):
    """Single-gripper hand-pinch teleop, native-Linux flavor: same tracking
    and control logic as the WSL version's single-gripper path, but shown
    in a real OpenCV window with keys read via cv2.waitKey()."""
    port = args.port or SIDE_PORTS[args.side]
    state = HandTeleopState(smoothing=args.smoothing)

    def encoder_cb(data):
        try:
            val = struct.unpack(">f", data)[0]
            state.update_encoder(val)
        except Exception:
            pass

    databus = DataBus(
        tty_port=port,
        baudrate=921600,
        encoder_freq=30,
        encoder_callback=encoder_cb,
        gripper_type=args.gripper_type,
    )

    threading.Thread(
        target=control_loop, args=(databus, state, args.hz), daemon=True
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
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print("=" * 60)
    print(f"Window '{WINDOW_NAME}' is open — click it to focus, then use:")
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

            state.visible_this_frame = False

            if result.hand_landmarks:
                lm = result.hand_landmarks[0]
                h, w, _ = frame.shape
                pts = [(p.x * w, p.y * h) for p in lm]
                state.visible_this_frame = True
                _draw_hand(frame, pts)
                _update_ratio_from_landmarks(frame, pts, state)

            _apply_calibration(state)

            target = state.get_target()
            overlay = [
                f"Target: {target:.4f} m   Encoder: {_encoder_str(state)}",
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
        try:
            databus.disable_motor()
            time.sleep(0.1)
        except Exception:
            pass
        databus.stop()
        print("Shutdown complete.")


def run_dual(args):
    """Dual-gripper hand-pinch teleop, native-Linux flavor: left hand ->
    left gripper, right hand -> right gripper, shown in one OpenCV window.

    Calibration ('n'/'f') can be done with both hands in frame at once, or
    one hand at a time (see module docstring) — visible_this_frame on each
    HandTeleopState is what makes the one-at-a-time flow safe."""
    left_port = args.left_port or SIDE_PORTS["left"]
    right_port = args.right_port or SIDE_PORTS["right"]

    state_left = HandTeleopState(smoothing=args.smoothing)
    state_right = HandTeleopState(smoothing=args.smoothing)

    def make_encoder_cb(state):
        def _cb(data):
            try:
                val = struct.unpack(">f", data)[0]
                state.update_encoder(val)
            except Exception:
                pass
        return _cb

    databus_left = DataBus(
        tty_port=left_port,
        baudrate=921600,
        encoder_freq=30,
        encoder_callback=make_encoder_cb(state_left),
        gripper_type=args.gripper_type,
    )
    databus_right = DataBus(
        tty_port=right_port,
        baudrate=921600,
        encoder_freq=30,
        encoder_callback=make_encoder_cb(state_right),
        gripper_type=args.gripper_type,
    )

    threading.Thread(
        target=control_loop, args=(databus_left, state_left, args.hz), daemon=True
    ).start()
    threading.Thread(
        target=control_loop, args=(databus_right, state_right, args.hz), daemon=True
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
    print(f"  Left gripper  <- your LEFT hand  (port {left_port})")
    print(f"  Right gripper <- your RIGHT hand (port {right_port})")
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
            state_left.visible_this_frame = False
            state_right.visible_this_frame = False

            if result.hand_landmarks:
                for idx, lm in enumerate(result.hand_landmarks):
                    # See module docstring for why mirrored (selfie-view)
                    # handedness maps directly to the user's own left/right
                    # hand here.
                    label = "Left"
                    if result.handedness and idx < len(result.handedness):
                        label = result.handedness[idx][0].category_name
                    if args.invert_hands:
                        label = "Right" if label == "Left" else "Left"

                    state = state_left if label == "Left" else state_right
                    seen[label] = True
                    state.visible_this_frame = True

                    pts = [(p.x * w, p.y * h) for p in lm]
                    _draw_hand(frame, pts)
                    _update_ratio_from_landmarks(frame, pts, state)

            _apply_calibration(state_left)
            _apply_calibration(state_right)

            overlay = [
                f"L target: {state_left.get_target():.4f} m   L encoder: {_encoder_str(state_left)}"
                f"{'  [tracking]' if seen['Left'] else ''}",
                f"L calib: {_calib_status(state_left)}",
                f"R target: {state_right.get_target():.4f} m   R encoder: {_encoder_str(state_right)}"
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
        for databus in (databus_left, databus_right):
            try:
                databus.disable_motor()
                time.sleep(0.1)
            except Exception:
                pass
            databus.stop()
        print("Shutdown complete.")


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Webcam hand-pinch teleop (native Linux version)")
    parser.add_argument("side", choices=["left", "right", "dual"],
                         help="Which gripper to drive. 'dual' teleops left+right "
                              "grippers at once with your left/right hands.")
    parser.add_argument("--port", type=str, default=None,
                         help="Serial port override for single-gripper mode (left/right).")
    parser.add_argument("--left-port", type=str, default=None,
                         help="Serial port override for the left gripper in 'dual' mode.")
    parser.add_argument("--right-port", type=str, default=None,
                         help="Serial port override for the right gripper in 'dual' mode.")
    parser.add_argument("--gripper-type", type=str, default="default_gripper")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--smoothing", type=float, default=0.4)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--invert-hands", action="store_true",
                         help="'dual' mode only: swap which detected hand "
                              "(Left/Right) drives which gripper.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    if args.side == "dual":
        run_dual(args)
    else:
        run_single(args)


if __name__ == "__main__":
    main()