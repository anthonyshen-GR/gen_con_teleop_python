#!/usr/bin/env python3
"""
teleop_client.py — Browser-previewed hand-pinch teleop CLIENT.
 
Run the driver first, in its own terminal:
    python3 driver.py left
 
Then, in a second terminal, run this client:
    python3 teleop_client.py left
 
This process never opens the serial port. It only computes a target
distance from webcam hand-pinch tracking and sends it over UDP to the
driver process, which owns the gripper hardware, camera streams, and
encoder printing.
"""
 
import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import os
import select
import socket
import struct
import sys
import termios
import threading
import time
import tty
 
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
DEFAULT_DRIVER_PORT = 5566
 
latest_jpeg = None
jpeg_lock = threading.Lock()
 
 
class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """
            <html>
            <head><title>Gripper Teleop Stream</title></head>
            <body style="background:#111; color:#eee; font-family:sans-serif; text-align:center;">
                <h2>DAS Gripper Controller — Hand Teleop (client)</h2>
                <p>Controls are in your <b>WSL Terminal</b>: <code>n</code> = close calib, <code>f</code> = open calib, <code>space</code> = freeze, <code>q</code> = quit</p>
                <p>Encoder readings and camera streams are printed / shown in the <b>driver</b> terminal.</p>
                <img src="/stream.mjpg" style="max-width:90%; border:2px solid #555;" />
            </body>
            </html>
            """
            self.wfile.write(html.encode("utf-8"))
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            while True:
                with jpeg_lock:
                    frame_bytes = latest_jpeg
                if frame_bytes is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
                time.sleep(0.033)
        else:
            self.send_error(404)
 
    def log_message(self, format, *args):
        return
 
 
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
    """Sends target-distance commands to the driver process over UDP."""
 
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
    def send_target(self, value: float):
        try:
            self.sock.sendto(struct.pack(">f", value), self.addr)
        except OSError as e:
            print(f"Failed to send target to driver: {e}")
 
 
def control_loop(link: DriverLink, state: TeleopState, hz: float):
    interval = 1.0 / hz
    while state.running:
        link.send_target(state.get_target())
        time.sleep(interval)
 
 
def terminal_input_thread(state: TeleopState):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while state.running:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
                if ch in ("q", "\x1b"):
                    state.running = False
                    print("\n[Quit requested from terminal]")
                    break
                elif ch == "n":
                    if state.smoothed_ratio is not None:
                        state.calib_near = state.smoothed_ratio
                        print(f"\n>> Calibrated CLOSED: {state.calib_near:.3f}")
                elif ch == "f":
                    if state.smoothed_ratio is not None:
                        state.calib_far = state.smoothed_ratio
                        print(f"\n>> Calibrated OPEN: {state.calib_far:.3f}")
                elif ch == " ":
                    is_frozen = state.toggle_freeze()
                    print("\n>> FROZEN" if is_frozen else "\n>> LIVE")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
 
 
def main():
    global latest_jpeg
    parser = argparse.ArgumentParser(
        description="Hand-pinch teleop client (sends commands to a separately running driver)"
    )
    parser.add_argument("side", choices=["left", "right"])
    parser.add_argument("--gripper-type", type=str, default="default_gripper",
                         choices=["default_gripper", "tactile_gripper", "soft_gripper"],
                         help="Must match the driver's --gripper-type so range clamping agrees")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--smoothing", type=float, default=0.4)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--driver-host", type=str, default="127.0.0.1")
    parser.add_argument("--driver-port", type=int, default=DEFAULT_DRIVER_PORT)
    args = parser.parse_args()
 
    max_dist = MAX_DIST_BY_TYPE[args.gripper_type]
    state = TeleopState(smoothing=args.smoothing, max_dist=max_dist)
    link = DriverLink(host=args.driver_host, port=args.driver_port)
 
    threading.Thread(target=control_loop, args=(link, state, args.hz), daemon=True).start()
 
    httpd = HTTPServer(("0.0.0.0", args.http_port), VideoStreamHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
 
    threading.Thread(target=terminal_input_thread, args=(state,), daemon=True).start()
 
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
 
    print("=" * 60)
    print("STREAM READY! Open this URL in Chrome/Edge on Windows:")
    print(f"  👉 http://localhost:{args.http_port}")
    print(f"Sending commands to driver at {args.driver_host}:{args.driver_port}")
    print("(Encoder readings and camera previews are in the DRIVER terminal)")
    print("=" * 60)
    print("CONTROLS (Keep cursor active in THIS terminal):")
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
                        state.smoothed_ratio = (
                            a * state.smoothed_ratio + (1 - a) * ratio
                        )
 
                cv2.line(
                    frame,
                    tuple(thumb.astype(int)),
                    tuple(index.astype(int)),
                    (0, 255, 0),
                    2,
                )
                cv2.circle(frame, tuple(thumb.astype(int)), 6, (255, 0, 0), -1)
                cv2.circle(frame, tuple(index.astype(int)), 6, (0, 0, 255), -1)
 
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
 
            target = state.get_target()
            calib_status = (
                f"CALIBRATED (near={state.calib_near:.2f}, far={state.calib_far:.2f})"
                if (state.calib_near and state.calib_far)
                else "NOT CALIBRATED (press 'n' then 'f' in terminal)"
            )
 
            overlay = [
                f"Target: {target:.4f} m  (encoder is in the DRIVER terminal)",
                f"Calib: {calib_status}",
                f"{'FROZEN' if state.frozen else 'LIVE'}  |  Terminal: n=near, f=far, space=freeze, q=quit",
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
 
            _, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            )
            with jpeg_lock:
                latest_jpeg = buf.tobytes()
 
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        time.sleep(0.1)
        cap.release()
        try:
            landmarker.close()
        except Exception:
            pass
        httpd.shutdown()
        print("Shutdown complete.")
 
 
if __name__ == "__main__":
    main()
 
