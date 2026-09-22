# GenRobot Gripper Teleoperation Tools

A collection of Python tools for teleoperating a GenRobot gripper through the **GenRobot Gripper Controller Python SDK**.

This repository includes:

- Keyboard-based gripper control for a single gripper controller
- Hand-tracking-based gripper control for one or two gripper controllers at once (single or dual mode)
- A WSL2-specific variant for environments where the SDK runs under WSL2
- An in-progress SDK client

> This repository does not include the GenRobot Gripper Controller Python SDK. Install and configure the SDK separately before using these tools.

## Prerequisites

All tools require:

- Python 3
- GenRobot gripper and controller
- The GenRobot Gripper Controller Python SDK, with its `scripts/` folder accessible from wherever you run these tools (either alongside them or symlinked in)
- SDK connection settings configured for your controller
- A safe, clear workspace for testing gripper motion

Create and activate a local Python virtual environment:

```
python3 -m venv venv
source venv/bin/activate
```

The local `venv/` directory is ignored by Git and should not be committed.

## `teleop_gripper.py` — Hand-Tracking Control (single or dual gripper)

`teleop_gripper.py` uses camera-based hand tracking (MediaPipe) to control one or two grippers at once. It opens an OpenCV window ("DAS Gripper Teleop") showing your mirrored webcam feed with the hand skeleton drawn on it, plus an overlay showing target distance, encoder reading, calibration status, and LIVE/FROZEN state.

**Standalone script — do not run alongside `start_gripper.py` or any other script holding the same serial port(s).**

### Modes

Single gripper:
```
python3 teleop_gripper.py left
python3 teleop_gripper.py right
```

Dual gripper — plug in both gripper controllers and drive them with both hands at once, left hand → left gripper, right hand → right gripper:
```
python3 teleop_gripper.py dual
python3 teleop_gripper.py dual --left-port /dev/ttyUSB0 --right-port /dev/ttyUSB1
```

The `--left-port`/`--right-port` flags are optional — they default to `/dev/ttyDeviceLeft` / `/dev/ttyDeviceRight`. Pass them explicitly whenever those udev symlinks aren't present on your system (common on WSL2), or whenever you want to be certain which physical controller is being driven by which hand rather than relying on USB attach order.

### Calibration

Before a hand can drive a gripper, its pinch (thumb tip to index fingertip) needs to be calibrated against a closed and an open reference point:

- `n` — capture the current pinch as the CLOSED reference
- `f` — capture the current pinch as the OPEN reference

In dual mode, calibration can be done either **simultaneously** (both hands in frame, pinch both closed, press `n`, spread both open, press `f`) or **sequentially, one hand at a time** — pinch with one hand, press `n` with your other hand (which doesn't need to be in frame to type), then repeat for the second hand. `n`/`f` only ever apply to whichever hand(s) are actually visible in the current frame, so calibrating the second hand never overwrites the first hand's calibration with a stale reading.

### Controls (window must be focused)

| Key | Action |
|-----|--------|
| `n` | Capture current pinch as CLOSED reference |
| `f` | Capture current pinch as OPEN reference |
| `SPACE` | Freeze/unfreeze target(s) — ignores hand tracking until pressed again |
| `q` / `ESC` | Disable motor(s) and quit |

### Options

| Flag | Default | Description |
|------|---------|--------------|
| `--port` | `/dev/ttyDeviceLeft` or `Right` | Serial port override, single-gripper mode only |
| `--left-port` | `/dev/ttyDeviceLeft` | Serial port override for the left gripper, dual mode only |
| `--right-port` | `/dev/ttyDeviceRight` | Serial port override for the right gripper, dual mode only |
| `--webcam-index` | `0` | OpenCV camera index |
| `--smoothing` | `0.4` | Pinch-ratio smoothing factor (higher = smoother, slower to respond) |
| `--hz` | `30.0` | Control loop rate |
| `--invert-hands` | off | Dual mode only — swap which detected hand (Left/Right) drives which gripper, useful if MediaPipe's handedness classification doesn't match reality for your camera setup |

### Dependencies

- Python 3
- GenRobot Gripper Controller Python SDK
- A webcam or supported camera
- `opencv-python`
- `mediapipe`
- `numpy`
- `pyserial`
- `hand_landmarker.task` (included in this repo — MediaPipe's hand landmark model)

```
pip install opencv-python mediapipe numpy pyserial
```

Before running, ensure the camera is available, lighting is sufficient for hand tracking, and the gripper workspace is clear.

## `teleop_gripper_keys.py` — Keyboard Control

`teleop_gripper_keys.py` controls the gripper through keyboard input — direct, predictable manual control, useful for verifying the SDK can connect to the controller, testing basic gripper movement, or demonstrating control without a camera.

```
python teleop_gripper_keys.py
```

Keep the terminal/window focused while using keyboard controls. Refer to the script for current key mappings.

## `teleop_gripper_WSL.py` — WSL2 Variant

A variant of the teleop tooling for environments where the SDK is running inside WSL2. Refer to the script directly for current usage and any WSL2-specific setup it expects (e.g. `usbipd` device attachment for the serial port and/or camera).

## `teleop_client.py` — SDK Client (Work in Progress)

`teleop_client.py` is an in-progress client intended to provide deeper integration with the GenRobot Gripper Controller Python SDK — managing SDK/controller connections, reusable gripper-control commands, and higher-level control logic. Interface and supported features may change.

## Setup

1. Clone this repository.

```
git clone https://github.com/anthonyshen-GR/teleop-tools.git
cd teleop-tools
```

2. Create and activate a virtual environment.

```
python3 -m venv venv
source venv/bin/activate
```

3. Install the GenRobot Gripper Controller Python SDK and the dependencies required by the tool(s) you want to run.

4. Make sure this repo's Python files can import the SDK's `scripts/` module (place `scripts/` alongside these files, or symlink it in).

5. Configure the SDK connection for your GenRobot controller.

6. Start with keyboard teleoperation to verify the controller connection.

```
python teleop_gripper_keys.py
```

7. Use hand-tracking teleoperation after confirming basic gripper control — single gripper first, then dual once you're comfortable with the calibration flow.

```
python teleop_gripper.py left
python teleop_gripper.py dual
```
