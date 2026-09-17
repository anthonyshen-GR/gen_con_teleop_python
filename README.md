# GenRobot Gripper Teleoperation Tools

A collection of Python tools for teleoperating a GenRobot gripper through the **GenRobot Gripper Controller Python SDK**.

This repository includes:

- Keyboard-based gripper control
- Hand-tracking-based gripper control
- An in-progress SDK client

> This repository does not include the GenRobot Gripper Controller Python SDK. Install and configure the SDK separately before using these tools.

## Prerequisites

All tools require:

- Python 3
- GenRobot gripper and controller
- The GenRobot Gripper Controller Python SDK
- SDK connection settings configured for your controller
- A safe, clear workspace for testing gripper motion

Create and activate a local Python virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

The local `venv/` directory is ignored by Git and should not be committed.

## `teleop_gripper_keys.py` — Keyboard Control

`teleop_gripper_keys.py` controls the gripper through keyboard input.

### Use case

Use this tool for direct and predictable manual control. It is a good starting point for:

- Verifying that the GenRobot SDK can connect to the controller
- Testing basic gripper movement
- Demonstrating gripper control without requiring a camera
- Debugging controller communication

### Dependencies

- Python 3
- GenRobot Gripper Controller Python SDK
- Any keyboard-input package imported by the script

### Run

```bash
python teleop_gripper_keys.py
```

Keep the terminal window focused while using keyboard controls. Refer to the script for the current key mappings.

## `teleop_gripper.py` — Hand-Tracking Control

`teleop_gripper.py` uses camera-based hand tracking to control the gripper.

### Use case

Use this tool for gesture-driven teleoperation. It is intended for:

- Human-in-the-loop gripper demonstrations
- Prototyping natural hand-motion control
- Testing hand tracking as an input method for the GenRobot gripper
- Exploring gesture-based control workflows

### Dependencies

- Python 3
- GenRobot Gripper Controller Python SDK
- A webcam or supported camera
- OpenCV (`opencv-python`)
- MediaPipe (`mediapipe`)
- Any additional packages imported by the script

Install the common hand-tracking dependencies:

```bash
pip install opencv-python mediapipe
```

### Run

```bash
python teleop_gripper.py
```

Before running the tool, ensure that the camera is available, lighting is sufficient for hand tracking, and the gripper workspace is clear.

## `teleop_client.py` — SDK Client Work in Progress

`teleop_client.py` is an in-progress client intended to provide deeper integration with the GenRobot Gripper Controller Python SDK.

### Use case

This file is intended as a foundation for a more complete controller client, including:

- Managing SDK and controller connections
- Providing reusable gripper-control commands
- Supporting future teleoperation workflows
- Building higher-level control logic around the GenRobot SDK

### Dependencies

- Python 3
- GenRobot Gripper Controller Python SDK
- Any additional packages imported as development continues

### Status

This client is actively in development. Its interface and supported features may change.

## Setup

1. Clone this repository.

   ```bash
   git clone https://github.com/anthonyshen-GR/teleop-tools.git
   cd teleop-tools
   ```

2. Create and activate a virtual environment.

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install the GenRobot Gripper Controller Python SDK and the dependencies required by the tool you want to run.

4. Configure the SDK connection for your GenRobot controller.

5. Start with keyboard teleoperation to verify the controller connection.

   ```bash
   python teleop_gripper_keys.py
   ```

6. Use hand-tracking teleoperation after confirming basic gripper control.

   ```bash
   python teleop_gripper.py
   ```
