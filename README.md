# SO-101 Robot Arm Kinematics and Calibration

Developed Python tools for an SO-101 arm on NVIDIA Jetson Nano, using chess-piece manipulation as the test task.

## Engineering Work

- **Kinematics:** URDF-based forward kinematics and constrained numerical [inverse kinematics](chess_robot/robot/ik.py).
- **Actuator integration:** Servo commissioning, calibration, [position readback](chess_robot/robot/servo_bus.py) and joint-limit checks.
- **Perception:** Camera/board calibration and [occupancy detection](chess_robot/vision/occupancy.py) with explicit uncertainty handling.
- **Motion:** Pick-and-place tooling, [sampled board-clearance checks](chess_robot/robot/motion_safety.py) and abort-aware recovery.
- **Planning:** [Chess-move action planning](chess_robot/planning/task_planner.py) and a Stockfish interface, separate from physical execution.

**Outcome:** Reliable piece pickup and playable chess were not achieved. Hardware trials showed arm shaking and repeated missed grasps. The project demonstrates subsystem development and debugging; successful numerical IK does not establish successful physical manipulation.

**Stack:** Python, NumPy, OpenCV, Jetson Nano, Feetech servos, LeRobot calibration tools, python-chess.

## Setup

On the original Jetson Nano environment:

```bash
cd /data/chess_robot
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements-nano.txt
python3 tools/verify_environment.py
python3 -m pytest
```

Hardware tools default to dry-run; real motion requires explicit confirmation and calibrated limits. Sampled clearance checks do not guarantee full collision avoidance.

See [environment requirements](docs/ENVIRONMENT.md), [physical validation scope](docs/CURRENT_SCOPE.md) and [tests](tests/) for technical details.
