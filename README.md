# Chess Robot

This repository is the initial lightweight skeleton for a low-cost, perception-guided chess-playing robot arm system running on a Jetson Nano.

The eventual system should observe a fixed chessboard with an overhead camera, detect human moves using occupancy-first vision, maintain symbolic chess state using chess legality constraints, select robot moves with Stockfish, execute calibrated and safety-checked motion primitives, and log results for debugging and evaluation.

## Current Status

This repository is skeleton-only. It does not yet implement board detection, occupancy detection, servo control, Stockfish integration, gameplay, or real robot motion.

The current phase prioritizes narrow vertical slices of working functionality over broad framework completion.

## Runtime Model

- Runtime target: Jetson Nano
- Laptop role: remote development, VS Code/Codex interface, SSH terminal, diff review, log review, and viewing saved debug images
- Language: Python only
- ROS: not used
- MoveIt: not used
- Target Python: Python 3.6 on Jetson Nano / JetPack 4.x

The Nano should eventually own camera access, the servo bus adapter, gripper control, Stockfish runtime, chess logic runtime, calibration tools, robot execution tools, logs, and debug outputs.

## Active Scope

1. Repository skeleton
2. Basic project documentation
3. Overhead camera capture later
4. Board calibration later
5. Occupancy detection later
6. Occupancy-grid rendering later
7. Servo discovery later
8. Servo ID-to-joint mapping later
9. Safe dry-run servo commands later
10. Safe single-joint micro-motion later

Future modules are placeholders until explicitly activated.

## Architecture Overview

- `chess_robot/vision`: future camera capture, board calibration, occupancy detection, and rendered debug outputs
- `chess_robot/calibration`: future measured camera, board, servo, gripper, and square-map profiles
- `chess_robot/chess_logic`: future symbolic board state, legal move matching, and Stockfish boundary
- `chess_robot/planning`: future conversion from chess moves into robot task plans
- `chess_robot/robot`: future safety checks, servo bus access, arm control, motion primitives, and gripper control
- `chess_robot/gui`: future interpreted board-state and saved-output views, plus manual ambiguity confirmation
- `tools`: small command-line entry points for future vertical-slice tests
- `configs`: safe placeholder configuration files with no invented hardware values
- `data`: calibration, snapshot, debug, GUI, and log output directories

Robot-black board orientation must be preserved: top-left `h1`, top-right `a1`, bottom-left `h8`, bottom-right `a8`.

## Environment Note

The Nano runtime environment is documented in `docs/ENVIRONMENT.md`. Use `requirements-nano.txt` inside the `.venv` created with `--system-site-packages`.

Do not install `opencv-python` with pip on the Nano. The runtime should use Jetson system OpenCV through the virtual environment.

## Recommended First Milestones

1. Camera test
2. Board calibration
3. Occupancy detection
4. Servo scanning
5. Dry-run motion primitives
