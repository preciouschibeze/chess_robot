# Chess Robot

Python-only software for a Jetson Nano chess-playing robot arm.

The system uses a fixed overhead camera, calibrated board geometry, occupancy detection, chess-state tracking, Stockfish move selection, and safety-checked servo motion primitives to move chess pieces on a physical board.

The robot plays as **black**.

---

## Current Scope

The current development focus is **physical pick-and-place validation**.

The immediate goal is not full autonomous gameplay. The immediate goal is:

```text
one piece
one source square
one destination square
safe robot motion
working gripper action
overhead-camera verification
full logs
```

Current active work:

- validate calibrated square targets
- test safe above-square motion
- add and verify pick/place poses
- execute one non-visual pick-and-place sequence
- measure backlash and placement error
- verify the board state with the overhead camera after movement
- use the results to decide where wrist-camera visual refinement is needed

Wrist-camera visual servoing is planned later, after the blind pick-and-place baseline is measured.

---

## System Overview

```text
Overhead camera
  -> board calibration
  -> occupancy detection
  -> changed-square detection
  -> legal move matching
  -> python-chess board state
  -> Stockfish robot move
  -> task planner
  -> motion primitives
  -> servo + gripper control
  -> overhead verification
```

Core design choices:

- Python only
- Jetson Nano runtime
- no ROS
- no MoveIt
- fixed board
- fixed overhead camera
- robot always plays black
- occupancy-first vision
- manual confirmation when vision is ambiguous
- dry-run default for hardware tools

---

## Board Orientation

The board is mapped from the robot-black side:

```text
top-left     = h1
top-right    = a1
bottom-left  = h8
bottom-right = a8
```

Grid mapping:

```python
def grid_to_square(row: int, col: int) -> str:
    files = "hgfedcba"
    return f"{files[col]}{row + 1}"
```

---

## Repository Layout

```text
chess_robot/
├── app/            # application entry points
├── calibration/    # calibration profile loaders and square maps
├── chess_logic/    # board state, legal move matching, Stockfish interface
├── gui/            # terminal and debug display helpers
├── planning/       # chess move to physical action planning
├── robot/          # servo bus, safety, arm control, gripper, primitives
└── vision/         # camera, board calibration, occupancy, transitions

configs/            # runtime configuration
data/               # calibration files, snapshots, debug outputs, logs
docs/               # project documentation and current scope
tests/              # hardware-free tests
tools/              # command-line tools for calibration and validation
```

---

## Setup

On the Jetson Nano:

```bash
cd /data/chess_robot
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python3 -m pip install -r requirements-nano.txt
```

Verify the environment:

```bash
python3 tools/verify_environment.py
```

Run tests:

```bash
python3 -m pytest
```

---

## Common Commands

Camera test:

```bash
python3 tools/test_camera.py
```

Board calibration:

```bash
python3 tools/calibrate_board.py
```

Occupancy detection:

```bash
python3 tools/detect_occupancy.py
```

Changed-square detection:

```bash
python3 tools/detect_changed_squares.py
```

Servo scan:

```bash
python3 tools/scan_servos.py
```

Read servo positions:

```bash
python3 tools/read_servo_positions.py
```

Gripper test:

```bash
python3 tools/test_gripper_monitored_motion.py
```

Square target audit:

```bash
python3 tools/audit_square_targets.py
```

Test above-square motion:

```bash
python3 tools/test_square_above_motion.py
```

Plan a robot move:

```bash
python3 tools/plan_robot_move.py --move e7e5
```

Resolve motion primitives:

```bash
python3 tools/resolve_move_plan.py
```

---

## Safety Rules

- Hardware tools must default to dry-run.
- Real motion must require explicit confirmation.
- Servo IDs must be mapped before movement.
- Joint limits must be loaded before movement.
- All target poses must be validated before execution.
- No module should bypass `chess_robot/robot/safety.py`.
- Only `chess_robot/robot/servo_bus.py` should communicate directly with the servo bus.
- Do not run full-game execution until pick-and-place works reliably.

---

## Minimum Working Demonstration

A successful minimum demonstration is:

```text
1. Capture the board with the overhead camera.
2. Detect the current board occupancy.
3. Move one chess piece from a source square to a destination square.
4. Release the piece cleanly.
5. Capture the board again.
6. Verify the expected occupancy change.
7. Save logs and debug outputs.
```

This is the current practical milestone.
