# AGENTS.md

Instructions for future Codex runs in this repository.

This file exists to prevent architectural drift, unsafe hardware edits, dependency breakage, and broad uncontrolled rewrites.

---

## 1. Required Reading Order

Before making changes, read:

1. `docs/MASTER_CONTEXT.md`
2. `README.md`
3. `docs/CURRENT_SCOPE.md`, if present
4. `docs/ENVIRONMENT.md`, if the task touches Python, dependencies, setup, runtime, or the virtual environment
5. The files directly related to the requested task

Treat `docs/MASTER_CONTEXT.md` as the main project memory.

Do not modify docs/MASTER_CONTEXT.md unless the task explicitly asks for documentation updates.

Treat Current Active Scope as higher priority than Future Architecture.

If old reports, specifications, or documents mention ROS, MoveIt, CNN-based piece classification, or broader future plans, do not treat that as current implementation scope. The current repository direction is Python-only on Jetson Nano unless the user explicitly changes it.

---

## 2. Current Project Direction

This is a Python-only Jetson Nano chess-playing robot project.

The current system direction is:

- fixed overhead camera
- calibrated board model
- robot-black board orientation
- occupancy-first vision
- symbolic chess state with `python-chess`
- Stockfish for move selection
- safe calibrated servo primitives
- dry-run-first hardware tools
- manual confirmation when perception is uncertain
- saved debug images and logs

Do not redesign the project into a ROS, MoveIt, web-dashboard, CNN-first, or distributed robotics framework.

---

## 3. Task Scope Rules

Keep every task narrow.

Do only what the user requested.

Do not touch unrelated modules.

Do not perform broad refactors unless explicitly requested.

Do not implement future architecture early.

Do not silently clean up working code outside the active task.

Prefer small, testable files over large abstractions.

A simple working subsystem is better than a sophisticated incomplete framework.

---

## 4. Jetson Nano Environment Rules

The Jetson Nano runtime uses Python 3.6 under JetPack/Ubuntu.

The repository virtual environment is intentionally created with:

```bash
python3 -m venv --system-site-packages .venv
```

This is correct.

Reason:

- Jetson system OpenCV must be reused
- `cv2` comes from Jetson system packages
- pip-installing OpenCV on the Nano is fragile and should be avoided

Use this Nano runtime path:

```bash
cd /data/chess_robot
source .venv/bin/activate
python -m pip install -r requirements-nano.txt
python tools/verify_environment.py
```

Do not use `requirements.txt` as the Nano runtime install path unless explicitly instructed.

Do not run:

```bash
pip install opencv-python
```

Do not run:

```bash
pip freeze > requirements.txt
```

Do not upgrade the Nano to a modern Python stack.

Do not replace or recreate `.venv` unless it is missing or broken and the user explicitly asks.

Known verified Nano runtime versions:

- Python 3.6.9
- cv2 4.1.1
- numpy 1.13.3
- yaml 3.12
- chess 0.31.2
- matplotlib 2.1.1
- serial 3.4

`python-chess==0.31.2` is intentionally pinned for Python 3.6 compatibility.

The import name is still:

```python
import chess
```

If dependency changes are made, run:

```bash
python tools/verify_environment.py
```

---

## 5. Setup Script Rule

If environment setup needs to be reproduced, use the repository setup script if present:

```bash
./scripts/setup_nano_env.sh
```

This script should create `.venv` only if missing, install `requirements-nano.txt`, and run `tools/verify_environment.py`.

Do not replace this script with a modern Python workflow.

---

## 6. Hardware Safety Rules

Hardware tools must be dry-run by default.

Real movement must require explicit user intent.

Do not bypass:

```text
chess_robot/robot/safety.py
```

No raw serial writes outside:

```text
chess_robot/robot/servo_bus.py
```

Real robot movement must require safeguards such as:

- explicit `--real` flag
- typed confirmation where implemented
- known servo ID
- known joint name
- readable current position
- configured limits
- target inside limits
- small movement delta for calibration tools
- logging

Do not invent servo IDs, joint limits, offsets, home poses, gripper values, camera indices, or board geometry.

If a value depends on physical hardware, make it configurable or ask the user to measure it.

Do not run real hardware movement commands unless the user explicitly asks for them.

---

## 7. Board Orientation Rules

Preserve robot-black board orientation.

The robot plays as black.

The calibrated image grid uses:

```text
row = 0 at top
row = 7 at bottom
col = 0 at left
col = 7 at right
```

Required square mapping:

```text
top-left     = h1
top-right    = a1
bottom-left  = h8
bottom-right = a8
```

Implementation rule:

```python
def grid_to_square(row: int, col: int) -> str:
    files = "hgfedcba"
    return f"{files[col]}{row + 1}"
```

Do not correct this to standard white-side chessboard orientation.

Board orientation belongs in calibration or board-mapping code, not as a later patch in chess logic.

---

## 8. Vision Rules

Current vision priority is occupancy-first.

Allowed current vision work:

- overhead camera capture
- camera calibration loading if available
- undistortion if calibration exists
- board calibration
- black-side square mapping
- square polygons
- central square crops
- occupancy detection
- uncertain state reporting
- debug overlays
- saved PNG outputs

Do not implement early:

- CNN piece classification
- full piece recognition
- wrist-camera fallback
- real-time visual servoing
- live video streaming GUI
- automatic gameplay loop unless explicitly requested

The vision module should produce board-state evidence.

It should not directly decide chess legality.

It must not send servo commands.

---

## 9. Chess Logic Rules

Use `python-chess`.

Chess logic may:

- maintain board state
- validate legal moves
- expose FEN
- match changed squares to legal moves
- represent ambiguity
- call Stockfish through a wrapper

Chess logic must not:

- send raw servo commands
- access the servo bus
- assume vision is always correct
- silently accept illegal moves

Ambiguity is allowed.

Manual confirmation is part of the intended system design, not a failure.

---

## 10. Servo and Robot Rules

Servo calibration is separate from chess gameplay.

Servo tasks may include:

- serial port discovery
- servo scanning
- reading positions
- mapping servo IDs to joint names
- dry-run validation
- safe single-joint micro-motion
- joint direction recording
- safe limits
- home pose
- gripper profile
- logging

Servo tasks must not accidentally implement:

- board manipulation
- square targeting
- full pick-and-place
- inverse kinematics
- autonomous gameplay

Motion primitives should call safety-checked robot modules.

No high-level module should write directly to serial.

---

## 11. Module Boundary Rules

Allowed dependency direction:

```text
app -> vision
app -> chess_logic
app -> planning
app -> robot
app -> gui

planning -> chess_logic
planning -> calibration
planning -> robot.motion_primitives

vision -> calibration
vision -> gui renderer outputs

robot -> calibration
robot -> safety
```

Forbidden dependencies:

```text
vision -> raw servo commands
chess_logic -> raw servo commands
gui -> raw servo commands
planning -> raw serial write
app -> raw serial write
```

Only this file should directly talk to the servo bus:

```text
chess_robot/robot/servo_bus.py
```

---

## 12. Data, Logs, and Generated Files

Do not commit runtime junk.

Expected ignored outputs include:

- `.venv/`
- `data/logs/`
- `data/snapshots/`
- `data/debug/`
- `data/gui/`
- temporary arrays
- model weights
- generated images

Calibration files may be important and should not be blindly ignored.

Do not globally ignore `.npz`, because camera calibration files may use `.npz`.

Before changing `.gitignore`, ensure these are not accidentally ignored:

```bash
git check-ignore -v data/calibration/cameras/overhead_calibration.npz || true
git check-ignore -v data/calibration/cameras/overhead_calibration.json || true
git check-ignore -v data/calibration/board/board_profile.yaml || true
```

---

## 13. Git Rules

The Nano repo is connected to GitHub.

Before changes:

```bash
git status
```

After changes:

```bash
git status
git diff
```

Report changed files clearly.

Do not force-push unless the user explicitly instructs it.

Do not run destructive Git commands casually.

Do not merge unrelated GitHub history into the Nano repo unless explicitly requested.

---

## 14. Testing Rules

For Python-only code changes, run the narrowest relevant checks.

Common checks:

```bash
python -m py_compile path/to/file.py
python tools/verify_environment.py
pytest
```

If full `pytest` is too slow or unavailable on the Nano, run targeted syntax/import checks.

For hardware-adjacent code, test dry-run paths first.

For camera code, save debug outputs rather than assuming visual correctness.

For dependency or environment changes, always run:

```bash
python tools/verify_environment.py
```

---

## 15. Tooling Rules

Command-line tools should include clear `--help`.

Hardware tools should clearly indicate whether they are running in dry-run or real mode.

Scripts should fail loudly on missing required files or unsafe assumptions.

Prefer explicit paths under:

```text
configs/
data/calibration/
data/logs/
data/debug/
data/snapshots/
data/gui/
```

Do not hard-code machine-specific paths except the repository root assumptions used in scripts.

---

## 16. Reporting Format After Each Codex Task

After each task, report:

1. Files changed
2. What was implemented
3. Commands run
4. Test results
5. Whether hardware was accessed
6. Remaining warnings or manual steps

For hardware tasks, also report:

- whether dry-run or real mode was used
- servo IDs touched, if any
- safety checks applied
- log file path

For vision tasks, also report:

- input image/camera used
- output image paths
- calibration files used
- known uncertainty

---

## 17. Hard Prohibitions

Do not add ROS.

Do not add MoveIt.

Do not add CNN piece classification unless explicitly requested.

Do not add live web dashboard unless explicitly requested.

Do not install `opencv-python` on the Nano.

Do not run `pip freeze > requirements.txt`.

Do not bypass `chess_robot/robot/safety.py`.

Do not send raw serial commands outside `chess_robot/robot/servo_bus.py`.

Do not invent hardware calibration values.

Do not change robot-black board orientation.

Do not commit `.venv/`.

Do not commit logs, snapshots, debug images, or generated runtime output.

Do not implement broad future modules without a narrow task request.

---

## 18. Core Principle

The robot should expose uncertainty instead of hiding it.

If perception, calibration, hardware state, or move inference is uncertain, surface that uncertainty clearly, save evidence, and require confirmation rather than silently guessing.
