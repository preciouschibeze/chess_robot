# MASTER_CONTEXT.md

## File Status

This is the master context file for the chess-playing robot arm project.

Use this file when starting a fresh ChatGPT chat or a Codex task for a specific subsystem. Paste or reference this file first, then state the exact subsystem to focus on.

This file is not a strict implementation contract. It is a persistent architectural memory document.

Not every module described here must be implemented immediately. Some sections define the current active scope. Other sections define future structure, module boundaries, safety rules, and design assumptions so the project does not drift over time.

Primary purpose:

- preserve the current project direction
- preserve board orientation assumptions
- preserve calibration strategy
- preserve safety rules
- preserve module boundaries
- guide Codex prompts
- reduce repeated redesign discussions
- reduce context rot across ChatGPT and Codex sessions

Working rule:

```text
Current Active Scope overrides Future Architecture.
```

If a section describes future functionality, do not implement it unless the current task explicitly asks for it.

---

# 1. Project Summary

The project is a Python-only chess-playing robot arm system running on a Jetson Nano.

The robot will play chess against a human opponent using:

- fixed overhead camera vision
- board calibration
- occupancy detection
- human move inference
- manual ambiguity confirmation
- symbolic chess-state tracking
- python-chess legal move constraints
- Stockfish-based move selection
- calibrated robot motion primitives
- servo control
- gripper control
- safety checks
- logging and evaluation
- optional future wrist-camera refinement

The current project framing is:

> A low-cost, perception-guided chess manipulation system using a fixed board, calibrated vision, symbolic chess logic, and safe primitive-based robot motion.

The project is not primarily an AI chess project. It is a robotics integration project.

The hard problems are:

- reliable perception
- board calibration
- physical manipulation
- state consistency
- safe actuation
- failure recovery
- repeatability

---

# 2. Current Active Scope

The current development phase prioritises vertical slices of working functionality rather than broad framework completion.

Immediate active priorities:

1. repository skeleton
2. basic project documentation
3. overhead camera capture
4. camera calibration loading, if calibration data exists
5. board calibration
6. black-side square mapping
7. occupancy detection
8. occupancy-grid rendering
9. servo discovery
10. servo ID-to-joint mapping
11. safe dry-run servo commands
12. safe single-joint micro-motion

All other modules should currently be treated as:

- placeholders
- future architecture references
- subsystem boundaries
- design memory

Do not overengineer abstractions before the first working perception and motion subsystems exist.

A simple working subsystem is more valuable than a sophisticated incomplete framework.

---

# 3. Current Non-Goals

Do not build these early:

- ROS integration
- MoveIt integration
- full dynamic motion planning
- full inverse kinematics framework
- CNN-based chess-piece classification
- full real-time visual servoing
- wrist-camera ambiguity resolution
- web dashboard
- distributed robotics architecture
- autonomous multi-agent Codex workflow
- complex GUI streaming system
- voice interaction
- reinforcement learning
- large neural network inference

These may be considered later only after the core perception-to-action loop works.

---

# 4. Runtime and Development Setup

## 4.1 Runtime target

The Jetson Nano is the robot runtime target.

The Nano should own:

- overhead camera, if physically connected
- wrist camera, if used later
- servo bus adapter
- gripper control
- Stockfish runtime
- python-chess runtime
- board-state logic
- local calibration tools
- robot execution tools
- logs and debug outputs

The laptop is mainly the development and monitoring interface.

The laptop should be used for:

- Codex interface
- VS Code interface
- SSH terminal
- viewing files
- inspecting diffs
- reading logs
- opening saved debug images
- editing convenience
- documentation updates

The current preferred workflow is:

```text
Laptop
  -> Codex / VS Code / terminal
  -> SSH into Jetson Nano
  -> edit and run code in the Nano repo

Jetson Nano
  -> owns the robot runtime
  -> owns connected hardware
  -> runs Python tools and robot scripts
```

The laptop is not currently a separate robotics compute node. It is a development workstation and remote interface.

---

# 5. Engineering Philosophy

Prefer:

- inspectability
- simple interfaces
- deterministic behaviour
- explicit uncertainty handling
- conservative motion
- dry-run defaults
- clear logs
- saved debug images
- narrow Codex tasks
- physical validation after each subsystem milestone

Avoid:

- premature abstraction
- hidden state
- unnecessary autonomy
- complex middleware
- broad Codex prompts
- large rewrites
- uncontrolled hardware commands
- implementing future modules before current modules work

Main principle:

```text
The robot should expose uncertainty instead of hiding it.
```

Example:

```text
I am not sure whether the human moved e2e4 or g1f3.
Please confirm.
```

This is better than silently making the wrong move.

---

# 6. Core Decisions

## D001 — Python-only system

The system will be implemented in Python.

No ROS.
No MoveIt.

Reason:

- the board is fixed
- the task is structured pick-and-place
- calibrated motion primitives are enough for the first working system
- Jetson Nano setup is simpler without ROS and MoveIt
- the timeline favours reliability over middleware complexity

Consequence:

- explicit module boundaries are required
- safety checks must be implemented manually
- calibration data must be stored clearly
- hardware access must be controlled through robot modules only

---

## D002 — Jetson Nano is the primary robot runtime

The robot should run locally on the Nano.

Stockfish, python-chess, camera capture, board detection, servo control, and game orchestration may all run on the Nano.

The laptop is mainly for development, remote inspection, and debugging.

Do not redesign the system into a laptop-hosted robotics architecture unless there is a concrete performance or hardware reason later.

---

## D003 — Robot plays only as black

The robot will always play as black.

This affects:

- board orientation
- square mapping
- GUI labels
- calibration profiles
- move display
- physical square targeting

The board is viewed from the robot-black side.

Agreed board mapping:

```text
top-left     = h1
top-right    = a1
bottom-left  = h8
bottom-right = a8
```

If the calibrated image grid uses:

```text
row = 0 at top
row = 7 at bottom
col = 0 at left
col = 7 at right
```

then:

```python
def grid_to_square(row: int, col: int) -> str:
    files = "hgfedcba"
    return f"{files[col]}{row + 1}"
```

Examples:

```text
row 0, col 0 -> h1
row 0, col 7 -> a1
row 7, col 0 -> h8
row 7, col 7 -> a8
```

This mapping must be implemented in the calibration or board-mapping layer, not patched later in chess logic.

---

## D004 — Fixed board

The board is physically fixed relative to the robot base.

Reason:

- simplifies calibration
- allows square-to-robot mapping
- removes the need for dynamic board pose estimation during every move
- supports repeatable manipulation

Consequence:

- board calibration only needs to be redone if the board or camera moves
- robot-square calibration remains valid if the robot and board remain fixed
- the system should not assume the board may freely move during gameplay

---

## D005 — Fixed overhead camera

The overhead camera is the main global vision sensor.

It handles:

- board view
- square localisation
- occupancy detection
- changed-square detection
- post-move verification
- debug overlays
- occupancy-grid rendering

The overhead camera is the main source of global board-state evidence.

---

## D006 — Wrist camera deferred

The wrist camera is not part of the first working system.

Do not use the wrist camera early for ambiguity resolution.

Reason:

- it adds hand-eye calibration complexity
- it adds inspection-pose planning
- it adds close-up classification complexity
- it may slow gameplay
- it is not necessary if legal-move constraints and manual confirmation are available

Potential later use:

- local grasp alignment
- checking whether a piece is centred before picking
- small X/Y correction before descent
- confirming whether a piece was lifted
- placement verification
- optional close-up inspection

---

## D007 — Occupancy-first vision

The first vision goal is not full chess-piece recognition.

The first vision goal is:

```text
empty square vs occupied square
```

Reason:

- the symbolic board state already knows what piece should be on each square
- legal move constraints reduce ambiguity
- full overhead piece classification is harder
- occupancy is enough for many human move detection cases
- manual confirmation can handle uncertainty

Full piece classification is deferred.

---

## D008 — Manual ambiguity confirmation is allowed

Manual confirmation is part of the system design.

It is not a failure.

When the vision system cannot confidently infer a human move, the system should show candidate moves and ask for confirmation.

Example:

```text
Ambiguous move detected.

Changed squares:
- e2
- e4
- g1
- f3

Candidate moves:
1. e2e4
2. g1f3

Enter choice:
```

Manual confirmation is simpler and more reliable than early wrist-camera inspection.

---

## D009 — Servo safety before motion

No real robot motion should occur until:

- servo IDs are known
- current positions can be read
- joint names are mapped to servo IDs
- joint limits are configured
- dry-run mode exists
- command logging exists
- emergency stop behaviour is documented
- movement is conservative
- real movement requires explicit confirmation

Default mode for hardware tools must be dry-run.

---

# 7. Current Physical Board Notes

The chessboard is made from two halves.

The hinges were removed and the board halves were glued or fixed into the robot base.

There is a slight central gap between the two halves.

Important physical notes:

- the board is fixed
- the board is flat enough for the current project
- the central gap is slightly wider on one side and narrower on the other
- the gap is a thin tapered seam, not a constant-width line
- the gap is not expected to move during gameplay

This is acceptable.

The gap should be handled in calibration rather than ignored.

Recommended handling:

- do not depend only on a perfect 4-corner equal-grid calibration as the final method
- use manual 9x9 grid-line calibration or manually corrected grid intersections
- store the central seam as an ignored polygon if needed
- occupancy should use the central region of each square only
- avoid using border and seam pixels for occupancy classification

---

# 8. Board Calibration Strategy

## 8.1 Why 4-corner calibration is not enough as the final method

A simple prototype method is:

```text
click 4 playable-board corners
compute homography
generate equal 8x8 grid
```

This may work for a first test, but it assumes:

- perfectly square board
- uniform spacing
- no seam
- no tapered gap
- no local alignment error

The actual board has a fixed central tapered seam.

Therefore, the stronger method is:

```text
manual 9x9 grid-line calibration
```

---

## 8.2 Recommended board model

Use a calibrated grid model:

```text
9 vertical grid lines
9 horizontal grid lines
64 square polygons
64 square centres
black-side square labels
optional ignored seam polygon
```

Each square should be represented by its actual calibrated polygon rather than assuming all squares are perfectly equal.

Stored board profile should include:

```yaml
board_orientation: robot_black_side

corner_labels:
  top_left: h1
  top_right: a1
  bottom_left: h8
  bottom_right: a8

grid_model: manual_9x9

grid_points:
  rows: 9
  cols: 9
  points: []

squares:
  h1:
    polygon: []
    centre: []
  g1:
    polygon: []
    centre: []

ignored_regions:
  - name: centre_seam
    type: polygon
    points: []

occupancy:
  crop_fraction: 0.60
  ignore_square_borders: true
  ignore_regions:
    - centre_seam
```

---

## 8.3 Occupancy crop rule

Occupancy detection should not use the full square crop.

Use only the central region:

```text
central 50–65% of each square
```

Recommended initial value:

```text
60%
```

Reason:

- avoids grid lines
- avoids printed labels
- avoids seam pixels
- avoids edge shadows
- focuses on where pieces should actually sit

---

## 8.4 Board calibration acceptance criteria

Board calibration is acceptable when:

- all 64 square polygons align visually
- all 64 square centres are visually centred
- square labels match black-side orientation
- the seam is either ignored or outside central occupancy crops
- the overlay remains stable across repeated captures
- the board image can be interpreted consistently

Required debug outputs:

```text
data/debug/board_grid_overlay.png
data/debug/board_labels_overlay.png
data/debug/occupancy_crop_overlay.png
data/calibration/board/board_profile.yaml
```

---

# 9. Initial Repository Skeleton

The initial repository structure should remain lightweight.

The purpose of the initial structure is:

- architectural organisation
- stable paths
- clean subsystem separation
- future scalability
- Codex consistency

It is not intended to fully implement every abstraction immediately.

Placeholder modules and TODO files are acceptable.

Recommended initial structure:

```text
chess_robot/
├── app/
│   ├── __init__.py
│   └── main.py
│
├── vision/
│   ├── __init__.py
│   ├── camera.py
│   ├── board_calibration.py
│   ├── occupancy.py
│   └── board_renderer.py
│
├── calibration/
│   ├── __init__.py
│   ├── camera_profile.py
│   ├── board_profile.py
│   ├── servo_profile.py
│   └── robot_square_map.py
│
├── chess_logic/
│   ├── __init__.py
│   ├── board_state.py
│   ├── legal_move_matcher.py
│   └── stockfish_engine.py
│
├── planning/
│   ├── __init__.py
│   ├── task_planner.py
│   └── move_plan.py
│
├── robot/
│   ├── __init__.py
│   ├── servo_bus.py
│   ├── safety.py
│   ├── arm_controller.py
│   ├── motion_primitives.py
│   └── gripper.py
│
├── gui/
│   ├── __init__.py
│   ├── board_state_view.py
│   ├── occupancy_display.py
│   └── cli_confirm.py
│
└── types.py

configs/
├── app.yaml
├── cameras.yaml
├── robot.yaml
├── vision.yaml
└── gui.yaml

tools/
├── test_camera.py
├── calibrate_board.py
├── detect_occupancy.py
├── scan_servos.py
├── read_servo_positions.py
├── move_servo_small_step.py
├── calibrate_servos.py
├── calibrate_gripper.py
├── test_stockfish.py
└── run_game_dry.py

data/
├── calibration/
│   ├── cameras/
│   ├── board/
│   ├── robot/
│   └── gripper/
├── snapshots/
├── debug/
├── gui/
└── logs/

docs/
├── MASTER_CONTEXT.md
├── CURRENT_SCOPE.md
├── DECISIONS.md
├── TEST_PLAN.md
└── EVALUATION_PLAN.md

tests/
├── test_board_mapping.py
├── test_legal_move_matcher.py
└── test_task_planner.py
```

Implementation rule:

```text
Create the skeleton early. Implement modules narrowly.
```

Do not turn placeholder files into complex frameworks before real subsystem tests exist.

---

# 10. Module Dependency Rules

## 10.1 Allowed dependency direction

Allowed:

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

## 10.2 Forbidden dependencies

Forbidden:

```text
vision -> raw servo commands
chess_logic -> raw servo commands
gui -> raw servo commands
planning -> raw serial write
app -> raw serial write
```

Only this module should talk directly to the servo bus:

```text
chess_robot/robot/servo_bus.py
```

All real motion must pass through safety checks.

---

# 11. Initial Runtime Flow

Do not start with a large state machine.

Start with this simplified runtime loop:

```text
INIT
WAIT_FOR_HUMAN
DETECT_MOVE
RESOLVE_AMBIGUITY_IF_NEEDED
UPDATE_BOARD_STATE
GET_ROBOT_MOVE
PLAN_ROBOT_MOVE
EXECUTE_ROBOT_MOVE
VERIFY
ERROR_OR_RECOVERY
```

A more detailed state machine may be added later if needed:

```text
BOOT
LOAD_CONFIG
LOAD_CALIBRATION
CONNECT_CAMERAS
CONNECT_ROBOT
HOME_ROBOT
READ_INITIAL_BOARD
WAIT_FOR_HUMAN_MOVE
CAPTURE_AFTER_HUMAN_MOVE
DETECT_HUMAN_MOVE
RESOLVE_AMBIGUITY
VALIDATE_HUMAN_MOVE
UPDATE_BOARD_STATE
GET_ENGINE_MOVE
PLAN_ROBOT_MOVE
EXECUTE_ROBOT_MOVE
VERIFY_ROBOT_MOVE
RECOVERY
GAME_OVER
SHUTDOWN
```

Each major state should eventually log:

- timestamp
- state name
- FEN
- occupancy grid
- vision confidence
- candidate moves
- selected move
- user confirmation if any
- robot task plan
- servo position snapshot
- success/failure
- relevant image paths

---

# 12. GUI and Occupancy Monitor Plan

## 12.1 GUI purpose

The GUI is not meant to stream live camera video at the start.

The GUI should show what the robot thinks is happening.

The useful display is:

```text
interpreted board state
+
occupancy grid
+
uncertainty indicators
+
candidate move indicators
```

The GUI should answer:

- What does the robot think the board state is?
- Which move does it think the human made?
- Which squares are uncertain?
- Does the internal chess state match the physical board?

---

## 12.2 First GUI implementation

Start with:

- terminal board display
- numbered ambiguity choices
- saved occupancy-grid PNG
- saved overlay PNG

Example terminal display:

```text
Robot perspective: Black side

    a b c d e f g h
8 | r n b q k b n r
7 | p p p p p p p p
6 | . . . . . . . .
5 | . . . . . . . .
4 | . . . . . . . .
3 | . . . . . . . .
2 | P P P P P P P P
1 | R N B Q K B N R

Detected changed squares: e2, e4
Candidate move:
1. e2e4

Enter choice or press Enter to accept:
```

Recommended config option:

```yaml
gui_perspective: robot_black
```

Allowed values later:

```text
robot_black
standard_white
```

---

## 12.3 Saved PNG outputs

Create renderer outputs:

```text
data/gui/latest_board_state.png
data/gui/latest_occupancy_grid.png
data/gui/latest_ambiguity.png
```

These should be easy to open from VS Code.

The rendered board should show:

- square labels
- internal piece symbols
- occupancy confidence
- uncertain squares
- changed squares
- candidate moves
- source and destination markers

---

# 13. Vision Module Plan

## 13.1 Purpose

The vision module turns camera images into board-state evidence.

It should produce:

- captured frame
- optional undistorted frame
- board grid overlay
- square polygons
- square centres
- occupancy grid
- confidence scores
- changed-square list
- debug images

It should not decide chess legality by itself.

---

## 13.2 Initial vision pipeline

```text
capture frame
-> load camera profile if available
-> undistort frame if calibration exists
-> load board profile
-> map board grid
-> crop central square regions
-> compute occupancy score per square
-> classify occupied / empty / uncertain
-> save occupancy grid
-> save debug overlays
```

Later pipeline:

```text
capture multiple frames
-> temporal filtering
-> compare previous and current occupancy grids
-> output changed squares
-> pass changed-square evidence to chess logic
```

---

## 13.3 Camera profile module

Files:

```text
chess_robot/calibration/camera_profile.py
chess_robot/vision/camera.py
```

Responsibilities:

- open camera device
- capture still frame
- load overhead camera calibration if available
- support pinhole or fisheye calibration later
- expose undistortion function if calibration data exists
- save raw and processed frames

Input examples:

```text
data/calibration/cameras/overhead_calibration.npz
data/calibration/cameras/overhead_calibration.json
```

Output examples:

```text
data/snapshots/latest_raw.png
data/snapshots/latest_undistorted.png
```

---

## 13.4 Board profile module

File:

```text
chess_robot/calibration/board_profile.py
```

Responsibilities:

- load board_profile.yaml
- validate board orientation
- provide square labels
- provide square polygons
- provide square centres
- provide ignored seam polygon
- provide crop region per square

---

## 13.5 Board calibration tool

File:

```text
tools/calibrate_board.py
```

Responsibilities:

- load one overhead image
- allow user to mark board grid
- support black-side orientation
- generate square labels
- save board profile
- render debug overlay

Minimum viable version:

```text
click 4 corners
compute grid
save YAML
render overlay
```

Improved version:

```text
manual 9x9 grid calibration
manual correction of grid intersections
optional ignored seam polygon
central occupancy crop overlay
```

---

## 13.6 Occupancy detector

File:

```text
chess_robot/vision/occupancy.py
```

Responsibilities:

- crop central region of each square
- compare crop against empty-board reference if available
- compute occupancy score
- ignore seam and borders
- return occupied / empty / uncertain
- return confidence score

Initial approach:

- capture empty-board reference
- compare current central crop to empty-board crop
- use colour, intensity, and/or edge difference
- threshold into occupied / empty / uncertain

Example output:

```python
{
    "e4": {
        "state": "occupied",
        "confidence": 0.91,
        "score": 34.2
    },
    "e5": {
        "state": "empty",
        "confidence": 0.87,
        "score": 5.1
    }
}
```

---

## 13.7 Temporal filter

File:

```text
chess_robot/vision/temporal_filter.py
```

This is optional at first.

Purpose:

- reduce frame-level noise
- avoid false detection from hand motion
- require stable occupancy over multiple frames

Example later logic:

```text
capture 5 frames
classify each square per frame
accept state only if at least 3/5 agree
mark uncertain otherwise
```

---

## 13.8 Move inference

File:

```text
chess_robot/vision/move_inference.py
```

This is not an early priority until occupancy works.

Responsibilities:

- compare previous and current occupancy grids
- identify changed squares
- pass changed-square evidence to chess_logic/legal_move_matcher.py

This module should not make final legal move decisions alone.

---

## 13.9 Vision acceptance criteria

Vision module is acceptable when:

- it captures an overhead image
- it loads camera calibration if available
- it loads board profile
- it renders 64 labelled squares correctly
- it uses black-side orientation correctly
- it ignores central seam and borders
- it outputs occupancy grid
- it saves latest occupancy PNG
- it marks uncertainty instead of guessing
- it can compare two board states and return changed squares later

---

# 14. Chess Logic Module Plan

## 14.1 Purpose

The chess logic module maintains symbolic board state and legal move constraints.

Use:

```text
python-chess
```

Responsibilities:

- represent current game board
- generate legal moves
- validate human moves
- maintain FEN
- handle turns
- handle special moves
- interface with Stockfish

---

## 14.2 Board state

File:

```text
chess_robot/chess_logic/board_state.py
```

Responsibilities:

- initialise board
- update board after confirmed move
- expose FEN
- expose legal moves
- expose piece at square
- provide board display data for GUI

---

## 14.3 Legal move matcher

File:

```text
chess_robot/chess_logic/legal_move_matcher.py
```

Input:

- previous occupancy
- current occupancy
- changed squares
- legal moves from python-chess
- vision confidence scores

Output:

- accepted move if unique
- candidate moves if ambiguous
- no-match status if invalid

Logic:

```text
1. Generate legal moves.
2. For each legal move, predict expected changed squares.
3. Compare expected changed squares with detected changed squares.
4. Score each candidate.
5. Accept if one candidate is clearly best.
6. Otherwise return ambiguity.
```

---

## 14.4 Ambiguity handler

Files:

```text
chess_robot/chess_logic/ambiguity.py
chess_robot/gui/cli_confirm.py
```

Responsibilities:

- present candidate moves
- allow user to choose by number
- allow rescan
- allow manual UCI entry
- allow abort
- log manual intervention

Accepted inputs:

```text
1, 2, 3...
r = rescan
m = manual move entry
q = abort
```

---

## 14.5 Stockfish engine

File:

```text
chess_robot/chess_logic/stockfish_engine.py
```

Responsibilities:

- receive current FEN
- call Stockfish
- return UCI move
- keep engine strength modest
- avoid long thinking time on Jetson Nano

Recommended initial settings:

```text
depth: 6–10
or
movetime: 500–2000 ms
```

Stockfish is not expected to be the main system bottleneck.

---

## 14.6 Chess logic acceptance criteria

Chess logic module is acceptable when:

- it can initialise a board
- it can print or render current board state
- it can validate legal moves
- it can reject illegal moves
- it can match changed squares to legal move candidates
- it can represent ambiguity cleanly
- it can ask user for confirmation
- it can update the board after confirmation
- it can return FEN for Stockfish

---

# 15. Servo Calibration Module Plan

## 15.1 Immediate purpose

Servo calibration is the first major hardware-control milestone.

The goal is not pick-and-place.

The goal is to safely identify and configure the robot servo system.

---

## 15.2 Servo calibration scope

Covers:

- serial port discovery
- servo ID scan
- read current positions
- map servo IDs to joint names
- verify torque enable/disable
- dry-run movement validation
- small real single-joint movement
- joint direction recording
- safe software limits
- home pose
- gripper open/close positions
- logging

Does not cover:

- board manipulation
- square target poses
- inverse kinematics
- chess moves
- Stockfish
- automatic gameplay

---

## 15.3 Servo files

Expected files:

```text
configs/robot.yaml
chess_robot/robot/servo_bus.py
chess_robot/robot/safety.py
chess_robot/robot/arm_controller.py
chess_robot/robot/gripper.py

tools/scan_servos.py
tools/read_servo_positions.py
tools/move_servo_small_step.py
tools/calibrate_servos.py
tools/calibrate_gripper.py

data/calibration/robot/servo_offsets.yaml
data/calibration/robot/joint_limits.yaml
data/calibration/robot/home_pose.yaml
data/calibration/gripper/gripper_profile.yaml
data/logs/servo.log
```

---

## 15.4 Servo safety rules

Real movement must require:

- explicit `--real` flag
- typed confirmation
- known joint name
- known servo ID
- known current position
- configured joint limits
- target inside joint limits
- small delta
- log entry

Default mode must be dry-run.

No module should bypass `safety.py`.

---

## 15.5 Joint names

Use these standard joint names unless hardware proves otherwise:

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

---

## 15.6 Servo acceptance criteria

Servo calibration is complete when:

- controller port is known
- all servos are detected
- each ID maps to a joint name
- current positions can be read
- torque can be enabled and disabled safely
- dry-run movement validation works
- small real movement works one joint at a time
- direction is recorded
- joint limits are recorded
- home pose is saved
- gripper profile is saved
- logs are created
- no module bypasses `safety.py`

---

# 16. Robot-Square Calibration Plan

## 16.1 Purpose

Robot-square calibration maps chess squares to physical robot target poses.

This happens after servo calibration.

The robot needs to know how to move safely to each square.

---

## 16.2 Recommended approach

Do not implement perfect IK at the beginning.

Use taught or calibrated joint poses.

Each square should eventually store:

```text
above_pose
pick_pose
place_pose
```

Initial version:

```text
above_pose only
```

Then:

```text
pick_pose/place_pose
```

---

## 16.3 Calibration method

Recommended process:

```text
1. Move robot to home.
2. Move manually or by small commands to a safe above-square pose.
3. Save joint positions.
4. Repeat for key board squares.
5. Interpolate if reliable.
6. Manually correct bad squares.
7. Test every above-square pose without descending.
8. Add pick/place heights only after above-square movement is safe.
```

---

## 16.4 Square target file

```text
data/calibration/robot/square_targets.yaml
```

Example:

```yaml
squares:
  e4:
    above_pose:
      shoulder_pan: 2048
      shoulder_lift: 2100
      elbow_flex: 2200
      wrist_flex: 2050
      wrist_roll: 2048
      gripper: 1700
    pick_pose:
      shoulder_pan: 2048
      shoulder_lift: 2160
      elbow_flex: 2250
      wrist_flex: 2080
      wrist_roll: 2048
      gripper: 1700
```

---

## 16.5 Acceptance criteria

Robot-square calibration is acceptable when:

- the robot can move above tested square centres safely
- no pose hits joint limits
- no pose collides with board or table
- home can be reached from each tested pose
- square-target file is saved
- movements are logged
- failed movement stops safely

Moving above all 64 squares is the eventual goal, not the first day-one requirement.

---

# 17. Motion Primitive Plan

## 17.1 Purpose

Motion primitives convert planned chess actions into physical robot actions.

They are not general-purpose motion planning.

---

## 17.2 Basic primitives

```text
move_home()
move_above_square(square)
descend_to_pick(square)
close_gripper()
lift_from_square(square)
move_above_square(destination)
descend_to_place(destination)
open_gripper()
lift_from_square(destination)
```

---

## 17.3 Pick-and-place sequence

Normal move:

```text
move_home
move_above_source
descend_to_pick
close_gripper
lift
move_above_destination
descend_to_place
open_gripper
lift
move_home
```

Capture:

```text
move_above_destination
descend_to_pick_captured_piece
close_gripper
lift
move_to_capture_zone
release
move_above_source
descend_to_pick_own_piece
close_gripper
lift
move_above_destination
descend_to_place
open_gripper
lift
move_home
```

---

## 17.4 Acceptance criteria

Motion primitives are acceptable when:

- each primitive logs action and result
- dry-run mode can print the full planned sequence
- no primitive sends raw servo commands directly
- each movement passes safety validation
- failed primitive stops the sequence
- single pawn pick-and-place works repeatedly

---

# 18. Task Planner Module Plan

## 18.1 Purpose

The task planner converts symbolic chess moves into physical action sequences.

Input:

```text
UCI move, e.g. e7e5
```

Output:

```text
physical action sequence
```

---

## 18.2 Planner responsibilities

The planner should eventually handle:

- normal moves
- captures
- castling
- promotion
- en passant
- capture-zone placement
- dry-run action preview
- validation before execution

Special moves may be deferred, but the planner should not silently ignore them.

Unsupported special moves should return a clear unsupported status.

---

## 18.3 Planned move object

Example:

```python
from dataclasses import dataclass

@dataclass
class PhysicalAction:
    name: str
    params: dict

@dataclass
class MovePlan:
    chess_move: str
    actions: list[PhysicalAction]
    requires_capture_zone: bool
    notes: list[str]
```

---

## 18.4 Acceptance criteria

Task planner is acceptable when:

- it converts normal legal moves into action lists
- it converts captures into two-stage action lists
- it rejects unsupported special moves clearly
- it can dry-run print the action sequence
- it does not directly control servos
- it integrates with motion primitives only

---

# 19. Logging and Evaluation Plan

## 19.1 Why logging matters

Logs are needed for:

- debugging
- thesis evidence later
- evaluation metrics
- failure analysis
- reproducibility
- engineering rigour

---

## 19.2 Logs to maintain

```text
data/logs/app.log
data/logs/vision.log
data/logs/servo.log
data/logs/game.log
data/logs/calibration.log
data/logs/evaluation.csv
```

---

## 19.3 Evaluation metrics

Final project can evaluate:

- board calibration success rate
- occupancy detection accuracy
- human move detection accuracy
- ambiguity rate
- manual confirmation frequency
- robot move execution success rate
- pick success rate
- placement success rate
- average move time
- recovery frequency
- full-game completion rate
- failure categories

---

## 19.4 Event log format

Each move should eventually log:

```text
move_number
timestamp
human_move_detected
human_move_confirmed
robot_move
vision_confidence
ambiguity_status
manual_confirmation_used
task_plan
execution_success
verification_success
failure_reason
image_paths
```

---

# 20. Codex Workflow

## 20.1 Role split

Use this split:

```text
User:
  project owner
  physical hardware operator
  final decision maker

ChatGPT:
  architecture planning
  module design
  implementation reasoning
  debugging strategy
  documentation support

Codex:
  local repo implementation
  code editing
  file creation
  test running
  traceback debugging
  diff review
```

Codex should not be the master architect.

---

## 20.2 Codex location

Current intended workflow:

```text
Open Codex on laptop.
Use Codex to SSH into the Jetson Nano.
Run commands and edit files in the Nano repo.
```

Reason:

- Codex interface benefits from laptop resources
- Nano remains the actual robot runtime
- hardware stays connected to Nano
- files and tests still happen on the target system

---

## 20.3 Codex task rules

Every Codex task should be narrow.

Good:

```text
Implement only tools/calibrate_board.py and chess_robot/calibration/board_profile.py.
Do not touch servo code.
Do not implement Stockfish.
Add debug overlay output.
Run syntax checks.
```

Bad:

```text
Build the chess robot software.
```

---

## 20.4 Codex prompt template

Use this template:

```text
You are working in the Jetson Nano chess robot repo.

Read docs/MASTER_CONTEXT.md first.

Task:
[exact task]

Active scope:
[what is allowed in this task]

Out of scope:
[what must not be touched]

Constraints:
- Python only.
- No ROS.
- No MoveIt.
- Do not touch unrelated modules.
- Keep files small and testable.
- Add logging where relevant.
- Add clear command-line help for tools.
- Dry-run by default for hardware-related tools.
- Do not invent hardware assumptions.
- Preserve black-side board orientation.
- Preserve safety rules.

Expected output:
- files changed
- how to run the tool/script
- what was tested
- what remains manual
```

---

## 20.5 When to use Codex subagents

Do not use subagents by default.

Use them only for review or audit tasks such as:

- architecture drift review
- servo safety review
- test coverage review
- calibration consistency review
- log failure summarisation

Do not use subagents as autonomous module owners.

---

# 21. Module-Specific ChatGPT Workflow

For each major subsystem:

```text
1. Start a fresh ChatGPT chat.
2. Paste or upload this MASTER_CONTEXT.md file.
3. Tell ChatGPT the exact subsystem to focus on.
4. Ask for a subsystem-specific implementation plan.
5. Convert that plan into a Codex prompt.
6. Run Codex against the Nano repo.
7. Inspect diffs in VS Code.
8. Run scripts/tests.
9. Paste errors back into the same subsystem chat.
10. Commit stable milestone.
```

This keeps context clean.

---

# 22. Suggested Implementation Order

Recommended order:

```text
0. Create clean repository skeleton
1. Add docs/MASTER_CONTEXT.md
2. Add basic configs and package structure
3. Add camera capture test
4. Add board calibration profile structure
5. Add black-side square mapping tests
6. Add simple board calibration tool
7. Add debug overlay rendering
8. Add empty-board reference capture
9. Add occupancy detection
10. Add occupancy-grid PNG renderer
11. Add changed-square comparison
12. Add python-chess board state wrapper
13. Add legal move matcher
14. Add manual ambiguity confirmation
15. Add Stockfish wrapper
16. Add servo scan
17. Add read servo positions
18. Add servo safety validation
19. Add single-joint dry-run
20. Add single-joint micro-motion
21. Add gripper calibration
22. Add robot-square calibration
23. Add dry-run motion primitives
24. Add single-piece pick-and-place
25. Add task planner
26. Add full dry-run game loop
27. Add physical robot move execution
28. Add verification and recovery
29. Add evaluation logging
30. Add optional wrist-camera refinement
```

If hardware is ready before vision, servo calibration may move earlier.

Do not connect vision, chess logic, and robot motion until each subsystem works independently.

---

# 23. Module-Specific Starting Prompts for New ChatGPT Chats

## 23.1 Board detection chat

```text
I am working on the board detection module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on overhead camera input, lens undistortion if calibration exists, black-side board orientation, seam-aware/manual 9x9 grid calibration, square polygons, central occupancy crops, debug overlays, and acceptance tests. Do not discuss servo motion, Stockfish, or robot control except where interfaces are needed.
```

---

## 23.2 Occupancy detection chat

```text
I am working on the occupancy detection module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on central square crops, empty-board reference comparison, occupied/empty/uncertain classification, confidence scoring, seam/border exclusion, temporal filtering if needed, changed-square output, and saved PNG debug outputs. Do not implement full piece classification or robot movement.
```

---

## 23.3 Servo calibration chat

```text
I am working on the servo calibration module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on safe servo ID scanning, reading positions, dry-run validation, joint-name mapping, torque enable/disable, joint direction, safe software limits, home pose, gripper profile, logging, and Codex implementation prompts. Do not discuss vision or chess logic except where interfaces are needed.
```

---

## 23.4 GUI / occupancy monitor chat

```text
I am working on the GUI/occupancy-monitor module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on showing the interpreted 8x8 board state, occupancy grid, uncertain squares, candidate moves, manual ambiguity confirmation, saved PNG outputs, terminal display, and VS Code-friendly debug files. Do not build a live camera streaming UI.
```

---

## 23.5 Chess logic chat

```text
I am working on the chess-logic module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on python-chess board state, legal move generation, changed-square matching, ambiguity handling, FEN, move validation, and Stockfish interface. Do not implement robot movement.
```

---

## 23.6 Robot-square calibration chat

```text
I am working on robot-square calibration for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on mapping chess squares to calibrated robot poses, taught positions, above/pick/place poses, safe movement validation, and calibration file structure. Do not implement full gameplay.
```

---

## 23.7 Motion primitive chat

```text
I am working on the motion primitive module for the Jetson Nano chess robot. Use the pasted MASTER_CONTEXT.md as the system reference. Focus only on safe primitive-based pick-and-place actions, dry-run sequencing, capture handling, gripper calls, safety checks, and logging. Do not implement vision or Stockfish.
```

---

# 24. Immediate Next Step Options

The next step depends on what is physically ready.

## Option A — Board detection first

Use this if the overhead camera is mounted and accessible.

Focus:

```text
camera capture
board calibration
black-side square mapping
manual or semi-manual grid calibration
occupancy-grid rendering
```

## Option B — Servo calibration first

Use this if the arm electronics are ready.

Focus:

```text
servo scan
read positions
joint mapping
dry-run validation
small real motion
joint limits
home pose
```

Both are valid.

Do not connect them until each side works independently.

---

# 25. Current Board Detection Priorities

Highest priorities:

```text
1. Capture overhead frame.
2. Load camera calibration if available.
3. Undistort frame if calibration exists.
4. Calibrate board with robot-black orientation.
5. Support seam-aware grid model.
6. Generate square labels.
7. Generate central crop polygons.
8. Save board profile YAML.
9. Render board overlay.
10. Render occupancy crop overlay.
```

Avoid:

```text
CNN piece classification
wrist-camera inspection
live stream GUI
automatic gameplay
```

---

# 26. Current Servo Priorities

Highest priorities:

```text
1. Detect serial port.
2. Scan servo IDs.
3. Read current positions.
4. Map IDs to joint names.
5. Create dry-run command validation.
6. Add safety checks.
7. Add logging.
8. Perform one-joint micro-movement only after dry-run.
9. Record direction.
10. Record safe limits.
11. Save home pose.
12. Save gripper profile.
```

Avoid:

```text
pick-and-place
square targeting
full arm trajectories
chess moves
vision integration
```

---

# 27. Future Wrist-Camera Visual Refinement

The wrist camera is an optional later extension.

Do not implement until:

- overhead board detection works
- servo calibration works
- square-to-robot calibration works
- pick-and-place works
- basic game loop works or nearly works

Best later use:

```text
move above target square
capture wrist image
detect piece centre
estimate offset from gripper centre
apply small correction
capture again
descend and grasp
```

This is not full continuous visual servoing.

It is local image-based correction.

Do not use wrist camera as the first ambiguity resolver.

---

# 28. Known Risks and Mitigations

## 28.1 Vision risks

Risks:

- glare
- shadows from human hand
- seam interfering with occupancy
- piece colour blending with board
- board/camera shift
- false changed-square detection
- exposure variation

Mitigations:

- fixed camera
- fixed board
- central crops
- ignored seam polygon
- temporal filtering
- debug overlays
- manual confirmation
- saved raw frames

---

## 28.2 Robot risks

Risks:

- wrong servo ID
- reversed joint direction
- unsafe joint limits
- gripper slipping
- collision with board
- placement error near square boundaries
- power instability
- loose mechanical parts

Mitigations:

- dry-run default
- one-joint testing
- conservative limits
- home pose
- explicit `--real` flag
- typed confirmation
- logging
- gradual calibration

---

## 28.3 Integration risks

Risks:

- symbolic board state diverges from physical board
- ambiguous captures
- unsupported special moves
- camera uncertainty after robot motion
- manual correction not logged
- failed robot motion but board state still updated

Mitigations:

- verification after each move
- manual confirmation
- FEN logging
- occupancy display
- recovery state
- only update board state after accepted move or confirmed execution

---

# 29. Definition of Done for the Whole System

The project is functionally successful if:

```text
1. The robot initialises safely.
2. The overhead camera captures the board.
3. The system detects or displays board occupancy.
4. The system displays what it thinks the board state is.
5. The human makes a legal move.
6. The system detects or asks for confirmation of the human move.
7. python-chess updates the symbolic board state.
8. Stockfish selects a black move.
9. The task planner converts that move into physical actions.
10. The robot executes the move using calibrated primitives.
11. The overhead camera verifies the resulting board state.
12. Failures are logged and recoverable.
```

The project does not need to beat humans at chess.

The project needs to demonstrate a reliable perception-to-action robotics pipeline.

---

# 30. Minimum Viable Demonstration

Minimum demonstration:

```text
1. Fixed board and overhead camera are calibrated.
2. Human makes a simple legal move.
3. Overhead camera detects changed squares.
4. System asks for confirmation if needed.
5. Stockfish or fixed rule selects a black response.
6. Robot picks and places one piece.
7. Terminal or PNG display shows updated board state.
8. Logs and images prove what happened.
```

Better demonstration:

```text
several moves in sequence with successful verification
```

Best demonstration:

```text
partial or full game with manual ambiguity assistance
```

---

# 31. What Not to Build Early

Do not build these early:

```text
full CNN piece recogniser
wrist-camera inspection fallback
real-time visual servoing
web dashboard
ROS-like messaging system
general inverse kinematics solver
dynamic obstacle avoidance
voice interaction
complex multi-agent Codex workflow
```

These are distractions until the basic pipeline works.

---

# 32. One-Sentence Architecture

The chess robot is a Python-only Jetson Nano system that uses a fixed overhead camera and calibrated board model to detect occupancy and human moves, maintains the true chess state with python-chess, asks for manual confirmation when ambiguous, uses Stockfish to choose black moves, and executes them through safety-checked calibrated servo motion primitives.
