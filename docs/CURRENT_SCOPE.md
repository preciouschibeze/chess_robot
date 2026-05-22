# Current Scope

This document defines the active development scope for the chess robot project.

It overrides broader future architecture notes. If a feature is not listed here, treat it as out of scope unless a task explicitly brings it back in.

---

## Active Milestone

The current milestone is:

```text
one reliable non-visual pick-and-place sequence
```

The goal is to move one chess piece from one calibrated source square to one calibrated destination square using safety-checked robot motion and gripper control, then verify the result with the overhead camera.

This is the practical bridge between calibration work and autonomous gameplay.

---

## Current Objective

Build and validate this sequence:

```text
1. move_home
2. move_above_source
3. descend_to_pick
4. close_gripper
5. lift
6. move_above_destination
7. descend_to_place
8. release_gripper
9. lift
10. move_home
11. capture overhead image
12. verify expected occupancy change
13. save logs and debug outputs
```

The test should begin with:

```text
one piece
one source square
one destination square
one taught pick pose
one taught place pose
no wrist-camera correction
no autonomous gameplay
```

---

## Current Development Focus

### 1. Square-target validation

Validate that calibrated square targets are physically usable.

Active work:

- inspect existing square target file
- audit generated and manual targets
- test safe above-square motion
- identify bad or risky square targets
- avoid using generated poses that have not been physically checked
- preserve joint-limit margins
- record failed or suspicious squares

Required outputs:

```text
data/calibration/robot/square_targets.yaml
data/debug/square_target_audit.json
data/logs/servo.log
```

Acceptance condition:

```text
The robot can move safely above the selected source and destination squares without collision or limit violation.
```

---

### 2. Pick/place pose addition

The current robot-square calibration must move beyond `above_pose`.

For the selected test squares, define:

```text
above_pose
pick_pose
place_pose
```

Do not generate all pick/place poses automatically yet.

Start with only the minimum required pair:

```text
source square: above_pose + pick_pose
destination square: above_pose + place_pose
```

Acceptance condition:

```text
The robot can descend from the above pose to the pick/place pose safely and return to the above pose.
```

---

### 3. Gripper software boundary

The gripper profile has to be connected cleanly to robot control.

Active work:

- load the calibrated gripper profile
- expose open/pre-grasp/grasp/release actions
- use dry-run by default
- require explicit real-motion confirmation
- route movement through the existing safety path
- log all gripper commands

Expected profile path:

```text
data/calibration/gripper/gripper_profile.yaml
```

Acceptance condition:

```text
The gripper can open, grasp, and release the selected piece repeatably without bypassing safety checks.
```

---

### 4. Non-visual pick-and-place execution

Implement or validate the narrow primitive chain needed for one physical move.

Required primitive actions:

```text
move_home
move_above_square
descend_to_pick
close_gripper
lift_from_square
move_above_square
descend_to_place
release_gripper
lift_from_square
move_home
```

The first physical test should not use wrist-camera correction.

Reason:

```text
A blind baseline is needed before visual refinement can be judged.
```

Acceptance condition:

```text
The robot completes one pick-and-place sequence without collision, dropped piece, unsafe joint motion, or unlogged state changes.
```

---

### 5. Overhead-camera verification

After physical movement, use the overhead camera to check the result.

Verification target:

```text
source square changed from occupied to empty
destination square changed from empty to occupied
```

Active work:

- capture pre-move board image
- capture post-move board image
- run occupancy detection
- run changed-square detection
- compare expected and detected changes
- save verification outputs

Required outputs:

```text
occupancy_snapshot.json
occupancy_diagnostics.json
transition_result.json
transition_grid.png
```

Acceptance condition:

```text
The system can confirm whether the physical board changed as expected.
```

---

### 6. Failure measurement

The first pick-and-place attempt is also a measurement exercise.

Record:

- backlash offset
- missed square centre
- descent-height error
- gripper slip
- failed release
- piece tilt
- collision risk
- servo readback mismatch
- occupancy verification failure
- any manual correction used

The goal is not to hide failure. The goal is to classify it precisely.

---

## In Scope

Currently allowed work:

- README and scope documentation
- safety documentation updates
- square-target inspection and auditing
- selected square target correction
- above-square motion testing
- pick/place pose teaching for selected squares
- gripper profile integration
- guarded gripper movement
- one-piece pick-and-place primitive execution
- overhead verification after motion
- logging and debug output improvements
- narrow tests for the above

---

## Out of Scope

Do not work on these now:

- full autonomous gameplay
- full 64-square pick/place calibration
- automatic gameplay loop
- wrist-camera visual servoing
- live GUI or web dashboard
- CNN piece recognition
- ROS
- MoveIt
- general inverse kinematics
- dynamic obstacle avoidance
- reinforcement learning
- voice interaction
- broad architecture rewrites

These are distractions until the physical pick-and-place baseline is measured.

---

## Safety Constraints

Hardware work must follow these rules:

```text
1. Dry-run by default.
2. Real motion requires an explicit flag and confirmation.
3. Servo IDs must be mapped before motion.
4. Joint limits must be loaded before motion.
5. Target poses must be checked before execution.
6. Motion must be logged.
7. No raw serial writes outside chess_robot/robot/servo_bus.py.
8. No module should bypass chess_robot/robot/safety.py.
9. Failed movement must stop the sequence.
10. Board state must not be updated blindly after failed physical execution.
```

---

## Current Success Criteria

This scope is complete when:

```text
1. A source square and destination square are selected.
2. Their above/pick/place poses are defined as needed.
3. The robot can move above both squares safely.
4. The gripper can grasp and release the selected piece.
5. The robot completes one non-visual pick-and-place attempt.
6. The overhead camera captures the board before and after motion.
7. Occupancy verification reports whether the expected change occurred.
8. Logs and debug outputs are saved.
9. Failure modes are documented clearly.
```

---

## Next Scope After This

Only after the above is complete, the next scope may include:

```text
wrist-camera local correction
repeatability testing over several moves
more square pick/place calibration
capture-zone handling
robot move execution from planned chess moves
partial autonomous turn loop
```

Do not start these until the current physical baseline is known.
