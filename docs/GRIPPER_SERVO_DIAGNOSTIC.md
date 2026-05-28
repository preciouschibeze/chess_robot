# Gripper Servo Diagnostic Audit

## Scope

This audit covers gripper servo ID 6 monitored motion and torque state diagnosis. No EEPROM writes, servo ID changes, multi-joint motion, pick-and-place, or joints 1-5 commands were added.

## Register Map In Use

`chess_robot/robot/servo_bus.py` and `tools/test_gripper_monitored_motion.py` use the same Feetech/ST register map:

- Present_Position: address `56`, length `2`
- Goal_Position: address `42`, length `2`
- Torque_Enable: address `40`, length `1`
- Operating_Mode: address `33`, length `1`
- Hardware_Error_Status: address `65`, length `1`
- Moving: address `66`, length `1`
- EEPROM Min_Angle_Limit: address `9`, length `2`
- EEPROM Max_Angle_Limit: address `11`, length `2`

The current code comments cite LeRobot STS/SMS control-table verification for `Torque_Enable` and `Goal_Position`.

## Repo Locations That Can Write Torque Or Goal Position

- `chess_robot/robot/servo_bus.py`: central hardware write facade and Feetech backend. It implements `ServoBus.torque_enable()` and `ServoBus.write_goal_position()` and refuses both in dry-run.
- `tools/test_gripper_monitored_motion.py`: gripper-only ID 6 writes to `Goal_Position` and `Torque_Enable` in real mode after safety checks and typed movement confirmation.
- `tools/diagnose_gripper_servo_state.py`: new diagnostic. Default read-only. Optional `--disable-torque` writes only `Torque_Enable=0` for ID 6 and verifies readback.
- `chess_robot/robot/arm_controller.py`: general single-joint executor writes `Goal_Position` through `ServoBus.write_goal_position()` after validation and typed confirmation. Current real-movement allowlist permits only `gripper`.
- `chess_robot/robot/ik_validation.py`: can write waypoint goal positions through `ServoBus.write_goal_position()` in execution paths.
- `chess_robot/robot/safe_transfer.py`: can write waypoint goal positions through `ServoBus.write_goal_position()` in transfer paths.
- `tools/calibrate_joint_limits.py`: can enable or disable torque for one selected mapped joint only with `--real` and exact confirmation.
- `tools/calibrate_servos.py`: can enable or disable torque for one selected mapped joint only with `--real` and exact confirmation; all-servo torque commands are refused.
- `tools/teleop_joints_keyboard.py`: can enable/disable torque and write goals during explicit teleop execution. Gripper is included only with `--allow-gripper`.
- `tools/test_square_above_motion.py`: can enable/disable torque and write goal positions for square-above testing. Gripper is excluded unless `--include-gripper`.
- `tools/test_open_loop_pick_place.py`: can enable/disable torque and write goal positions for open-loop pick/place execution, including gripper stages.
- `tools/jog_and_save_square_pose.py`: can enable/disable torque and write goal positions for explicit jog/teach flows.

Read-only tools inspected, such as `tools/read_servo_positions.py`, `tools/report_servo_angle_limits.py`, `tools/inspect_servo_registers.py`, `tools/calibrate_home_pose.py`, and `tools/calibrate_gripper.py`, do not command movement or torque.

## Automatic Re-Enable Assessment

No import side effect or module initialization was found that automatically enables gripper torque. The write paths above require explicit tool execution and safety gates. A later run seeing `Torque_Enable=1` after a previous immediate readback of `0` is therefore most consistent with one of these possibilities:

- another running process/tool re-enabled torque after script exit,
- the disable write/readback path is not durable or is being overwritten,
- backend/register behavior differs from the assumed Feetech/ST map,
- or hardware state changed outside this process.

## Gap Found

`tools/test_gripper_monitored_motion.py` printed that torque was disabled, but on normal successful write status it did not require a readback match inside `_write_torque_with_readback()`. It did read torque before final goal alignment, but that was not reported as the final disable verification or used as the explicit final-disable exit condition.

## Changes Made

- Added `tools/diagnose_gripper_servo_state.py`.
- Added readback verification for normal `Torque_Enable` writes in `tools/test_gripper_monitored_motion.py`.
- Added startup warning when initial `Torque_Enable` is already `1`.
- Added `--require-initial-torque-disabled` to refuse real motion if torque starts enabled.
- Added `--disable-before-start` to disable and verify torque before proceeding to the normal typed real-motion confirmation flow.
- Added final `Torque_Enable` readback after final disable; if it remains enabled, the tool exits nonzero and warns about a competing process or failed write.
- Kept dry-run as default, max delta limits, typed confirmation for real movement, and the no-observed-motion failure.
- Added hardware-free tests for diagnostic parsing/loading and monitored torque policy/verification.

## Safe Commands To Run Next

Read-only state diagnosis:

```bash
cd /data/chess_robot
source .venv/bin/activate
python3 tools/diagnose_gripper_servo_state.py --backend feetech
```

Safe ID 6 torque-disable diagnosis:

```bash
cd /data/chess_robot
source .venv/bin/activate
python3 tools/diagnose_gripper_servo_state.py --backend feetech --disable-torque --verify-duration 3.0
```

Guarded monitored motion only after reviewing diagnostic output:

```bash
cd /data/chess_robot
source .venv/bin/activate
python3 tools/test_gripper_monitored_motion.py --delta -5 --real --require-initial-torque-disabled
```

If torque starts enabled and you want the tool to make it safe before continuing to the normal typed confirmation flow:

```bash
cd /data/chess_robot
source .venv/bin/activate
python3 tools/test_gripper_monitored_motion.py --delta -5 --real --disable-before-start
```

## Interpreting Results

A competing process is indicated by any of these:

- diagnostic output lists another Python robot/servo process,
- `fuser` or `lsof` shows another process using the configured serial port or `/dev/serial/by-id/*`,
- `--disable-torque --verify-duration 3.0` reads `0` immediately and then reads `1` later.

A register/backend mismatch is indicated by any of these:

- `Torque_Enable` readback does not change to `0` immediately after a successful disable write,
- `Goal_Position` readback does not match the written goal while status reports success,
- `Moving` or `Hardware_Error_Status` values are unavailable or nonsensical while Present_Position remains readable,
- EEPROM min/max readbacks conflict with known calibrated limits.

Mechanical binding or insufficient delta/deadband is indicated by this pattern:

- `Torque_Enable=1`, `Operating_Mode=0`, `Hardware_Error_Status=0`, and `Moving` pulses or clears normally,
- `Goal_Position` changes to the target,
- Present_Position remains unchanged or changes less than `min_observed_delta`,
- no competing process is detected and torque remains disabled after the diagnostic disable loop.

## Current Likely Root Cause

Based on the servo log observed before these changes, the tool read `Torque_Enable=0` immediately after final disable and final goal alignment, while a later run started with `Torque_Enable=1`. No automatic import or controller initialization path was found that re-enables torque. The most likely cause is still unknown, with a competing process or another explicitly run tool being the leading hypothesis. The new diagnostic loop is intended to separate competing-process re-enable from register/backend mismatch and mechanical/deadband behavior.
