# Servo Metrics Summary

Source files: `data/logs/servo.log`, `data/calibration/robot/servo_map.yaml`, `data/calibration/robot/joint_limits.yaml`, `data/calibration/robot/joint_directions.yaml`, `data/calibration/robot/home_pose.yaml`, `data/calibration/robot/servo_snapshot.yaml`, `data/calibration/gripper/gripper_profile.yaml`.

## 1. Servo ID Mapping
| joint name | servo ID | calibrated |
| --- | --- | --- |
| shoulder_pan | 1 | true |
| shoulder_lift | 2 | true |
| elbow_flex | 3 | true |
| wrist_flex | 4 | true |
| wrist_roll | 5 | true |
| gripper | 6 | true |

## 2. Servo Tick Range
| joint name | provisional_min | provisional_max | neutral | margin_ticks | span_ticks | calibrated | direction sign | physical direction description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| shoulder_pan | 876 | 3078 | 1074 | 20 | 2202 | true | 1 | increasing ticks rotates the arm clockwise when viewed from above |
| shoulder_lift | 857 | 3232 | 895 | 20 | 2375 | true | 1 | increasing ticks raises the upper arm |
| elbow_flex | 923 | 3040 | 3030 | 20 | 2117 | true | 1 | increasing ticks bends the elbow inward |
| wrist_flex | 1145 | 2935 | 1905 | 20 | 1790 | true | 1 | increasing ticks lowers the gripper tip when viewed from the side |
| wrist_roll | 379 | 2350 | 1093 | 20 | 1971 | true | 1 | increasing ticks rotates wrist clockwise when viewed from the gripper end |
| gripper | 1463 | 1738 | 1616 | 10 | 275 | true | -1 | decreasing ticks closes the moving jaw against the fixed jaw |

Formula: `span_ticks = provisional_max - provisional_min`.

## 3. Home Pose Evidence
| joint name | home position | within min/max | distance from min | distance from max |
| --- | --- | --- | --- | --- |
| shoulder_pan | 1074 | true | 198 | 2004 |
| shoulder_lift | 1990 | true | 1133 | 1242 |
| elbow_flex | 1857 | true | 934 | 1183 |
| wrist_flex | 2706 | true | 1561 | 229 |
| wrist_roll | 1091 | true | 712 | 1259 |
| gripper | 1602 | true | 139 | 136 |

Formula: `distance_from_min = home_position - min_limit` and `distance_from_max = max_limit - home_position`.

## 4. Gripper Profile Evidence
- open_position: 1704
- pre_grasp_position: 1596
- grasp_position: 1510
- release_position: 1652
- neutral_position: 1616
- direction_sign: -1
- min/max limits: 1463 / 1738
- notes: Previous 1033..1293 / 1122..1250 gripper values are stale after LeRobot recalibration. | Active profile was manually recorded after LeRobot recalibration. | Asymmetric gripper: one moving jaw, one fixed jaw.
- grasp_position above calibrated min: true
- grasp_position - min_limit = 47 ticks
- full mechanical close: not explicitly evidenced; the file only records calibrated min/max limits.
- chess piece used for grasp calibration: not evidenced.
- previous_profile values: open_position=1250, pre_grasp_position=1176, grasp_position=1122, release_position=1185, neutral_position=1138, direction_sign=-1.
- previous_profile evidence: calibration-save records exist in servo.log at 2026-05-17T16:31:16Z, 2026-05-17T16:35:49Z, 2026-05-17T16:42:04Z, 2026-05-17T16:46:11Z, and 2026-05-17T16:48:46Z.
- previous_profile goal-to-actual error: not evidenced in current logs.

## 5. Dry-Run Evidence
| category | count | first timestamp | last timestamp | representative examples | writes prevented |
| --- | --- | --- | --- | --- | --- |
| servo_scan_dry_run | 5 | 2026-05-16T14:57:50.630113Z | 2026-05-16T14:57:50.630968Z | servo_scan_start / servo_scan_complete | yes |
| read_positions_dry_run | 83 | 2026-05-16T14:57:50.912442Z | 2026-05-18T03:04:36.230747Z | servo_position_read_start / servo_read_position; status=ok; servo_id=6 | yes |
| torque_request_refused | 8 | 2026-05-17T06:46:05.061681Z | 2026-05-17T09:44:12.667040Z | servo_torque_request; status=refused; enabled=False; reason=missing_real / servo_torque_request; status=refused; joint=gripper; enabled=False; reason=missing_real | yes |
| single_joint_move_attempt_dry_run | 10 | 2026-05-18T03:04:35.458142Z | 2026-05-18T04:08:40.608158Z | single_joint_move_attempt; joint=gripper; servo_id=6; success=False / single_joint_move_attempt; joint=gripper; servo_id=6; target=2049; success=False | yes |
| gripper_motion_dry_run_plan | 6 | 2026-05-19T07:26:57.235109Z | 2026-05-27T03:54:03.215277Z | gripper_monitored_motion; step=dry_run_plan; status=ok; servo_id=6 / gripper_monitored_motion; step=dry_run_plan; status=ok; servo_id=6 | yes |

Dry-run write evidence is limited to refusal/planning records in the log; no successful hardware write appears in the dry-run categories.

## 6. Real Hardware Read Evidence
- detected servo IDs from real scan: [1, 2, 3, 4, 5, 6]
- successful real position-read snapshots: 5 of 7 real snapshots
| servo ID | joint name | successful read events | latest live position | latest read timestamp |
| --- | --- | --- | --- | --- |
| 1 | shoulder_pan | 1328 | 1076 | 2026-05-27T07:29:28.231095Z |
| 2 | shoulder_lift | 1260 | 2069 | 2026-05-27T07:29:28.231818Z |
| 3 | elbow_flex | 1264 | 1908 | 2026-05-27T07:29:28.232450Z |
| 4 | wrist_flex | 1259 | 2705 | 2026-05-27T07:29:28.233027Z |
| 5 | wrist_roll | 1364 | 1088 | 2026-05-27T07:29:28.233568Z |
| 6 | gripper | 462 | 1645 | 2026-05-27T07:29:24.252150Z |

## 7. Torque Evidence
| joint name | servo ID | real enable writes | real disable writes | enable observed | disable observed |
| --- | --- | --- | --- | --- | --- |
| shoulder_pan | 1 | 42 | 43 | true | true |
| shoulder_lift | 2 | 42 | 43 | true | true |
| elbow_flex | 3 | 41 | 43 | true | true |
| wrist_flex | 4 | 42 | 44 | true | true |
| wrist_roll | 5 | 41 | 42 | true | true |
| gripper | 6 | 30 | 51 | true | true |

- torque request refusals: 8
- real torque writes observed for servo IDs 1-6; each joint has at least one enable and one disable write.

## 8. Goal vs Actual Error
- movement-trial source: `gripper_monitored_motion` `final_validation` events only.
- joint coverage: gripper only.
| joint | sample count |
| --- | --- |
| gripper | 25 |
- trial count: 25
- mean absolute error: 8.56 ticks
- max absolute error: 37 ticks
- Formula: `signed_error = actual_position - target_position`; `absolute_error = abs(signed_error)`.
- Previous gripper profile actual-position error: not evidenced; the old profile appears only in calibration-save records and does not have a linked target/readback trail.
- General single-joint move actual-position error: not evidenced in `servo.log`; the one real `single_joint_move_attempt` only logs goal-position readback.

## 9. Safety Evidence
- scan/read were read-only: the log shows dry-run scan and read batches.
- dry-run refused hardware writes: `servo_torque_request` logs `status=refused` with `reason=missing_real`.
- torque writes used only explicit real commands: real `servo_torque` writes are logged separately from the refusal path.
- all six joints passed validation: the calibration validator records `pass_count = 6` and `fail_count = 0`.
- no multi-joint movement evidence exists in `servo.log`.

## Not Evidenced
- chess piece used for grasp calibration.
- full mechanical close position as a separate recorded tick value.
- general multi-joint move trials in `servo.log`.


## 10. Saved EEPROM Values
The log-backed saved register values are the latest `servo_read_register` reads at addresses 9 and 11 for each servo.
| joint name | servo ID | register 9 value | register 11 value | register 9 timestamp | register 11 timestamp |
| --- | --- | --- | --- | --- | --- |
| shoulder_pan | 1 | 876 | 3113 | 2026-05-26T04:06:27.667572Z | 2026-05-26T04:06:27.668259Z |
| shoulder_lift | 2 | 857 | 3230 | 2026-05-26T04:06:27.668965Z | 2026-05-26T04:06:27.669491Z |
| elbow_flex | 3 | 837 | 3041 | 2026-05-26T04:06:27.670132Z | 2026-05-26T04:06:27.670773Z |
| wrist_flex | 4 | 776 | 3110 | 2026-05-26T04:06:27.671333Z | 2026-05-26T04:06:27.671958Z |
| wrist_roll | 5 | 0 | 4095 | 2026-05-26T04:06:27.672473Z | 2026-05-26T04:06:27.672972Z |
| gripper | 6 | 1460 | 2880 | 2026-05-27T04:13:52.090599Z | 2026-05-27T04:13:52.091200Z |

Formula: these are direct register-read values from `servo.log`; no arithmetic was applied.

## Old Profile Supplementary Files
- data/evaluation/servo_gripper_profile_history.csv contains the active and previous gripper profile values.
- data/evaluation/servo_goal_actual_errors_old_profile.csv records the absence of old-profile goal-to-actual error evidence.
