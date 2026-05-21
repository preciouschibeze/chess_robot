# Gripper Safety Audit (Post LeRobot Recalibration)

## Summary

- Previous software gripper profile range was `1033..1293`.
- Previous live EEPROM range observed before recalibration was `2043..3552`.
- That mismatch caused command clamping/overshoot behavior risk.

## Recalibration outcome

- LeRobot recalibration updated gripper EEPROM angle limits to `1460..2880`.
- Live present position after recalibration is `1462`.
- Old chess_robot gripper profile values are now stale and must not be used as active targets.

## Safety status

- Real movement remains disabled in `tools/move_servo_small_step.py` for `--real`.
- Do not re-enable real movement until gripper-only profile recalibration is completed and validated.

## Next step

Perform gripper-only profile recalibration with tiny, monitored moves and readback checks.

Do not use old `1122` / `1176` / `1185` / `1250` profile values.
