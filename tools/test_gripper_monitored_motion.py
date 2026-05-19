#!/usr/bin/env python3
"""Dedicated gripper-only monitored motion test.

This tool is intentionally narrower than the general single-joint movement
path. Real writes are limited to servo ID 6 Goal_Position and Torque_Enable.
"""

from __future__ import print_function

import argparse
import os
import sys
import time

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.servo_bus import (  # noqa: E402
    BackendUnavailable,
    ServoBusError,
    build_servo_bus,
    load_robot_config,
)

GRIPPER_JOINT = "gripper"
GRIPPER_SERVO_ID = 6
DEFAULT_MAX_DELTA = 5
DEFAULT_JUMP_LIMIT = 5
DEFAULT_TARGET_TOLERANCE = 3
DEFAULT_MIN_OBSERVED_DELTA = 2
DEFAULT_MIN_POLL_SECONDS = 1.0
DEFAULT_TIMEOUT_SECONDS = 5.0
DEFAULT_POLL_INTERVAL_SECONDS = 0.2

SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
GRIPPER_PROFILE_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "gripper_profile.yaml")

REG_PRESENT_POSITION = (56, 2)
REG_GOAL_POSITION = (42, 2)
REG_OPERATING_MODE = (33, 1)
REG_TORQUE_ENABLE = (40, 1)
REG_MIN_ANGLE_LIMIT = (9, 2)
REG_MAX_ANGLE_LIMIT = (11, 2)
REG_MOVING = (66, 1)
REG_HARDWARE_ERROR_STATUS = (65, 1)
MISSING_WRITE_STATUS_TEXT = "No valid status response after register write"


class GripperMotionError(RuntimeError):
    """Raised when the monitored gripper test refuses to proceed."""


def build_parser():
    parser = argparse.ArgumentParser(
        description="Dedicated ID 6 gripper-only monitored motion test. Dry-run is default."
    )
    parser.add_argument(
        "--config",
        default=os.path.join(ROOT, "configs", "robot.yaml"),
        help="Path to robot YAML config. Default: configs/robot.yaml",
    )
    parser.add_argument(
        "--delta",
        required=True,
        type=int,
        help="Tiny relative target offset in raw ticks. Positive opens, negative closes.",
    )
    parser.add_argument(
        "--max-delta",
        type=int,
        default=DEFAULT_MAX_DELTA,
        help="Maximum allowed absolute delta. Default: 5.",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="Use live Feetech backend and allow the guarded ID 6 writes after typed confirmation.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help="Polling timeout in seconds after writing the target. Default: 5.0.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=DEFAULT_POLL_INTERVAL_SECONDS,
        help="Polling interval in seconds. Default: 0.2.",
    )
    parser.add_argument(
        "--min-observed-delta",
        type=int,
        default=DEFAULT_MIN_OBSERVED_DELTA,
        help="Minimum required physical movement in raw ticks. Default: 2.",
    )
    parser.add_argument(
        "--tolerance",
        type=int,
        default=DEFAULT_TARGET_TOLERANCE,
        help="Allowed final target error in raw ticks. Default: 3.",
    )
    return parser


def _read_yaml(path):
    if yaml is None:
        raise GripperMotionError("PyYAML is required to read calibration files.")
    if not os.path.exists(path):
        raise GripperMotionError("Required calibration file is missing: {}".format(path))
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise GripperMotionError("YAML file must contain a mapping: {}".format(path))
    return data


def _to_int(value, label):
    if isinstance(value, bool):
        raise GripperMotionError("{} must be an integer.".format(label))
    try:
        return int(value)
    except (TypeError, ValueError):
        raise GripperMotionError("{} must be an integer.".format(label))


def _load_gripper_calibration():
    servo_map = _read_yaml(SERVO_MAP_PATH)
    servo_joints = servo_map.get("joints") or {}
    servo_entry = servo_joints.get(GRIPPER_JOINT) or {}
    servo_id = _to_int(servo_entry.get("id"), "servo_map gripper id")
    if servo_id != GRIPPER_SERVO_ID:
        raise GripperMotionError(
            "Refusing: servo_map gripper id is {}, expected {}.".format(
                servo_id, GRIPPER_SERVO_ID
            )
        )

    joint_limits = _read_yaml(JOINT_LIMITS_PATH)
    limits = joint_limits.get("limits") or {}
    joint_entry = limits.get(GRIPPER_JOINT) or {}
    if not bool(joint_entry.get("calibrated")):
        raise GripperMotionError("Refusing: joint_limits gripper entry is not calibrated.")
    joint_min = _to_int(joint_entry.get("provisional_min"), "joint_limits gripper provisional_min")
    joint_max = _to_int(joint_entry.get("provisional_max"), "joint_limits gripper provisional_max")

    profile_data = _read_yaml(GRIPPER_PROFILE_PATH)
    profile = profile_data.get("gripper") or {}
    profile_servo_id = _to_int(profile.get("servo_id"), "gripper_profile servo_id")
    if profile_servo_id != GRIPPER_SERVO_ID:
        raise GripperMotionError(
            "Refusing: gripper_profile servo_id is {}, expected {}.".format(
                profile_servo_id, GRIPPER_SERVO_ID
            )
        )
    if profile.get("joint") != GRIPPER_JOINT:
        raise GripperMotionError("Refusing: gripper_profile joint is not gripper.")
    if not bool(profile.get("active_profile_valid")):
        raise GripperMotionError("Refusing: gripper active profile is not marked valid.")

    profile_limits = profile.get("limits") or {}
    profile_min = profile_limits.get("min")
    profile_max = profile_limits.get("max")
    if profile_min is None:
        profile_min = joint_min
    if profile_max is None:
        profile_max = joint_max
    active_min = _to_int(profile_min, "gripper active min")
    active_max = _to_int(profile_max, "gripper active max")
    if active_min >= active_max:
        raise GripperMotionError("Refusing: invalid gripper active limits {}..{}.".format(active_min, active_max))

    return {
        "servo_id": servo_id,
        "joint_min": joint_min,
        "joint_max": joint_max,
        "active_min": active_min,
        "active_max": active_max,
        "profile": profile,
    }


def _format_range(minimum, maximum):
    return "{}..{}".format(minimum, maximum)


def _inside(value, minimum, maximum):
    return value is not None and int(minimum) <= int(value) <= int(maximum)


def _movement_in_requested_direction(observed_delta, requested_delta, min_observed_delta):
    observed_delta = int(observed_delta)
    requested_delta = int(requested_delta)
    min_observed_delta = int(min_observed_delta)
    if requested_delta > 0:
        return observed_delta > 0 and abs(observed_delta) >= min_observed_delta
    if requested_delta < 0:
        return observed_delta < 0 and abs(observed_delta) >= min_observed_delta
    return False


def _log(bus, step, status, **fields):
    if bus is None:
        return
    event = {
        "step": step,
        "status": status,
        "servo_id": GRIPPER_SERVO_ID,
        "tool": "test_gripper_monitored_motion",
    }
    event.update(fields)
    bus.logger.log("gripper_monitored_motion", **event)


def _is_missing_write_status(exc):
    return MISSING_WRITE_STATUS_TEXT in str(exc)


def _print_write_step(write_step, intended_register, intended_value):
    print(
        "write_step: {} intended_register: {} intended_value: {}".format(
            write_step, intended_register, intended_value
        )
    )


def _log_write_result(
        bus, write_step, intended_register, intended_value, status_response_ok,
        readback_verified, readback_value, continued_after_missing_status,
        status="ok", **fields):
    fields.update({
        "write_step": write_step,
        "intended_register": intended_register,
        "intended_value": intended_value,
        "status_response_ok": bool(status_response_ok),
        "readback_verified": bool(readback_verified),
        "readback_value": readback_value,
        "continued_after_missing_status": bool(continued_after_missing_status),
    })
    _log(bus, "write_{}".format(write_step), status, **fields)


def _read_required_register(bus, label, spec):
    address, length = spec
    value = bus.read_register(GRIPPER_SERVO_ID, address, length)
    if value is None:
        raise GripperMotionError("Refusing: {} register is unreadable.".format(label))
    return int(value)


def _write_goal_with_readback(bus, write_step, intended_value, allow_verified_continue):
    intended_value = int(intended_value)
    _print_write_step(write_step, "Goal_Position", intended_value)
    try:
        bus.write_goal_position(GRIPPER_SERVO_ID, intended_value)
        _log_write_result(
            bus, write_step, "Goal_Position", intended_value,
            status_response_ok=True,
            readback_verified=False,
            readback_value=None,
            continued_after_missing_status=False,
        )
        return False
    except ServoBusError as exc:
        if not _is_missing_write_status(exc):
            _log_write_result(
                bus, write_step, "Goal_Position", intended_value,
                status_response_ok=False,
                readback_verified=False,
                readback_value=None,
                continued_after_missing_status=False,
                status="error",
                error=str(exc),
            )
            raise
        readback_value = _read_required_register(bus, "goal position readback", REG_GOAL_POSITION)
        readback_verified = int(readback_value) == intended_value
        continued = bool(allow_verified_continue and readback_verified)
        _log_write_result(
            bus, write_step, "Goal_Position", intended_value,
            status_response_ok=False,
            readback_verified=readback_verified,
            readback_value=readback_value,
            continued_after_missing_status=continued,
            status="write_status_missing_but_readback_verified" if continued else "error",
            error=str(exc),
        )
        if continued:
            print("write_status_missing_but_readback_verified")
            return True
        raise GripperMotionError(
            "Refusing: {} write status was missing and Goal_Position readback was {}.".format(
                write_step, readback_value
            )
        )


def _write_target_goal(bus, target):
    target = int(target)
    write_step = "command_target"
    _print_write_step(write_step, "Goal_Position", target)
    try:
        bus.write_goal_position(GRIPPER_SERVO_ID, target)
        _log_write_result(
            bus, write_step, "Goal_Position", target,
            status_response_ok=True,
            readback_verified=False,
            readback_value=None,
            continued_after_missing_status=False,
        )
        return {
            "missing_status": False,
            "readback_verified": False,
            "readback_value": None,
        }
    except ServoBusError as exc:
        if not _is_missing_write_status(exc):
            _log_write_result(
                bus, write_step, "Goal_Position", target,
                status_response_ok=False,
                readback_verified=False,
                readback_value=None,
                continued_after_missing_status=False,
                status="error",
                error=str(exc),
            )
            raise
        readback_value = _read_required_register(bus, "goal position readback", REG_GOAL_POSITION)
        readback_verified = int(readback_value) == target
        continued = bool(readback_verified)
        _log_write_result(
            bus, write_step, "Goal_Position", target,
            status_response_ok=False,
            readback_verified=readback_verified,
            readback_value=readback_value,
            continued_after_missing_status=continued,
            status="write_status_missing_but_readback_verified" if continued else "error",
            error=str(exc),
        )
        if not continued:
            raise GripperMotionError(
                "Refusing: command_target write status was missing and Goal_Position readback was {}.".format(
                    readback_value
                )
            )
        print("write_status_missing_but_readback_verified")
        return {
            "missing_status": True,
            "readback_verified": True,
            "readback_value": readback_value,
        }


def _write_torque_with_readback(bus, write_step, enabled, allow_verified_continue):
    intended_value = 1 if enabled else 0
    _print_write_step(write_step, "Torque_Enable", intended_value)
    try:
        bus.torque_enable(GRIPPER_SERVO_ID, bool(enabled))
        _log_write_result(
            bus, write_step, "Torque_Enable", intended_value,
            status_response_ok=True,
            readback_verified=False,
            readback_value=None,
            continued_after_missing_status=False,
        )
        return False
    except ServoBusError as exc:
        if not _is_missing_write_status(exc):
            _log_write_result(
                bus, write_step, "Torque_Enable", intended_value,
                status_response_ok=False,
                readback_verified=False,
                readback_value=None,
                continued_after_missing_status=False,
                status="error",
                error=str(exc),
            )
            raise
        readback_value = _read_required_register(bus, "torque enable readback", REG_TORQUE_ENABLE)
        readback_verified = int(readback_value) == intended_value
        continued = bool(allow_verified_continue and readback_verified)
        _log_write_result(
            bus, write_step, "Torque_Enable", intended_value,
            status_response_ok=False,
            readback_verified=readback_verified,
            readback_value=readback_value,
            continued_after_missing_status=continued,
            status="write_status_missing_but_readback_verified" if continued else "error",
            error=str(exc),
        )
        if continued:
            print("write_status_missing_but_readback_verified")
            return True
        raise GripperMotionError(
            "Refusing: {} write status was missing and Torque_Enable readback was {}.".format(
                write_step, readback_value
            )
        )


def _align_final_goal_to_present_with_torque_disabled(bus):
    torque = _read_required_register(bus, "cleanup torque state", REG_TORQUE_ENABLE)
    if torque != 0:
        raise GripperMotionError(
            "Refusing cleanup: torque is still enabled before final goal alignment."
        )
    final_present = _read_required_register(
        bus, "cleanup final present position", REG_PRESENT_POSITION
    )
    _write_goal_with_readback(
        bus, "align_final_goal_to_present", final_present,
        allow_verified_continue=True,
    )
    final_goal = _read_required_register(
        bus, "cleanup final goal position", REG_GOAL_POSITION
    )
    verified = int(final_goal) == int(final_present)
    _log(
        bus,
        "align_final_goal_to_present",
        "ok" if verified else "error",
        final_present=final_present,
        final_goal=final_goal,
        torque=torque,
        verified=verified,
    )
    if not verified:
        raise GripperMotionError(
            "Cleanup failed: Goal_Position readback {} did not match final present {}.".format(
                final_goal, final_present
            )
        )
    print("Final goal aligned to present with torque disabled: {}".format(final_present))


def _print_dry_run_plan(args, calibration, log_path):
    delta = int(args.delta)
    max_delta = int(args.max_delta)
    min_observed_delta = int(args.min_observed_delta)
    tolerance = int(args.tolerance)
    target_expression = "present_position {:+d}".format(delta)
    if abs(delta) > max_delta:
        validation = "REFUSED: abs(delta) {} exceeds max_delta {}".format(abs(delta), max_delta)
    elif delta == 0:
        validation = "REFUSED: delta 0 cannot prove physical movement"
    else:
        validation = "pending live present-position read"

    print("Mode: DRY-RUN")
    print("Real movement: blocked unless --real is supplied")
    print("Servo ID: {} ({})".format(GRIPPER_SERVO_ID, GRIPPER_JOINT))
    print("Allowed writes in real mode: ID 6 goal position; ID 6 torque enable/disable")
    print("Forbidden writes: IDs 1-5, EEPROM, mode, speed, acceleration")
    print("Forbidden actions: multi-joint movement, move_home, IK, pick-and-place, chess logic")
    print("Delta: {}".format(delta))
    print("Max delta: {}".format(max_delta))
    print("Min observed delta: {}".format(min_observed_delta))
    print("Tolerance: {}".format(tolerance))
    print("Active software limits: {}".format(_format_range(calibration["active_min"], calibration["active_max"])))
    print("Joint-limit source range: {}".format(_format_range(calibration["joint_min"], calibration["joint_max"])))
    print("Target expression: {}".format(target_expression))
    print("Validation: {}".format(validation))
    print("Real safety sequence:")
    print("  1. Read ID 6 present, goal, torque, mode, EEPROM min/max")
    print("  2. Refuse unless mode is 0 and present/target are inside active software limits")
    print("  3. Refuse if abs(delta) exceeds max_delta")
    print("  4. Set ID 6 goal equal to present before enabling torque")
    print("  5. Enable torque for ID 6 only and abort if present jumps more than {} ticks".format(DEFAULT_JUMP_LIMIT))
    print("  6. Require exact typed confirmation after target is known")
    print("  7. Write ID 6 target, poll for at least {:.1f}s unless target is reached earlier".format(DEFAULT_MIN_POLL_SECONDS))
    print("  8. Require observed motion in the requested direction by at least {} ticks".format(min_observed_delta))
    print("  9. Require final within {} ticks of target or stopped near target".format(tolerance))
    print("  10. Disable torque for ID 6 after the test")
    print("  11. With torque disabled, align final Goal_Position to final Present_Position")
    print("Write steps: align_goal_to_present, enable_torque, command_target, disable_torque, align_final_goal_to_present")
    print("Typed confirmation template: MOVE gripper <target>")
    print("No hardware writes performed.")
    print("Log: {}".format(log_path))


def _validate_initial_state(registers, calibration, target, delta, max_delta):
    active_min = calibration["active_min"]
    active_max = calibration["active_max"]
    if registers["mode"] != 0:
        raise GripperMotionError("Refusing: ID 6 operating mode is {}, expected 0.".format(registers["mode"]))
    if not _inside(registers["present"], active_min, active_max):
        raise GripperMotionError(
            "Refusing: present position {} outside active gripper limits {}.".format(
                registers["present"], _format_range(active_min, active_max)
            )
        )
    if not _inside(target, active_min, active_max):
        raise GripperMotionError(
            "Refusing: target {} outside active gripper limits {}.".format(
                target, _format_range(active_min, active_max)
            )
        )
    if delta == 0:
        raise GripperMotionError("Refusing: delta 0 cannot prove physical movement.")
    if abs(delta) > max_delta:
        raise GripperMotionError(
            "Refusing: abs(delta) {} exceeds max_delta {}.".format(abs(delta), max_delta)
        )
    if registers["eeprom_min"] > active_min or registers["eeprom_max"] < active_max:
        raise GripperMotionError(
            "Refusing: active limits {} are not inside EEPROM limits {}.".format(
                _format_range(active_min, active_max),
                _format_range(registers["eeprom_min"], registers["eeprom_max"]),
            )
        )


def _poll_until_stopped(bus, target, timeout_seconds, poll_interval, tolerance, start_present):
    samples = []
    start_time = time.time()
    deadline = time.time() + float(timeout_seconds)
    min_poll_deadline = start_time + DEFAULT_MIN_POLL_SECONDS
    stable_count = 0
    previous = None
    final_present = None
    final_moving = None
    observed_motion_or_reached = False

    while time.time() <= deadline:
        present = _read_required_register(bus, "present position", REG_PRESENT_POSITION)
        moving = bus.read_register(GRIPPER_SERVO_ID, REG_MOVING[0], REG_MOVING[1])
        hardware_error = bus.read_register(
            GRIPPER_SERVO_ID, REG_HARDWARE_ERROR_STATUS[0], REG_HARDWARE_ERROR_STATUS[1]
        )
        final_present = present
        final_moving = moving
        samples.append({
            "present": present,
            "moving": moving,
            "hardware_error": hardware_error,
        })
        _log(
            bus, "poll_present_position", "ok",
            present_position=present,
            moving=moving,
            hardware_error=hardware_error,
        )
        if moving not in (None, 0):
            observed_motion_or_reached = True
        if abs(int(present) - int(start_present)) > 1:
            observed_motion_or_reached = True
        reached_target = abs(int(present) - int(target)) <= int(tolerance)
        if reached_target:
            observed_motion_or_reached = True

        if previous is not None and abs(present - previous) <= 1:
            stable_count += 1
        else:
            stable_count = 0
        previous = present

        min_poll_elapsed = time.time() >= min_poll_deadline
        if reached_target:
            break
        if moving == 0 and len(samples) >= 2 and min_poll_elapsed:
            break
        if stable_count >= 2 and min_poll_elapsed:
            break
        time.sleep(float(poll_interval))

    final_hardware_error = None
    if samples:
        final_hardware_error = samples[-1].get("hardware_error")
    return {
        "samples": samples,
        "final_present": final_present,
        "final_moving": final_moving,
        "final_hardware_error": final_hardware_error,
        "timed_out": time.time() > deadline,
        "observed_motion_or_reached": observed_motion_or_reached,
        "poll_seconds": time.time() - start_time,
    }


def _run_real(args, config, calibration):
    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=False,
        backend_name="feetech",
    )
    disable_torque_after_test = False
    try:
        _log(bus, "start", "ok", delta=int(args.delta), max_delta=int(args.max_delta))
        registers = {
            "present": _read_required_register(bus, "present position", REG_PRESENT_POSITION),
            "goal": _read_required_register(bus, "goal position", REG_GOAL_POSITION),
            "torque": _read_required_register(bus, "torque state", REG_TORQUE_ENABLE),
            "mode": _read_required_register(bus, "operating mode", REG_OPERATING_MODE),
            "eeprom_min": _read_required_register(bus, "EEPROM min angle", REG_MIN_ANGLE_LIMIT),
            "eeprom_max": _read_required_register(bus, "EEPROM max angle", REG_MAX_ANGLE_LIMIT),
        }
        delta = int(args.delta)
        max_delta = int(args.max_delta)
        min_observed_delta = int(args.min_observed_delta)
        tolerance = int(args.tolerance)
        initial_present = int(registers["present"])
        target = initial_present + delta
        _log(
            bus, "initial_read", "ok",
            initial_present=initial_present,
            target=target,
            min_observed_delta=min_observed_delta,
            tolerance=tolerance,
            **registers
        )

        _validate_initial_state(registers, calibration, target, delta, max_delta)
        _log(bus, "initial_validation", "ok", target=target)

        print("Mode: REAL")
        print("Servo ID: {} ({})".format(GRIPPER_SERVO_ID, GRIPPER_JOINT))
        print("Present position: {}".format(registers["present"]))
        print("Initial present: {}".format(initial_present))
        print("Goal position: {}".format(registers["goal"]))
        print("Torque state: {} ({})".format(registers["torque"], "enabled" if registers["torque"] else "disabled"))
        print("Operating mode: {}".format(registers["mode"]))
        print("EEPROM limits: {}".format(_format_range(registers["eeprom_min"], registers["eeprom_max"])))
        print("Active software limits: {}".format(_format_range(calibration["active_min"], calibration["active_max"])))
        print("Delta: {}".format(delta))
        print("Max delta: {}".format(max_delta))
        print("Min observed delta: {}".format(min_observed_delta))
        print("Tolerance: {}".format(tolerance))
        print("Target: {}".format(target))

        if int(registers["goal"]) == int(registers["present"]):
            print("goal already aligned")
            _log(bus, "align_goal_to_present", "skipped", goal_position=registers["present"])
        else:
            _write_goal_with_readback(
                bus, "align_goal_to_present", registers["present"],
                allow_verified_continue=True,
            )

        if int(registers["torque"]) == 1:
            print("torque already enabled")
            disable_torque_after_test = True
            _log(bus, "enable_torque", "skipped", enabled=True)
        else:
            _write_torque_with_readback(
                bus, "enable_torque", True,
                allow_verified_continue=True,
            )
            disable_torque_after_test = True

        post_enable_present = _read_required_register(bus, "post-enable present position", REG_PRESENT_POSITION)
        jump = abs(int(post_enable_present) - int(registers["present"]))
        _log(bus, "post_torque_present_read", "ok", present_position=post_enable_present, jump=jump)
        if jump > DEFAULT_JUMP_LIMIT:
            raise GripperMotionError(
                "Aborting: present position jumped {} ticks after torque enable.".format(jump)
            )

        expected_confirmation = "MOVE gripper {}".format(target)
        print("Typed confirmation required: {}".format(expected_confirmation))
        confirmation = input("> ")
        if confirmation != expected_confirmation:
            _log(bus, "confirmation", "refused", expected=expected_confirmation, provided=confirmation)
            raise GripperMotionError("Typed confirmation did not match exactly.")
        _log(bus, "confirmation", "ok", expected=expected_confirmation)

        target_write = _write_target_goal(bus, target)

        poll_result = _poll_until_stopped(
            bus=bus,
            target=target,
            timeout_seconds=args.timeout,
            poll_interval=args.poll_interval,
            tolerance=tolerance,
            start_present=initial_present,
        )
        final_present = poll_result.get("final_present")
        if final_present is None:
            raise GripperMotionError("No final present-position read was available.")
        final_error = abs(int(final_present) - int(target))
        observed_delta = int(final_present) - int(initial_present)
        movement_in_direction = _movement_in_requested_direction(
            observed_delta, delta, min_observed_delta
        )
        final_inside_limits = _inside(
            final_present, calibration["active_min"], calibration["active_max"]
        )
        final_moving = poll_result.get("final_moving")
        moving_stopped = final_moving == 0
        final_hardware_error = poll_result.get("final_hardware_error")
        hardware_error_zero = final_hardware_error == 0
        final_within_tolerance = final_error <= tolerance
        near_target_error_limit = max(tolerance, min_observed_delta)
        stopped_near_target = (
            movement_in_direction
            and moving_stopped
            and final_error <= near_target_error_limit
        )
        success = bool(
            (final_within_tolerance or stopped_near_target)
            and movement_in_direction
            and final_inside_limits
            and moving_stopped
            and hardware_error_zero
        )
        _log(
            bus,
            "final_validation",
            "ok" if success else "refused",
            initial_present=initial_present,
            target=target,
            final_present=final_present,
            final_error=final_error,
            observed_delta=observed_delta,
            min_observed_delta=min_observed_delta,
            tolerance=tolerance,
            near_target_error_limit=near_target_error_limit,
            movement_in_direction=movement_in_direction,
            final_inside_limits=final_inside_limits,
            final_moving=final_moving,
            final_hardware_error=final_hardware_error,
            timed_out=bool(poll_result.get("timed_out")),
            poll_seconds=poll_result.get("poll_seconds"),
            command_target_missing_status=bool(target_write.get("missing_status")),
            command_target_readback_verified=bool(target_write.get("readback_verified")),
            command_target_readback_value=target_write.get("readback_value"),
            observed_motion_or_reached=bool(poll_result.get("observed_motion_or_reached")),
        )
        print("Final present position: {}".format(final_present))
        print("Final target error: {} ticks".format(final_error))
        print("Observed delta: {} ticks".format(observed_delta))
        print("Final moving flag: {}".format(final_moving))
        print("Final hardware error: {}".format(final_hardware_error))
        print("Samples: {}".format(poll_result.get("samples")))
        if target_write.get("missing_status") and not movement_in_direction:
            raise GripperMotionError(
                "Motion did not verify: command_target status was missing and observed motion was not in the requested direction."
            )
        if not movement_in_direction:
            print("Result: no observed motion")
            raise GripperMotionError(
                "No observed motion: observed_delta {} did not satisfy requested delta {} and min_observed_delta {}.".format(
                    observed_delta, delta, min_observed_delta
                )
            )
        if not success:
            if not final_inside_limits:
                raise GripperMotionError(
                    "Motion did not verify: final present {} is outside active gripper limits {}.".format(
                        final_present,
                        _format_range(calibration["active_min"], calibration["active_max"]),
                    )
                )
            if not moving_stopped:
                raise GripperMotionError(
                    "Motion did not verify: moving flag is not stopped: {}.".format(final_moving)
                )
            if not hardware_error_zero:
                raise GripperMotionError(
                    "Motion did not verify: hardware error status is {}.".format(final_hardware_error)
                )
            raise GripperMotionError(
                "Motion did not verify: final present {} is more than {} ticks from target {}.".format(
                    final_present, tolerance, target
                )
            )
        print("Result: success")
        print("Log: {}".format(bus.logger.path))
    finally:
        torque_disabled = False
        if disable_torque_after_test:
            try:
                _write_torque_with_readback(
                    bus, "disable_torque", False,
                    allow_verified_continue=True,
                )
                torque_disabled = True
                print("Torque disabled for ID 6.")
            except Exception as exc:
                _log_write_result(
                    bus, "disable_torque", "Torque_Enable", 0,
                    status_response_ok=False,
                    readback_verified=False,
                    readback_value=None,
                    continued_after_missing_status=False,
                    status="error",
                    error=str(exc),
                )
                print("WARNING: failed to disable torque for ID 6: {}".format(exc))
        if torque_disabled:
            try:
                _align_final_goal_to_present_with_torque_disabled(bus)
            except Exception as exc:
                _log(
                    bus,
                    "align_final_goal_to_present",
                    "error",
                    error=str(exc),
                )
                print("WARNING: failed to align final goal to present: {}".format(exc))
        bus.close()


def main():
    args = build_parser().parse_args()
    if int(args.max_delta) <= 0:
        raise SystemExit("ERROR: --max-delta must be positive.")
    if float(args.timeout) <= 0:
        raise SystemExit("ERROR: --timeout must be positive.")
    if float(args.poll_interval) <= 0:
        raise SystemExit("ERROR: --poll-interval must be positive.")
    if int(args.min_observed_delta) <= 0:
        raise SystemExit("ERROR: --min-observed-delta must be positive.")
    if int(args.tolerance) < 0:
        raise SystemExit("ERROR: --tolerance must be non-negative.")

    config = load_robot_config(args.config)
    calibration = _load_gripper_calibration()

    if not args.real:
        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=True,
            backend_name="mock",
            mock_ids=[GRIPPER_SERVO_ID],
        )
        try:
            _log(
                bus, "dry_run_plan", "ok",
                delta=int(args.delta),
                max_delta=int(args.max_delta),
                min_observed_delta=int(args.min_observed_delta),
                tolerance=int(args.tolerance),
            )
            _print_dry_run_plan(args, calibration, bus.logger.path)
        finally:
            bus.close()
        return

    try:
        _run_real(args, config, calibration)
    except (BackendUnavailable, ServoBusError, GripperMotionError, OSError, ValueError) as exc:
        print("ERROR: {}".format(exc))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
