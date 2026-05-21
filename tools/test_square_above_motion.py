#!/usr/bin/env python3
from __future__ import absolute_import, print_function

import argparse
import json
import math
import os
import shlex
import sys
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map  # noqa: E402
from chess_robot.robot import safety  # noqa: E402
from chess_robot.robot.servo_bus import (  # noqa: E402
    BackendUnavailable,
    ServoBusError,
    build_servo_bus,
    load_robot_config,
)

DEFAULT_ROBOT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
DEFAULT_LOG_PATH = os.path.join(ROOT, "data", "logs", "square_above_motion.log")
EXPECTED_CONFIRM_TEXT = "MOVE ABOVE SQUARES"
REG_HARDWARE_ERROR_STATUS = (65, 1)
MANUAL_ANCHOR_ORDER = [
    "a1", "c1", "f1", "h1",
    "h3", "f3", "c3", "a3",
    "a6", "c6", "f6", "h6",
    "h8", "f8", "c8", "a8",
]


class SquareAboveMotionError(RuntimeError):
    """Raised when above-square playback should refuse to continue."""


class SquareAboveMotionAbort(RuntimeError):
    """Raised when the operator aborts the sequence cleanly."""


def _utc_timestamp():
    return datetime.utcnow().isoformat() + "Z"


def _ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def build_all_squares_path():
    squares = []
    files = list("abcdefgh")
    for rank in range(1, 9):
        rank_files = list(files)
        if rank % 2 == 0:
            rank_files.reverse()
        for file_name in rank_files:
            squares.append("{}{}".format(file_name, rank))
    return squares


def build_corner_loop_path():
    return ["a1", "h1", "h8", "a8", "a1"]


def build_manual_anchor_path(document):
    doc = robot_square_map.normalise_square_targets_document(document)
    squares = []
    warnings = []
    for square_name in MANUAL_ANCHOR_ORDER:
        square_info = doc.get("squares", {}).get(square_name) or {}
        above_pose = square_info.get("above_pose") if isinstance(square_info, dict) else None
        source = above_pose.get("source") if isinstance(above_pose, dict) else None
        if source == "manual":
            squares.append(square_name)
        else:
            warnings.append("Preferred manual anchor missing or not manual: {}".format(square_name))
    return squares, warnings


def parse_explicit_square_path(raw):
    if raw is None:
        return None
    values = []
    for chunk in str(raw).split(","):
        item = chunk.strip()
        if not item:
            continue
        values.append(robot_square_map.normalise_square_name(item))
    if not values:
        raise SquareAboveMotionError("--squares did not contain any square names.")
    return values


def apply_square_filters(squares, start_square=None, end_square=None, max_squares=None):
    result = list(squares)
    if start_square:
        start_name = robot_square_map.normalise_square_name(start_square)
        if start_name not in result:
            raise SquareAboveMotionError("start square {} is not in the selected path.".format(start_name))
        start_index = result.index(start_name)
        result = result[start_index:]
    if end_square:
        end_name = robot_square_map.normalise_square_name(end_square)
        if end_name not in result:
            raise SquareAboveMotionError("end square {} is not in the selected path after start filtering.".format(end_name))
        end_index = result.index(end_name)
        result = result[:end_index + 1]
    if max_squares is not None:
        limit = int(max_squares)
        if limit <= 0:
            raise SquareAboveMotionError("--max-squares must be a positive integer.")
        result = result[:limit]
    if not result:
        raise SquareAboveMotionError("Selected path is empty after filters were applied.")
    return result


def resolve_requested_squares(document, path_name, squares_arg=None, start_square=None, end_square=None, max_squares=None):
    warnings = []
    explicit = parse_explicit_square_path(squares_arg)
    if explicit is not None:
        base_squares = explicit
        resolved_path_name = "explicit-squares"
    elif path_name == "corner-loop":
        base_squares = build_corner_loop_path()
        resolved_path_name = path_name
    elif path_name == "manual-anchors":
        base_squares, warnings = build_manual_anchor_path(document)
        resolved_path_name = path_name
    elif path_name == "all-squares":
        base_squares = build_all_squares_path()
        resolved_path_name = path_name
    else:
        raise SquareAboveMotionError("Unknown path {!r}.".format(path_name))
    filtered = apply_square_filters(
        base_squares,
        start_square=start_square,
        end_square=end_square,
        max_squares=max_squares,
    )
    return resolved_path_name, filtered, warnings


def _movement_joints(document, include_gripper):
    joint_order = list(document.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)
    result = []
    for joint_name in joint_order:
        normalized = safety.normalize_joint_name(joint_name)
        if normalized == "gripper" and not include_gripper:
            continue
        result.append(normalized)
    if not result:
        raise SquareAboveMotionError("No movement joints are available for playback.")
    return joint_order, result


def _servo_ids_by_joint(servo_map, movement_joints):
    joints = servo_map.get("joints") or {}
    ids_by_joint = {}
    for joint_name in movement_joints:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise SquareAboveMotionError("servo_map is missing joint {}.".format(joint_name))
        if "id" not in entry:
            raise SquareAboveMotionError("servo_map joint {} is missing id.".format(joint_name))
        ids_by_joint[joint_name] = safety.validate_servo_id(entry.get("id"))
    return ids_by_joint


def _validate_joint_value(joint_name, value, joint_limits, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise SquareAboveMotionError("{} joint {} must be an integer tick value.".format(context, joint_name))
    limits = safety.resolve_joint_limits(joint_limits, joint_name)
    if limits is None:
        raise SquareAboveMotionError("No joint limits are configured for {}.".format(joint_name))
    minimum = int(limits["min"])
    maximum = int(limits["max"])
    if value < minimum or value > maximum:
        raise SquareAboveMotionError(
            "{} joint {} target {} is outside software limits {}..{}.".format(
                context, joint_name, value, minimum, maximum
            )
        )
    return int(value)


def select_square_targets(document, joint_limits, servo_map, requested_squares, include_gripper=False):
    doc = robot_square_map.normalise_square_targets_document(document)
    joint_order, movement_joints = _movement_joints(doc, include_gripper)
    known_joint_names = set([safety.normalize_joint_name(name) for name in joint_order])
    ids_by_joint = _servo_ids_by_joint(servo_map, movement_joints)
    squares_data = doc.get("squares") or {}
    selected = []
    for square_name in requested_squares:
        normalized_square = robot_square_map.normalise_square_name(square_name)
        square_info = squares_data.get(normalized_square)
        if not isinstance(square_info, dict):
            raise SquareAboveMotionError("square {} is missing from square_targets.yaml.".format(normalized_square))
        above_pose = square_info.get("above_pose")
        if not isinstance(above_pose, dict):
            raise SquareAboveMotionError("square {} is missing above_pose.".format(normalized_square))
        joints = above_pose.get("joints")
        if not isinstance(joints, dict):
            raise SquareAboveMotionError("square {} above_pose.joints must be a mapping.".format(normalized_square))
        unknown = sorted([name for name in joints.keys() if safety.normalize_joint_name(name) not in known_joint_names])
        if unknown:
            raise SquareAboveMotionError(
                "square {} includes unknown joints: {}".format(normalized_square, ", ".join(unknown))
            )
        target_joints = {}
        for joint_name in movement_joints:
            if joint_name not in joints:
                raise SquareAboveMotionError(
                    "square {} is missing required joint {} in above_pose.".format(normalized_square, joint_name)
                )
            target_joints[joint_name] = _validate_joint_value(
                joint_name,
                joints.get(joint_name),
                joint_limits,
                "square {}".format(normalized_square),
            )
        selected.append({
            "square": normalized_square,
            "source": above_pose.get("source") or "unknown",
            "target_joints": target_joints,
            "raw_joints": dict(joints),
        })
    return {
        "movement_joints": movement_joints,
        "ids_by_joint": ids_by_joint,
        "selected": selected,
    }


def build_intermediate_poses(current_joints, target_joints, step_size_ticks):
    step_size = int(step_size_ticks)
    if step_size <= 0:
        raise SquareAboveMotionError("--step-size-ticks must be a positive integer.")
    current_keys = sorted(current_joints.keys())
    target_keys = sorted(target_joints.keys())
    if current_keys != target_keys:
        raise SquareAboveMotionError("Current and target joint sets do not match.")
    max_delta = 0
    for joint_name in current_keys:
        delta = abs(int(target_joints[joint_name]) - int(current_joints[joint_name]))
        if delta > max_delta:
            max_delta = delta
    if max_delta == 0:
        return [dict((name, int(target_joints[name])) for name in current_keys)]
    step_count = int(math.ceil(float(max_delta) / float(step_size)))
    poses = []
    for step_index in range(1, step_count + 1):
        pose = {}
        for joint_name in current_keys:
            start_value = int(current_joints[joint_name])
            target_value = int(target_joints[joint_name])
            delta = target_value - start_value
            interpolated = start_value + int(round((float(delta) * float(step_index)) / float(step_count)))
            pose[joint_name] = int(interpolated)
        poses.append(pose)
    return poses


def validate_pose_within_limits(pose, joint_limits, context):
    for joint_name, value in pose.items():
        _validate_joint_value(joint_name, int(value), joint_limits, context)


def validate_step_sequence(current_joints, poses, joint_limits, step_size_ticks):
    previous = dict(current_joints)
    step_size = int(step_size_ticks)
    for index, pose in enumerate(poses):
        validate_pose_within_limits(pose, joint_limits, "intermediate step {}".format(index + 1))
        for joint_name, value in pose.items():
            delta = abs(int(value) - int(previous[joint_name]))
            if delta > step_size:
                raise SquareAboveMotionError(
                    "Intermediate step {} joint {} delta {} exceeds step-size {}.".format(
                        index + 1, joint_name, delta, step_size
                    )
                )
        previous = pose


def estimate_step_count(current_joints, target_joints, step_size_ticks):
    return len(build_intermediate_poses(current_joints, target_joints, step_size_ticks))


def _command_hint(args):
    command = [
        "python3",
        "tools/test_square_above_motion.py",
        "--targets", args.targets,
        "--joint-limits", args.joint_limits,
        "--servo-map", args.servo_map,
        "--robot-config", args.robot_config,
    ]
    if args.squares:
        command.extend(["--squares", args.squares])
    else:
        command.extend(["--path", args.path])
    if args.max_squares is not None:
        command.extend(["--max-squares", str(args.max_squares)])
    if args.start_square:
        command.extend(["--start-square", args.start_square])
    if args.end_square:
        command.extend(["--end-square", args.end_square])
    command.extend([
        "--real",
        "--confirm-text", EXPECTED_CONFIRM_TEXT,
        "--step-size-ticks", str(args.step_size_ticks),
        "--step-delay", str(args.step_delay),
        "--settle-time", str(args.settle_time),
    ])
    if args.pause_each:
        command.append("--pause-each")
    if args.include_gripper:
        command.append("--include-gripper")
    if args.log:
        command.extend(["--log", args.log])
    if args.output_json:
        command.extend(["--output-json", args.output_json])
    return " ".join([shlex.quote(part) for part in command])


def append_text_log(path, payload):
    _ensure_parent_dir(path)
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def write_json_result(path, result):
    _ensure_parent_dir(path)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")

def build_result(path_name, requested_squares, movement_joints, include_gripper, args):
    return {
        "started_at": _utc_timestamp(),
        "completed_at": None,
        "dry_run": not bool(args.real),
        "real": bool(args.real),
        "path_name": path_name,
        "requested_squares": list(requested_squares),
        "movement_joints": list(movement_joints),
        "include_gripper": bool(include_gripper),
        "step_size_ticks": int(args.step_size_ticks),
        "step_delay": float(args.step_delay),
        "settle_time": float(args.settle_time),
        "per_square_results": [],
        "aborted": False,
        "abort_reason": None,
        "final_torque_disable_attempted": False,
        "final_torque_disable_success": None,
    }


def mark_aborted(result, reason):
    result["aborted"] = True
    result["abort_reason"] = str(reason)
    return result


def _resolve_real_backend_name(config):
    servo_config = config.get("servo_bus") or {}
    configured_backend = servo_config.get("backend")
    has_feetech = bool(servo_config.get("feetech"))
    if configured_backend == "mock" and has_feetech:
        return "feetech"
    if configured_backend:
        return configured_backend
    if has_feetech:
        return "feetech"
    return "mock"


def _validate_current_positions(current_positions, joint_limits):
    for joint_name, value in current_positions.items():
        _validate_joint_value(joint_name, int(value), joint_limits, "current position")


def _read_joint_positions(bus, ids_by_joint):
    positions = {}
    for joint_name in ids_by_joint:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_position(servo_id)
        if value is None:
            raise SquareAboveMotionError(
                "Present position could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        positions[joint_name] = int(value)
    return positions


def _read_hardware_errors(bus, ids_by_joint):
    errors = {}
    address, length = REG_HARDWARE_ERROR_STATUS
    for joint_name in ids_by_joint:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_register(servo_id, address, length)
        if value is None:
            raise SquareAboveMotionError(
                "Hardware error register could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        errors[joint_name] = int(value)
    return errors


def _enable_torque(bus, ids_by_joint):
    for joint_name in ids_by_joint:
        servo_id = ids_by_joint[joint_name]
        bus.torque_enable(servo_id, True)


def _disable_torque(bus, ids_by_joint):
    success = True
    for joint_name in ids_by_joint:
        servo_id = ids_by_joint[joint_name]
        try:
            bus.torque_enable(servo_id, False)
        except Exception:
            success = False
    return success


def _write_pose(bus, ids_by_joint, movement_joints, pose):
    for joint_name in movement_joints:
        bus.write_goal_position(ids_by_joint[joint_name], int(pose[joint_name]))


def _print_operator_instructions(path_name, movement_joints):
    print("Mode: REAL")
    print("Safety warning:")
    print("  - empty board")
    print("  - remove chess pieces")
    print("  - keep hand near power switch")
    print("  - watch for cable snag / board collision / servo strain")
    print("  - press Ctrl+C to abort")
    print("Selected path: {}".format(path_name))
    print("Movement joints: {}".format(", ".join(movement_joints)))
    print("Typed confirmation required: {}".format(EXPECTED_CONFIRM_TEXT))


def _print_dry_run_summary(path_name, plan, warnings, args):
    print("Mode: DRY-RUN")
    print("Selected path: {}".format(path_name))
    print("Movement joints: {}".format(", ".join(plan["movement_joints"])))
    print("Square count: {}".format(len(plan["selected"])))
    if warnings:
        print("Warnings:")
        for warning in warnings:
            print("  - {}".format(warning))
    previous_target = None
    for item in plan["selected"]:
        target_joints = item["target_joints"]
        estimated_steps = None
        if previous_target is not None:
            estimated_steps = estimate_step_count(previous_target, target_joints, args.step_size_ticks)
        print("{}: source={} target_joints={} estimated_steps={}".format(
            item["square"],
            item["source"],
            ", ".join(["{}={}".format(name, target_joints[name]) for name in plan["movement_joints"]]),
            estimated_steps if estimated_steps is not None else "live-read-required",
        ))
        previous_target = target_joints
    print("Validation summary: OK")
    print("Real command (not executed):")
    print(_command_hint(args))


def _record_square_result(result, args, path_name, square_result):
    result["per_square_results"].append(square_result)
    if args.log:
        payload = dict(square_result)
        payload["timestamp"] = _utc_timestamp()
        payload["path_name"] = path_name
        append_text_log(args.log, payload)


def _build_square_result_template(path_name, item):
    return {
        "timestamp": _utc_timestamp(),
        "path_name": path_name,
        "square": item["square"],
        "source": item["source"],
        "target_joints": dict(item["target_joints"]),
        "current_joints_before": None,
        "step_count": None,
        "status": None,
        "observed_joints_after": None,
        "final_error": None,
        "warnings": [],
        "exception": None,
    }


def run(args, bus_factory=build_servo_bus, config_loader=load_robot_config, sleep_fn=time.sleep, pause_input_fn=input):
    if float(args.step_delay) < 0.0:
        raise SquareAboveMotionError("--step-delay must be >= 0.")
    if float(args.settle_time) < 0.0:
        raise SquareAboveMotionError("--settle-time must be >= 0.")
    if float(args.pause_seconds) < 0.0:
        raise SquareAboveMotionError("--pause-seconds must be >= 0.")

    document = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits)
    servo_map = robot_square_map.load_servo_map(args.servo_map)
    path_name, requested_squares, warnings = resolve_requested_squares(
        document,
        args.path,
        squares_arg=args.squares,
        start_square=args.start_square,
        end_square=args.end_square,
        max_squares=args.max_squares,
    )
    plan = select_square_targets(
        document,
        joint_limits,
        servo_map,
        requested_squares,
        include_gripper=bool(args.include_gripper),
    )
    result = build_result(
        path_name,
        requested_squares,
        plan["movement_joints"],
        bool(args.include_gripper),
        args,
    )

    if warnings and bool(args.stop_on_warning):
        raise SquareAboveMotionError(
            "Warnings were detected and stop-on-warning is enabled: {}".format("; ".join(warnings))
        )

    if not args.real:
        previous_target = None
        for item in plan["selected"]:
            square_result = _build_square_result_template(path_name, item)
            if previous_target is not None:
                square_result["step_count"] = estimate_step_count(previous_target, item["target_joints"], args.step_size_ticks)
            square_result["status"] = "dry_run"
            _record_square_result(result, args, path_name, square_result)
            previous_target = item["target_joints"]
        _print_dry_run_summary(path_name, plan, warnings, args)
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        return 0, result

    config = config_loader(args.robot_config)
    configured_joints = config.get("joints") or {}
    safety.validate_multi_joint_commanded_joints(
        configured_joints=configured_joints,
        commanded_joints=plan["movement_joints"],
        include_gripper=bool(args.include_gripper),
    )
    safety.require_real_above_square_confirmation(args.confirm_text)

    bus = None
    current_square_result = None
    try:
        _print_operator_instructions(path_name, plan["movement_joints"])
        backend_name = _resolve_real_backend_name(config)
        bus = bus_factory(
            config=config,
            config_path=args.robot_config,
            dry_run=False,
            backend_name=backend_name,
            mock_ids=None,
        )
        current_positions = _read_joint_positions(bus, plan["ids_by_joint"])
        _validate_current_positions(current_positions, joint_limits)
        _read_hardware_errors(bus, plan["ids_by_joint"])
        _enable_torque(bus, plan["ids_by_joint"])
        for index, item in enumerate(plan["selected"]):
            print("Square {} / {}: {}".format(index + 1, len(plan["selected"]), item["square"]))
            current_square_result = _build_square_result_template(path_name, item)
            current_square_result["current_joints_before"] = current_positions
            steps = build_intermediate_poses(current_positions, item["target_joints"], args.step_size_ticks)
            validate_step_sequence(current_positions, steps, joint_limits, args.step_size_ticks)
            current_square_result["step_count"] = len(steps)
            for pose in steps:
                _write_pose(bus, plan["ids_by_joint"], plan["movement_joints"], pose)
                if float(args.step_delay) > 0.0:
                    sleep_fn(float(args.step_delay))
            if float(args.settle_time) > 0.0:
                sleep_fn(float(args.settle_time))
            observed = _read_joint_positions(bus, plan["ids_by_joint"])
            _validate_current_positions(observed, joint_limits)
            hardware_errors = _read_hardware_errors(bus, plan["ids_by_joint"])
            bad_hardware = dict((joint_name, code) for joint_name, code in hardware_errors.items() if int(code) != 0)
            if bad_hardware:
                raise SquareAboveMotionError("Hardware error register is non-zero: {}".format(bad_hardware))
            final_error = {}
            for joint_name in plan["movement_joints"]:
                final_error[joint_name] = int(observed[joint_name]) - int(item["target_joints"][joint_name])
            current_square_result["observed_joints_after"] = observed
            current_square_result["final_error"] = final_error
            current_square_result["status"] = "ok"
            _record_square_result(result, args, path_name, current_square_result)
            print("Readback: {}".format(", ".join(["{}={}".format(name, observed[name]) for name in plan["movement_joints"]])))
            current_square_result = None
            current_positions = observed
            if index + 1 < len(plan["selected"]):
                if args.pause_each:
                    print("Press Enter for next square, or type q to abort.")
                    response = pause_input_fn("")
                    if str(response).strip().lower() == "q":
                        raise SquareAboveMotionAbort("operator requested abort during pause")
                elif float(args.pause_seconds) > 0.0:
                    sleep_fn(float(args.pause_seconds))
    except KeyboardInterrupt:
        if current_square_result is not None and current_square_result.get("status") is None:
            current_square_result["status"] = "aborted"
            current_square_result["exception"] = "KeyboardInterrupt"
            _record_square_result(result, args, path_name, current_square_result)
        mark_aborted(result, "keyboard_interrupt")
        return 1, result
    except SquareAboveMotionAbort as exc:
        if current_square_result is not None and current_square_result.get("status") is None:
            current_square_result["status"] = "aborted"
            current_square_result["exception"] = str(exc)
            _record_square_result(result, args, path_name, current_square_result)
        mark_aborted(result, str(exc))
        return 1, result
    except Exception as exc:
        if current_square_result is not None and current_square_result.get("status") is None:
            current_square_result["status"] = "failed"
            current_square_result["exception"] = str(exc)
            _record_square_result(result, args, path_name, current_square_result)
        raise
    finally:
        if bus is not None:
            result["final_torque_disable_attempted"] = True
            result["final_torque_disable_success"] = _disable_torque(bus, plan["ids_by_joint"])
            bus.close()
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
    return 0, result


def build_parser():
    parser = argparse.ArgumentParser(
        description="Safely preview or play back above-square joint targets from square_targets.yaml."
    )
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits", default=DEFAULT_JOINT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--servo-map", dest="servo_map", default=DEFAULT_SERVO_MAP_PATH,
                        help="Servo-map YAML path.")
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG_PATH,
                        help="Robot config YAML path. Default: configs/robot.yaml")
    parser.add_argument("--path", choices=("corner-loop", "manual-anchors", "all-squares"),
                        default="corner-loop", help="Named square path. Default: corner-loop")
    parser.add_argument("--squares", default=None,
                        help="Comma-separated explicit square path, for example a1,h1,h8,a8,a1")
    parser.add_argument("--real", action="store_true", help="Allow real movement after exact confirmation.")
    parser.add_argument("--confirm-text", default=None,
                        help="Exact real-mode confirmation text. Must be MOVE ABOVE SQUARES.")
    parser.add_argument("--pause-each", action="store_true",
                        help="Pause after each square and wait for Enter or q.")
    parser.add_argument("--pause-seconds", type=float, default=1.0,
                        help="Automatic pause between squares when --pause-each is not used. Default: 1.0")
    parser.add_argument("--step-size-ticks", type=int, default=20,
                        help="Maximum per-joint tick change per intermediate step. Default: 20")
    parser.add_argument("--step-delay", type=float, default=0.05,
                        help="Delay between intermediate steps in real mode. Default: 0.05")
    parser.add_argument("--settle-time", type=float, default=0.5,
                        help="Delay after each target pose in real mode. Default: 0.5")
    parser.add_argument("--include-gripper", action="store_true",
                        help="Include gripper target ticks from square_targets.yaml. Default: disabled")
    parser.add_argument("--stop-on-warning", action="store_true", default=True,
                        help="Stop when path-selection warnings are detected. Default: true")
    parser.add_argument("--log", default=None, help="Optional append-only JSONL text log path.")
    parser.add_argument("--output-json", default=None, help="Optional JSON result output path.")
    parser.add_argument("--max-squares", type=int, default=None,
                        help="Optional maximum number of selected squares to run.")
    parser.add_argument("--start-square", default=None,
                        help="Optional inclusive starting square inside the selected path.")
    parser.add_argument("--end-square", default=None,
                        help="Optional inclusive ending square inside the selected path.")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        exit_code, result = run(args)
    except (BackendUnavailable, ServoBusError, SquareAboveMotionError, ValueError, OSError) as exc:
        print("ERROR: {}".format(exc))
        if args.output_json and os.path.exists(args.output_json):
            pass
        raise SystemExit(1)
    print("Completed. Aborted: {}".format("yes" if result.get("aborted") else "no"))
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
