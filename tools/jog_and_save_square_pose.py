#!/usr/bin/env python3
from __future__ import absolute_import, print_function

import argparse
import copy
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

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.calibration import robot_square_map  # noqa: E402
from chess_robot.robot import safety  # noqa: E402
from chess_robot.robot.servo_bus import (  # noqa: E402
    build_servo_bus,
    load_robot_config,
)

DEFAULT_ROBOT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
DEFAULT_GRIPPER_PROFILE_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "gripper_profile.yaml")
DEFAULT_LOG_PATH = os.path.join(ROOT, "data", "logs", "jog_and_save_square_pose.log")
DEFAULT_OUTPUT_JSON_PATH = os.path.join(ROOT, "data", "debug", "jog_and_save_square_pose.json")
DEFAULT_SQUARE = "c3"
DEFAULT_POSE_NAME = "above_pose"
DEFAULT_JOG_STEP = 5
DEFAULT_MAX_SINGLE_JOG = 10
DEFAULT_MAX_TOTAL_DELTA = 100
DEFAULT_STEP_SIZE_TICKS = 5
DEFAULT_STEP_DELAY = 0.12
DEFAULT_SETTLE_TIME = 0.75
REQUIRED_CONFIRM_TEXT = "POWERED JOG SAVE"
REG_HARDWARE_ERROR_STATUS = (65, 1)
DEFAULT_MOVEMENT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
SHORTHAND_JOINTS = {
    "pan": "shoulder_pan",
    "lift": "shoulder_lift",
    "elbow": "elbow_flex",
    "wrist": "wrist_flex",
    "roll": "wrist_roll",
    "gripper": "gripper",
}


class JogAndSaveSquarePoseError(RuntimeError):
    """Raised when powered jog/save validation or execution should refuse to continue."""


class JogAndSaveSquarePoseAbort(RuntimeError):
    """Raised when the operator aborts the interactive session cleanly."""


class SaveRejected(JogAndSaveSquarePoseError):
    """Raised when save policy rejects the requested overwrite."""


class CommandRejected(JogAndSaveSquarePoseError):
    """Raised when an operator command is invalid."""


class SessionState(object):
    def __init__(self, jog_step):
        self.jog_step = int(jog_step)
        self.initial_readback = None
        self.current_readback = None
        self.final_readback = None
        self.saved = False
        self.would_save = False
        self.abort_reason = None



def _utc_timestamp():
    return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"



def _ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)



def append_text_log(path, payload):
    if not path:
        return
    _ensure_parent_dir(path)
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")



def write_json_result(path, result):
    if not path:
        return False
    result["output_written"] = True
    _ensure_parent_dir(path)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return True



def _load_optional_yaml(path, default):
    if yaml is None:
        raise JogAndSaveSquarePoseError("PyYAML is required to read calibration data.")
    if not path or not os.path.exists(path):
        return copy.deepcopy(default)
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        return copy.deepcopy(default)
    return data



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



def _coerce_positive_int(value, label):
    try:
        normalized = int(value)
    except (TypeError, ValueError):
        raise JogAndSaveSquarePoseError("{} must be an integer.".format(label))
    if normalized <= 0:
        raise JogAndSaveSquarePoseError("{} must be > 0.".format(label))
    return normalized



def _coerce_nonnegative_float(value, label):
    try:
        normalized = float(value)
    except (TypeError, ValueError):
        raise JogAndSaveSquarePoseError("{} must be numeric.".format(label))
    if normalized < 0.0:
        raise JogAndSaveSquarePoseError("{} must be >= 0.".format(label))
    return normalized



def _movement_joints(allow_gripper_jog):
    joints = list(DEFAULT_MOVEMENT_JOINTS)
    if allow_gripper_jog:
        joints.append("gripper")
    return joints



def _servo_ids_by_joint(servo_map, joint_names):
    joints = servo_map.get("joints") or {}
    mapping = {}
    for joint_name in joint_names:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise JogAndSaveSquarePoseError("servo_map is missing joint {}.".format(joint_name))
        if "id" not in entry:
            raise JogAndSaveSquarePoseError("servo_map joint {} is missing id.".format(joint_name))
        mapping[joint_name] = safety.validate_servo_id(entry.get("id"))
    return mapping



def _validate_joint_value(joint_name, value, limits_by_joint, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise JogAndSaveSquarePoseError(
            "{} joint {} must be an integer tick value.".format(context, joint_name)
        )
    limits = safety.resolve_joint_limits(limits_by_joint, joint_name)
    if limits is None:
        raise JogAndSaveSquarePoseError("No joint limits are configured for {}.".format(joint_name))
    minimum = int(limits["min"])
    maximum = int(limits["max"])
    if int(value) < minimum or int(value) > maximum:
        raise JogAndSaveSquarePoseError(
            "{} joint {} target {} is outside software limits {}..{}.".format(
                context,
                joint_name,
                int(value),
                minimum,
                maximum,
            )
        )
    return int(value)



def _merged_gripper_limits(joint_limits, gripper_profile):
    base_limits = safety.resolve_joint_limits(joint_limits, "gripper")
    if base_limits is None:
        raise JogAndSaveSquarePoseError("No joint limits are configured for gripper.")
    minimum = int(base_limits["min"])
    maximum = int(base_limits["max"])
    profile_gripper = (gripper_profile or {}).get("gripper") or {}
    profile_limits = profile_gripper.get("limits") if isinstance(profile_gripper, dict) else None
    if isinstance(profile_limits, dict):
        if profile_limits.get("min") is not None:
            minimum = max(minimum, int(profile_limits.get("min")))
        if profile_limits.get("max") is not None:
            maximum = min(maximum, int(profile_limits.get("max")))
    if minimum > maximum:
        raise JogAndSaveSquarePoseError("Merged gripper limits are invalid.")
    return {"min": minimum, "max": maximum}



def _validate_pose_entry(square_name, pose_name, pose_entry, joint_order, joint_limits):
    context = "square {} {}".format(square_name, pose_name)
    if pose_entry is None:
        return None
    if not isinstance(pose_entry, dict):
        raise JogAndSaveSquarePoseError("{} is invalid.".format(context))
    joints = pose_entry.get("joints")
    if not isinstance(joints, dict):
        raise JogAndSaveSquarePoseError("{} must contain a joints mapping.".format(context))
    unknown = [name for name in sorted(joints.keys()) if name not in joint_order]
    if unknown:
        raise JogAndSaveSquarePoseError("{} includes unknown joints: {}".format(context, ", ".join(unknown)))
    normalized = {}
    for joint_name in joint_order:
        if joint_name not in joints:
            raise JogAndSaveSquarePoseError("{} is missing required joint {}.".format(context, joint_name))
        normalized[joint_name] = _validate_joint_value(joint_name, joints.get(joint_name), joint_limits, context)
    return {
        "source": pose_entry.get("source") or "unknown",
        "confidence": pose_entry.get("confidence"),
        "recorded_at": pose_entry.get("recorded_at"),
        "notes": list(pose_entry.get("notes") or []),
        "joints": normalized,
        "raw": dict(pose_entry),
    }



def build_intermediate_poses(current_joints, target_joints, step_size_ticks):
    step_size = _coerce_positive_int(step_size_ticks, "--step-size-ticks")
    current_keys = sorted(current_joints.keys())
    target_keys = sorted(target_joints.keys())
    if current_keys != target_keys:
        raise JogAndSaveSquarePoseError("Current and target joint sets do not match.")
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



def validate_step_sequence(current_joints, poses, joint_limits, step_size_ticks):
    previous = dict((name, int(value)) for name, value in current_joints.items())
    step_size = _coerce_positive_int(step_size_ticks, "--step-size-ticks")
    for index, pose in enumerate(poses):
        for joint_name in sorted(pose.keys()):
            position = _validate_joint_value(
                joint_name,
                int(pose[joint_name]),
                joint_limits,
                "intermediate step {}".format(index + 1),
            )
            delta = abs(int(position) - int(previous[joint_name]))
            if delta > step_size:
                raise JogAndSaveSquarePoseError(
                    "intermediate step {} joint {} delta {} exceeds step-size {}.".format(
                        index + 1,
                        joint_name,
                        delta,
                        step_size,
                    )
                )
            previous[joint_name] = int(position)



def determine_approach_through_above(args):
    if args.approach_through_above is not None:
        return bool(args.approach_through_above)
    return bool(args.move_to_existing_pose and args.pose_name in ("pick_pose", "place_pose"))



def build_startup_plan(current_joints, startup_targets, step_size_ticks, joint_limits):
    stages = []
    current = dict((name, int(value)) for name, value in current_joints.items())
    for target in startup_targets:
        target_joints = dict((name, int(value)) for name, value in target["target_joints"].items())
        steps = build_intermediate_poses(current, target_joints, step_size_ticks)
        validate_step_sequence(current, steps, joint_limits, step_size_ticks)
        stages.append({
            "label": target["label"],
            "pose_name": target["pose_name"],
            "target_joints": target_joints,
            "steps": steps,
            "step_count": len(steps),
        })
        current = target_joints
    return stages



def _read_joint_positions(bus, ids_by_joint, joint_names):
    positions = {}
    for joint_name in joint_names:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_position(servo_id)
        if value is None:
            raise JogAndSaveSquarePoseError(
                "Present position could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        positions[joint_name] = int(value)
    return positions



def _validate_current_positions(current_positions, limits_by_joint, gripper_limits=None):
    for joint_name, value in current_positions.items():
        if joint_name == "gripper" and gripper_limits is not None:
            minimum = int(gripper_limits["min"])
            maximum = int(gripper_limits["max"])
            if int(value) < minimum or int(value) > maximum:
                raise JogAndSaveSquarePoseError(
                    "Current position joint gripper {} is outside software limits {}..{}.".format(
                        int(value),
                        minimum,
                        maximum,
                    )
                )
            continue
        _validate_joint_value(joint_name, int(value), limits_by_joint, "current position")



def _read_hardware_errors(bus, ids_by_joint, joint_names):
    errors = {}
    address, length = REG_HARDWARE_ERROR_STATUS
    for joint_name in joint_names:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_register(servo_id, address, length)
        if value is None:
            raise JogAndSaveSquarePoseError(
                "Hardware error register could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        errors[joint_name] = int(value)
    return errors



def _check_zero_hardware_errors(bus, ids_by_joint, joint_names):
    errors = _read_hardware_errors(bus, ids_by_joint, joint_names)
    nonzero = dict((joint_name, code) for joint_name, code in errors.items() if int(code) != 0)
    if nonzero:
        raise JogAndSaveSquarePoseError("Hardware error register is non-zero: {}".format(nonzero))
    return errors



def _enable_torque(bus, ids_by_joint, joint_names):
    for joint_name in joint_names:
        bus.torque_enable(ids_by_joint[joint_name], True)



def _disable_torque(bus, ids_by_joint, joint_names):
    success = True
    for joint_name in joint_names:
        try:
            bus.torque_enable(ids_by_joint[joint_name], False)
        except Exception:
            success = False
    return success



def _write_pose(bus, ids_by_joint, joint_names, pose):
    for joint_name in joint_names:
        bus.write_goal_position(ids_by_joint[joint_name], int(pose[joint_name]))



def _write_single_joint(bus, ids_by_joint, joint_name, target_position):
    bus.write_goal_position(ids_by_joint[joint_name], int(target_position))



def build_result_template(args, validation):
    return {
        "started_at": _utc_timestamp(),
        "completed_at": None,
        "real": bool(args.real),
        "dry_run": not bool(args.real),
        "square": validation["square"],
        "pose_name": validation["pose_name"],
        "move_to_existing_pose": bool(validation["move_to_existing_pose"]),
        "approach_through_above": bool(validation["approach_through_above"]),
        "write_enabled": bool(args.write),
        "force": bool(args.force),
        "initial_readback": None,
        "original_pose_joints": copy.deepcopy(validation["original_pose_joints"]),
        "final_readback": None,
        "saved": False,
        "would_save": False,
        "output_written": False,
        "events": [],
        "aborted": False,
        "abort_reason": None,
        "final_torque_disable_attempted": False,
        "final_torque_disable_success": None,
    }



def _record_event(result, args, event_type, **fields):
    event = {"timestamp": _utc_timestamp(), "event": event_type}
    event.update(fields)
    result["events"].append(event)
    if args.log:
        append_text_log(args.log, event)
    return event



def _resolve_command_hint(args):
    parts = ["python3", "tools/jog_and_save_square_pose.py"]
    parts.extend(["--square", args.square])
    parts.extend(["--pose-name", args.pose_name])
    parts.extend(["--targets", args.targets])
    parts.extend(["--joint-limits", args.joint_limits])
    parts.extend(["--servo-map", args.servo_map])
    parts.extend(["--robot-config", args.robot_config])
    parts.append("--real")
    parts.extend(["--confirm-text", REQUIRED_CONFIRM_TEXT])
    if args.move_to_existing_pose:
        parts.append("--move-to-existing-pose")
    if determine_approach_through_above(args):
        parts.append("--approach-through-above")
    if args.write:
        parts.append("--write")
    if args.force:
        parts.append("--force")
    if args.allow_gripper_jog:
        parts.append("--allow-gripper-jog")
    parts.extend(["--jog-step", str(args.jog_step)])
    parts.extend(["--max-single-jog", str(args.max_single_jog)])
    parts.extend(["--max-total-delta", str(args.max_total_delta)])
    parts.extend(["--step-size-ticks", str(args.step_size_ticks)])
    parts.extend(["--step-delay", str(args.step_delay)])
    parts.extend(["--settle-time", str(args.settle_time)])
    if args.log:
        parts.extend(["--log", args.log])
    if args.output_json:
        parts.extend(["--output-json", args.output_json])
    return " ".join([shlex.quote(part) for part in parts])



def _print_pose(label, joints, joint_names):
    if joints is None:
        print("{}: none".format(label))
        return
    print("{}: {}".format(
        label,
        ", ".join(["{}={}".format(name, joints[name]) for name in joint_names]),
    ))



def _print_dry_run_summary(args, validation):
    print("Mode: DRY-RUN")
    print("Selected square: {}".format(validation["square"]))
    print("Selected pose_name: {}".format(validation["pose_name"]))
    _print_pose("Existing pose values", validation["original_pose_joints"], validation["joint_order"])
    if validation["startup_targets"]:
        print("Startup movement preview:")
        for item in validation["startup_targets"]:
            print("  - {} -> {}".format(item["label"], ", ".join([
                "{}={}".format(name, item["target_joints"][name])
                for name in validation["movement_joints"]
            ])))
    else:
        print("Startup movement preview: none")
    print("Allowed jog joints: {}".format(", ".join(validation["joggable_joints"])))
    print("Write enabled: {}".format("yes" if args.write else "no"))
    print("Force enabled: {}".format("yes" if args.force else "no"))
    print("Warning: this is powered calibration and must be supervised.")
    print("Real command (not executed):")
    print(_resolve_command_hint(args))



def _print_real_instructions(validation):
    print("Mode: REAL")
    print("Safety instructions:")
    print("  - keep one hand near the robot power switch")
    print("  - supervise every jog visually")
    print("  - do not allow cable snag or board collision")
    print("  - press Ctrl+C or type q to abort")
    print("Exact confirmation required: {}".format(REQUIRED_CONFIRM_TEXT))
    print("Allowed jog joints: {}".format(", ".join(validation["joggable_joints"])))
    print("Operator commands: help, status, read, step N, jog JOINT DELTA, save, quit, q")



def _status_lines(validation, state, args):
    lines = []
    joint_names = validation["joint_order"]
    current = state.current_readback
    original = validation["original_pose_joints"]
    delta_text = []
    if current is not None and original is not None:
        for joint_name in joint_names:
            delta_text.append("{}={}".format(joint_name, int(current[joint_name]) - int(original[joint_name])))
    lines.append("square: {}".format(validation["square"]))
    lines.append("pose_name: {}".format(validation["pose_name"]))
    if current is not None:
        lines.append("current readback joints: {}".format(", ".join(["{}={}".format(name, current[name]) for name in joint_names])))
    else:
        lines.append("current readback joints: unavailable")
    if original is not None:
        lines.append("original pose joints: {}".format(", ".join(["{}={}".format(name, original[name]) for name in joint_names])))
    else:
        lines.append("original pose joints: none")
    lines.append("delta from original pose: {}".format(", ".join(delta_text) if delta_text else "none"))
    lines.append("current jog step: {}".format(state.jog_step))
    lines.append("allowed joints: {}".format(", ".join(validation["joggable_joints"])))
    lines.append("write enabled: {}".format("yes" if args.write else "no"))
    lines.append("force enabled: {}".format("yes" if args.force else "no"))
    return lines



def _print_status(validation, state, args):
    for line in _status_lines(validation, state, args):
        print(line)



def parse_operator_command(raw, current_step, allowed_joints, max_single_jog):
    text = str(raw or "").strip()
    if not text:
        raise CommandRejected("Empty command. Type help for available commands.")
    parts = text.split()
    head = parts[0].lower()
    if head in ("help", "status", "read", "save", "quit", "q"):
        action = head
        if action == "q":
            action = "quit"
        return {"action": action}
    if head == "step":
        if len(parts) != 2:
            raise CommandRejected("Usage: step N")
        try:
            value = int(parts[1])
        except ValueError:
            raise CommandRejected("step requires an integer tick value.")
        if value <= 0:
            raise CommandRejected("step must be > 0.")
        if abs(value) > int(max_single_jog):
            raise CommandRejected("step {} exceeds max-single-jog {}.".format(value, max_single_jog))
        return {"action": "step", "value": int(value)}
    if head == "jog":
        if len(parts) != 3:
            raise CommandRejected("Usage: jog JOINT DELTA")
        joint_name = safety.normalize_joint_name(parts[1])
        if joint_name not in allowed_joints:
            raise CommandRejected("Joint {} is not joggable in this session.".format(joint_name))
        try:
            delta = int(parts[2])
        except ValueError:
            raise CommandRejected("jog delta must be an integer.")
        if delta == 0:
            raise CommandRejected("jog delta must be non-zero.")
        if abs(delta) > int(max_single_jog):
            raise CommandRejected("jog delta {} exceeds max-single-jog {}.".format(delta, max_single_jog))
        return {"action": "jog", "joint": joint_name, "delta": int(delta)}
    shorthand_joint = SHORTHAND_JOINTS.get(head)
    if shorthand_joint is not None:
        if shorthand_joint not in allowed_joints:
            raise CommandRejected("Joint {} is not joggable in this session.".format(shorthand_joint))
        if len(parts) != 2:
            raise CommandRejected("Usage: {} +/-N".format(head))
        token = parts[1].strip()
        if token in ("+", "-"):
            delta = int(current_step) if token == "+" else (-1 * int(current_step))
        else:
            try:
                delta = int(token)
            except ValueError:
                raise CommandRejected("shorthand delta must be an integer.")
        if delta == 0:
            raise CommandRejected("jog delta must be non-zero.")
        if abs(delta) > int(max_single_jog):
            raise CommandRejected("jog delta {} exceeds max-single-jog {}.".format(delta, max_single_jog))
        return {"action": "jog", "joint": shorthand_joint, "delta": int(delta)}
    raise CommandRejected("Unknown command {!r}. Type help for available commands.".format(head))



def validate_jog_request(current_positions, initial_positions, joint_name, delta,
                         joint_limits, max_total_delta, gripper_limits=None):
    if joint_name not in current_positions:
        raise JogAndSaveSquarePoseError("Current position is unavailable for {}.".format(joint_name))
    current_value = int(current_positions[joint_name])
    target_value = current_value + int(delta)
    if joint_name == "gripper" and gripper_limits is not None:
        minimum = int(gripper_limits["min"])
        maximum = int(gripper_limits["max"])
        if target_value < minimum or target_value > maximum:
            raise JogAndSaveSquarePoseError(
                "jog target for gripper {} is outside limits {}..{}.".format(
                    target_value,
                    minimum,
                    maximum,
                )
            )
    else:
        _validate_joint_value(joint_name, int(target_value), joint_limits, "jog target")
    origin = int(initial_positions[joint_name])
    total_delta = target_value - origin
    if abs(total_delta) > int(max_total_delta):
        raise JogAndSaveSquarePoseError(
            "total delta {} for {} exceeds max-total-delta {}.".format(
                total_delta,
                joint_name,
                int(max_total_delta),
            )
        )
    return {
        "joint": joint_name,
        "current_position": current_value,
        "delta": int(delta),
        "target_position": target_value,
        "total_delta": total_delta,
    }



def build_save_notes(previous_source):
    return [
        "powered jog correction",
        "previous pose source: {}".format(previous_source or "none"),
        "taught under torque",
    ]



def prepare_save_document(document, square, pose_name, current_joints, write_enabled, force):
    doc = robot_square_map.normalise_square_targets_document(document)
    square_name = robot_square_map.normalise_square_name(square)
    pose_key = robot_square_map.validate_pose_name(pose_name)
    square_info = doc.get("squares", {}).get(square_name) or {}
    existing_pose = square_info.get(pose_key) if isinstance(square_info, dict) else None
    existing_source = existing_pose.get("source") if isinstance(existing_pose, dict) else None
    notes = build_save_notes(existing_source)
    timestamp = _utc_timestamp()
    updated = robot_square_map.upsert_manual_pose(
        doc,
        square_name,
        pose_key,
        current_joints,
        notes=notes,
        force=bool(force),
        recorded_at=timestamp,
    )
    entry = updated["squares"][square_name][pose_key]
    return {
        "document": updated,
        "entry": entry,
        "existing_source": existing_source,
        "saved": bool(write_enabled),
        "would_save": True,
        "timestamp": timestamp,
    }



def _print_save_entry(entry, label):
    print(label)
    print(yaml.safe_dump(entry, default_flow_style=False).rstrip())



def _perform_save(args, validation, state, document, current_joints, result):
    if not isinstance(current_joints, dict):
        raise JogAndSaveSquarePoseError("Current readback is unavailable for save.")
    issues = robot_square_map.validate_pose_joints(current_joints, validation["joint_limits"], validation["joint_order"])
    if issues:
        raise JogAndSaveSquarePoseError("Save validation failed: {}".format("; ".join(issues)))
    prepared = prepare_save_document(
        document,
        validation["square"],
        validation["pose_name"],
        current_joints,
        args.write,
        args.force,
    )
    state.saved = bool(prepared["saved"])
    state.would_save = bool(prepared["would_save"])
    result["saved"] = bool(prepared["saved"])
    result["would_save"] = bool(prepared["would_save"])
    if args.write:
        robot_square_map.save_yaml_file(args.targets, prepared["document"])
        _print_save_entry(prepared["entry"], "Wrote pose entry:")
        print("Save result: wrote {} {}.".format(validation["square"], validation["pose_name"]))
    else:
        _print_save_entry(prepared["entry"], "Would write pose entry:")
        print("Save result: dry-run only; file not modified.")
    _record_event(
        result,
        args,
        "save",
        saved=bool(prepared["saved"]),
        would_save=True,
        existing_source=prepared["existing_source"],
        square=validation["square"],
        pose_name=validation["pose_name"],
        joints=dict(current_joints),
    )
    return prepared



def validate_inputs(args, config_loader=load_robot_config):
    args.square = robot_square_map.normalise_square_name(args.square)
    try:
        args.pose_name = robot_square_map.validate_pose_name(args.pose_name)
    except robot_square_map.SquareTargetError as exc:
        raise JogAndSaveSquarePoseError(str(exc))
    args.jog_step = _coerce_positive_int(args.jog_step, "--jog-step")
    args.max_single_jog = _coerce_positive_int(args.max_single_jog, "--max-single-jog")
    args.max_total_delta = _coerce_positive_int(args.max_total_delta, "--max-total-delta")
    args.step_size_ticks = _coerce_positive_int(args.step_size_ticks, "--step-size-ticks")
    args.step_delay = _coerce_nonnegative_float(args.step_delay, "--step-delay")
    args.settle_time = _coerce_nonnegative_float(args.settle_time, "--settle-time")
    if args.jog_step > args.max_single_jog:
        raise JogAndSaveSquarePoseError("--jog-step must be <= --max-single-jog.")
    if args.real and args.confirm_text != REQUIRED_CONFIRM_TEXT:
        raise JogAndSaveSquarePoseError(
            "Real mode requires exact confirmation text {!r}.".format(REQUIRED_CONFIRM_TEXT)
        )

    document = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits)
    servo_map = robot_square_map.load_servo_map(args.servo_map)
    config = config_loader(args.robot_config)
    gripper_profile = _load_optional_yaml(DEFAULT_GRIPPER_PROFILE_PATH, {})
    joint_order = list(document.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)
    movement_joints = list(DEFAULT_MOVEMENT_JOINTS)
    joggable_joints = _movement_joints(bool(args.allow_gripper_jog))
    save_joint_ids = _servo_ids_by_joint(servo_map, joint_order)
    ids_by_joint = _servo_ids_by_joint(servo_map, joggable_joints)
    approach_through_above = determine_approach_through_above(args)
    gripper_limits = _merged_gripper_limits(joint_limits, gripper_profile)

    squares = document.get("squares") or {}
    square_info = squares.get(args.square)
    if not isinstance(square_info, dict):
        raise JogAndSaveSquarePoseError("square {} is missing from square_targets.yaml.".format(args.square))

    existing_pose = _validate_pose_entry(args.square, args.pose_name, square_info.get(args.pose_name), joint_order, joint_limits)
    if args.move_to_existing_pose and existing_pose is None:
        raise JogAndSaveSquarePoseError(
            "--move-to-existing-pose requires an existing {} for square {}.".format(args.pose_name, args.square)
        )

    above_pose = _validate_pose_entry(args.square, "above_pose", square_info.get("above_pose"), joint_order, joint_limits)
    if approach_through_above and above_pose is None:
        raise JogAndSaveSquarePoseError(
            "--approach-through-above requires above_pose for square {}.".format(args.square)
        )

    startup_targets = []
    if args.move_to_existing_pose:
        if approach_through_above and args.pose_name in ("pick_pose", "place_pose"):
            startup_targets.append({
                "label": "approach through above_pose",
                "pose_name": "above_pose",
                "target_joints": dict((joint_name, int(above_pose["joints"][joint_name])) for joint_name in movement_joints),
            })
        startup_targets.append({
            "label": "move to existing {}".format(args.pose_name),
            "pose_name": args.pose_name,
            "target_joints": dict((joint_name, int(existing_pose["joints"][joint_name])) for joint_name in movement_joints),
        })

    return {
        "square": args.square,
        "pose_name": args.pose_name,
        "document": document,
        "joint_limits": joint_limits,
        "servo_map": servo_map,
        "config": config,
        "gripper_profile": gripper_profile,
        "joint_order": joint_order,
        "movement_joints": movement_joints,
        "joggable_joints": joggable_joints,
        "ids_by_joint": ids_by_joint,
        "save_joint_ids": save_joint_ids,
        "original_pose_joints": None if existing_pose is None else dict(existing_pose["joints"]),
        "original_pose_source": None if existing_pose is None else existing_pose["source"],
        "startup_targets": startup_targets,
        "move_to_existing_pose": bool(args.move_to_existing_pose),
        "approach_through_above": bool(approach_through_above),
        "gripper_limits": gripper_limits,
    }



def _read_all_session_positions(bus, validation):
    joint_names = list(validation["joint_order"])
    current = _read_joint_positions(bus, validation["save_joint_ids"], joint_names)
    _validate_current_positions(current, validation["joint_limits"], validation["gripper_limits"])
    return current



def _execute_startup_stages(bus, validation, args, result, sleep_fn):
    current_positions = _read_joint_positions(bus, validation["ids_by_joint"], validation["movement_joints"])
    _validate_current_positions(current_positions, validation["joint_limits"])
    stages = build_startup_plan(
        current_positions,
        validation["startup_targets"],
        args.step_size_ticks,
        validation["joint_limits"],
    )
    for stage in stages:
        _record_event(
            result,
            args,
            "startup_stage_begin",
            label=stage["label"],
            pose_name=stage["pose_name"],
            step_count=stage["step_count"],
            target_joints=dict(stage["target_joints"]),
        )
        for pose in stage["steps"]:
            _write_pose(bus, validation["ids_by_joint"], validation["movement_joints"], pose)
            if float(args.step_delay) > 0.0:
                sleep_fn(float(args.step_delay))
        if float(args.settle_time) > 0.0:
            sleep_fn(float(args.settle_time))
        observed = _read_joint_positions(bus, validation["ids_by_joint"], validation["movement_joints"])
        _validate_current_positions(observed, validation["joint_limits"])
        hardware_errors = _check_zero_hardware_errors(bus, validation["ids_by_joint"], validation["joggable_joints"])
        _record_event(
            result,
            args,
            "startup_stage_end",
            label=stage["label"],
            pose_name=stage["pose_name"],
            observed=observed,
            hardware_errors=hardware_errors,
        )
    return stages



def _print_help():
    print("Commands:")
    print("  help")
    print("  status")
    print("  read")
    print("  step N")
    print("  jog JOINT DELTA")
    print("  save")
    print("  quit")
    print("  q")
    print("Optional shorthand:")
    print("  pan +/-N, lift +/-N, elbow +/-N, wrist +/-N, roll +/-N")



def run(args, bus_factory=build_servo_bus, config_loader=load_robot_config,
        sleep_fn=time.sleep, input_fn=input):
    validation = validate_inputs(args, config_loader=config_loader)
    result = build_result_template(args, validation)
    state = SessionState(args.jog_step)
    _record_event(
        result,
        args,
        "session_start",
        square=validation["square"],
        pose_name=validation["pose_name"],
        real=bool(args.real),
        write_enabled=bool(args.write),
        force=bool(args.force),
        confirmation_status=(args.confirm_text == REQUIRED_CONFIRM_TEXT),
        startup_targets=copy.deepcopy(validation["startup_targets"]),
    )

    if not args.real:
        _print_dry_run_summary(args, validation)
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            result["output_written"] = bool(write_json_result(args.output_json, result))
        return 0, result

    bus = None
    torque_joint_names = list(validation["joggable_joints"])
    try:
        _print_real_instructions(validation)
        bus = bus_factory(
            config=validation["config"],
            config_path=args.robot_config,
            dry_run=False,
            backend_name=_resolve_real_backend_name(validation["config"]),
            mock_ids=None,
        )
        current_before = _read_all_session_positions(bus, validation)
        state.initial_readback = dict(current_before)
        state.current_readback = dict(current_before)
        result["initial_readback"] = dict(current_before)
        _check_zero_hardware_errors(bus, validation["ids_by_joint"], torque_joint_names)
        _enable_torque(bus, validation["ids_by_joint"], torque_joint_names)
        _record_event(result, args, "torque_enable", joints=list(torque_joint_names))
        if validation["startup_targets"]:
            _execute_startup_stages(bus, validation, args, result, sleep_fn)
            current_after_startup = _read_all_session_positions(bus, validation)
            state.initial_readback = dict(current_after_startup)
            state.current_readback = dict(current_after_startup)
            result["initial_readback"] = dict(current_after_startup)
        _print_status(validation, state, args)
        while True:
            raw = input_fn("jog> ")
            try:
                command = parse_operator_command(
                    raw,
                    state.jog_step,
                    validation["joggable_joints"],
                    args.max_single_jog,
                )
            except CommandRejected as exc:
                print("Command rejected: {}".format(exc))
                continue

            action = command["action"]
            if action == "help":
                _print_help()
                continue
            if action == "status":
                _print_status(validation, state, args)
                continue
            if action == "read":
                state.current_readback = _read_all_session_positions(bus, validation)
                result["final_readback"] = dict(state.current_readback)
                _record_event(result, args, "read", readback=dict(state.current_readback))
                print("Readback: {}".format(", ".join(["{}={}".format(name, state.current_readback[name]) for name in validation["joint_order"]])))
                continue
            if action == "step":
                state.jog_step = int(command["value"])
                _record_event(result, args, "step_change", jog_step=state.jog_step)
                print("Current jog step set to {} ticks.".format(state.jog_step))
                continue
            if action == "quit":
                raise JogAndSaveSquarePoseAbort("operator_quit")
            if action == "save":
                state.current_readback = _read_all_session_positions(bus, validation)
                result["final_readback"] = dict(state.current_readback)
                _perform_save(args, validation, state, validation["document"], state.current_readback, result)
                continue
            if action == "jog":
                if state.current_readback is None:
                    state.current_readback = _read_all_session_positions(bus, validation)
                plan = validate_jog_request(
                    state.current_readback,
                    state.initial_readback,
                    command["joint"],
                    command["delta"],
                    validation["joint_limits"],
                    args.max_total_delta,
                    gripper_limits=validation["gripper_limits"],
                )
                _write_single_joint(bus, validation["ids_by_joint"], plan["joint"], plan["target_position"])
                if float(args.settle_time) > 0.0:
                    sleep_fn(float(args.settle_time))
                state.current_readback = _read_all_session_positions(bus, validation)
                result["final_readback"] = dict(state.current_readback)
                hardware_errors = _check_zero_hardware_errors(bus, validation["ids_by_joint"], torque_joint_names)
                _record_event(
                    result,
                    args,
                    "jog",
                    joint=plan["joint"],
                    delta=plan["delta"],
                    target_position=plan["target_position"],
                    total_delta=plan["total_delta"],
                    readback=dict(state.current_readback),
                    hardware_errors=hardware_errors,
                )
                print("Current readback: {}".format(", ".join(["{}={}".format(name, state.current_readback[name]) for name in validation["joint_order"]])))
                continue
        return 0, result
    except KeyboardInterrupt:
        result["aborted"] = True
        result["abort_reason"] = "keyboard_interrupt"
        state.abort_reason = result["abort_reason"]
        _record_event(result, args, "abort", reason=result["abort_reason"])
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            result["output_written"] = bool(write_json_result(args.output_json, result))
        return 1, result
    except JogAndSaveSquarePoseAbort as exc:
        result["aborted"] = True
        result["abort_reason"] = str(exc)
        state.abort_reason = result["abort_reason"]
        _record_event(result, args, "abort", reason=result["abort_reason"])
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            result["output_written"] = bool(write_json_result(args.output_json, result))
        return 1, result
    except SaveRejected as exc:
        _record_event(result, args, "save_rejected", reason=str(exc))
        raise
    except Exception as exc:
        _record_event(result, args, "exception", error=str(exc))
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            result["output_written"] = bool(write_json_result(args.output_json, result))
        raise
    finally:
        if bus is not None:
            result["final_torque_disable_attempted"] = True
            result["final_torque_disable_success"] = _disable_torque(
                bus,
                validation["ids_by_joint"],
                torque_joint_names,
            )
            _record_event(
                result,
                args,
                "torque_disable",
                attempted=True,
                success=bool(result["final_torque_disable_success"]),
            )
            bus.close()
        if result.get("completed_at") is None:
            result["completed_at"] = _utc_timestamp()
        if state.current_readback is not None:
            result["final_readback"] = dict(state.current_readback)
        if args.output_json:
            result["output_written"] = bool(write_json_result(args.output_json, result))



def build_parser():
    parser = argparse.ArgumentParser(
        description="Powered jog-and-save square pose correction tool. Dry-run is default."
    )
    parser.add_argument("--square", default=DEFAULT_SQUARE, help="Target square. Prefer explicit, for example c3.")
    parser.add_argument(
        "--pose-name",
        default=DEFAULT_POSE_NAME,
        choices=robot_square_map.ALLOWED_POSE_NAMES,
        help="Pose entry to correct. Defaults to above_pose.",
    )
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits", default=DEFAULT_JOINT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--servo-map", dest="servo_map", default=DEFAULT_SERVO_MAP_PATH,
                        help="Servo-map YAML path.")
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG_PATH,
                        help="Robot config YAML path.")
    parser.add_argument("--real", action="store_true", help="Enable real powered jogging.")
    parser.add_argument("--confirm-text", default=None,
                        help="Exact required confirmation text for real mode.")
    parser.add_argument("--move-to-existing-pose", action="store_true",
                        help="Move slowly to the existing saved pose before interactive jogging.")
    parser.add_argument("--approach-through-above", action="store_true", default=None,
                        help="Approach pick/place via above_pose before moving to the selected pose.")
    parser.add_argument("--write", action="store_true", help="Write the updated YAML file on save.")
    parser.add_argument("--force", action="store_true", help="Allow overwrite of an existing manual pose.")
    parser.add_argument("--jog-step", type=int, default=DEFAULT_JOG_STEP,
                        help="Default current jog step used by status and shorthand commands.")
    parser.add_argument("--max-single-jog", type=int, default=DEFAULT_MAX_SINGLE_JOG,
                        help="Maximum allowed absolute delta per jog command.")
    parser.add_argument("--max-total-delta", type=int, default=DEFAULT_MAX_TOTAL_DELTA,
                        help="Maximum allowed total per-joint delta from session start.")
    parser.add_argument("--step-size-ticks", type=int, default=DEFAULT_STEP_SIZE_TICKS,
                        help="Maximum per-joint delta per startup movement step.")
    parser.add_argument("--step-delay", type=float, default=DEFAULT_STEP_DELAY,
                        help="Delay between startup movement steps in seconds.")
    parser.add_argument("--settle-time", type=float, default=DEFAULT_SETTLE_TIME,
                        help="Post-movement settle delay in seconds.")
    parser.add_argument("--allow-gripper-jog", action="store_true",
                        help="Allow manual jogging of gripper in this session.")
    parser.add_argument("--log", default=DEFAULT_LOG_PATH, help="Text log path.")
    parser.add_argument("--output-json", default=DEFAULT_OUTPUT_JSON_PATH,
                        help="Optional JSON result output path.")
    return parser



def main():
    args = build_parser().parse_args()
    try:
        exit_code, _result = run(args)
    except JogAndSaveSquarePoseError as exc:
        print("ERROR: {}".format(exc))
        return 1
    return int(exit_code)


if __name__ == "__main__":
    sys.exit(main())
