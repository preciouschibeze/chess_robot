#!/usr/bin/env python3
from __future__ import absolute_import
from __future__ import print_function

import argparse
import json
import math
import os
import shlex
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

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

EXPECTED_CONFIRM_TEXT = "PICK PLACE BASELINE"
REG_HARDWARE_ERROR_STATUS = (65, 1)
DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
DEFAULT_GRIPPER_PROFILE_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "gripper_profile.yaml")
DEFAULT_ROBOT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_LOG_PATH = os.path.join(ROOT, "data", "logs", "open_loop_pick_place.log")
DEFAULT_OUTPUT_JSON_PATH = os.path.join(ROOT, "data", "debug", "open_loop_pick_place_c3_c3_dry_run.json")
DEFAULT_SOURCE = "c3"
DEFAULT_DEST = "c3"
DEFAULT_PIECE = "rook"


class OpenLoopPickPlaceError(RuntimeError):
    """Raised when the baseline tool should refuse to continue."""


class OpenLoopPickPlaceAbort(RuntimeError):
    """Raised when the operator aborts the baseline cleanly."""


def _utc_timestamp():
    return datetime.utcnow().isoformat() + "Z"


def _ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def append_text_log(path, payload):
    _ensure_parent_dir(path)
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def write_json_result(path, payload):
    _ensure_parent_dir(path)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _resolve_pause_each(args):
    if args.pause_each is None:
        return bool(args.real)
    return bool(args.pause_each)


def _validate_joint_value(joint_name, value, joint_limits, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise OpenLoopPickPlaceError(
            "{} joint {} must be an integer tick value.".format(context, joint_name)
        )
    limits = safety.resolve_joint_limits(joint_limits, joint_name)
    if limits is None:
        raise OpenLoopPickPlaceError("No joint limits are configured for {}.".format(joint_name))
    minimum = int(limits["min"])
    maximum = int(limits["max"])
    if value < minimum or value > maximum:
        raise OpenLoopPickPlaceError(
            "{} joint {} target {} is outside limits {}..{}.".format(
                context,
                joint_name,
                value,
                minimum,
                maximum,
            )
        )
    return int(value)


def validate_pose_within_limits(pose, joint_limits, context):
    for joint_name, value in pose.items():
        _validate_joint_value(joint_name, int(value), joint_limits, context)


def build_intermediate_poses(current_joints, target_joints, step_size_ticks):
    step_size = int(step_size_ticks)
    if step_size <= 0:
        raise OpenLoopPickPlaceError("--step-size-ticks must be > 0.")
    current_keys = sorted(current_joints.keys())
    if current_keys != sorted(target_joints.keys()):
        raise OpenLoopPickPlaceError("Current joints and target joints must contain the same keys.")
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


def build_intermediate_positions(current_position, target_position, step_size_ticks):
    current_value = int(current_position)
    target_value = int(target_position)
    step_size = int(step_size_ticks)
    if step_size <= 0:
        raise OpenLoopPickPlaceError("--gripper-step-size-ticks must be > 0.")
    max_delta = abs(target_value - current_value)
    if max_delta == 0:
        return [target_value]
    step_count = int(math.ceil(float(max_delta) / float(step_size)))
    positions = []
    for step_index in range(1, step_count + 1):
        delta = target_value - current_value
        interpolated = current_value + int(round((float(delta) * float(step_index)) / float(step_count)))
        positions.append(int(interpolated))
    return positions


def validate_step_sequence(current_joints, poses, joint_limits, step_size_ticks):
    previous = dict(current_joints)
    step_size = int(step_size_ticks)
    for index, pose in enumerate(poses):
        validate_pose_within_limits(pose, joint_limits, "intermediate step {}".format(index + 1))
        for joint_name, value in pose.items():
            delta = abs(int(value) - int(previous[joint_name]))
            if delta > step_size:
                raise OpenLoopPickPlaceError(
                    "Intermediate step {} joint {} delta {} exceeds step-size {}.".format(
                        index + 1,
                        joint_name,
                        delta,
                        step_size,
                    )
                )
        previous = dict(pose)


def validate_position_sequence(current_position, positions, limits, step_size_ticks, label):
    previous = int(current_position)
    step_size = int(step_size_ticks)
    minimum = int(limits["min"])
    maximum = int(limits["max"])
    for index, position in enumerate(positions):
        position = int(position)
        if position < minimum or position > maximum:
            raise OpenLoopPickPlaceError(
                "{} intermediate step {} target {} is outside limits {}..{}.".format(
                    label,
                    index + 1,
                    position,
                    minimum,
                    maximum,
                )
            )
        delta = abs(position - previous)
        if delta > step_size:
            raise OpenLoopPickPlaceError(
                "{} intermediate step {} delta {} exceeds step-size {}.".format(
                    label,
                    index + 1,
                    delta,
                    step_size,
                )
            )
        previous = position


def estimate_step_count(current_joints, target_joints, step_size_ticks):
    return len(build_intermediate_poses(current_joints, target_joints, step_size_ticks))


def estimate_position_step_count(current_position, target_position, step_size_ticks):
    return len(build_intermediate_positions(current_position, target_position, step_size_ticks))


def _stage_name_list(stages):
    return [stage["name"] for stage in stages]


def _servo_ids_by_joint(servo_map, joint_names):
    joints = servo_map.get("joints") or {}
    mapping = {}
    for joint_name in joint_names:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise OpenLoopPickPlaceError("servo_map is missing joint {}.".format(joint_name))
        if "id" not in entry:
            raise OpenLoopPickPlaceError("servo_map joint {} is missing id.".format(joint_name))
        mapping[joint_name] = safety.validate_servo_id(entry.get("id"))
    return mapping


def _load_gripper_profile(path):
    document = robot_square_map.load_yaml_file(path, {}) or {}
    profile = document.get("gripper") if isinstance(document, dict) else None
    if not isinstance(profile, dict):
        raise OpenLoopPickPlaceError("gripper_profile.yaml must contain a top-level 'gripper' mapping.")
    return profile


def _validate_pose_entry(square_name, pose_name, pose_entry, movement_joints, known_joint_names,
                         joint_limits, require_manual, warnings):
    context = "square {} {}".format(square_name, pose_name)
    if not isinstance(pose_entry, dict):
        raise OpenLoopPickPlaceError("{} is missing or invalid.".format(context))
    source = pose_entry.get("source") or "unknown"
    if require_manual and source != "manual":
        raise OpenLoopPickPlaceError("{} must have source manual, got {}.".format(context, source))
    if (not require_manual) and source != "manual":
        warnings.append("{} source is {}; treat this above pose as operator-reviewed only.".format(context, source))
    joints = pose_entry.get("joints")
    if not isinstance(joints, dict):
        raise OpenLoopPickPlaceError("{} must contain a joints mapping.".format(context))
    unknown = [joint_name for joint_name in sorted(joints.keys()) if joint_name not in known_joint_names]
    if unknown:
        raise OpenLoopPickPlaceError("{} includes unknown joints: {}".format(context, ", ".join(unknown)))
    target_joints = {}
    for joint_name in movement_joints:
        if joint_name not in joints:
            raise OpenLoopPickPlaceError("{} is missing required joint {}.".format(context, joint_name))
        target_joints[joint_name] = _validate_joint_value(joint_name, joints.get(joint_name), joint_limits, context)
    return {
        "square": square_name,
        "pose_name": pose_name,
        "source": source,
        "target_joints": target_joints,
        "raw_joints": dict(joints),
    }


def _validate_gripper_positions(gripper_profile, joint_limits, servo_map):
    joint_name = gripper_profile.get("joint") or "gripper"
    if joint_name != "gripper":
        raise OpenLoopPickPlaceError("gripper_profile joint must be gripper, got {}.".format(joint_name))
    servo_ids = _servo_ids_by_joint(servo_map, ["gripper"])
    servo_id = servo_ids["gripper"]
    profile_servo_id = gripper_profile.get("servo_id")
    if profile_servo_id is not None and int(profile_servo_id) != int(servo_id):
        raise OpenLoopPickPlaceError(
            "gripper_profile servo_id {} does not match servo_map gripper id {}.".format(
                profile_servo_id,
                servo_id,
            )
        )

    limits = safety.resolve_joint_limits(joint_limits, "gripper")
    if limits is None:
        raise OpenLoopPickPlaceError("No joint limits are configured for gripper.")
    profile_limits = safety.resolve_joint_limits({"gripper": gripper_profile.get("limits")}, "gripper")
    if profile_limits is not None:
        merged_limits = {
            "min": max(int(limits["min"]), int(profile_limits["min"])),
            "max": min(int(limits["max"]), int(profile_limits["max"])),
        }
    else:
        merged_limits = {"min": int(limits["min"]), "max": int(limits["max"])}
    if merged_limits["min"] > merged_limits["max"]:
        raise OpenLoopPickPlaceError("Merged gripper limits are invalid.")

    required_names = ["open_position", "grasp_position"]
    for name in required_names:
        if gripper_profile.get(name) is None:
            raise OpenLoopPickPlaceError("gripper_profile.yaml is missing required {}.".format(name))

    positions = {
        "open_position": int(gripper_profile.get("open_position")),
        "grasp_position": int(gripper_profile.get("grasp_position")),
        "release_position": int(
            gripper_profile.get("release_position")
            if gripper_profile.get("release_position") is not None
            else gripper_profile.get("open_position")
        ),
        "pre_grasp_position": None,
        "neutral_position": None,
    }
    if gripper_profile.get("pre_grasp_position") is not None:
        positions["pre_grasp_position"] = int(gripper_profile.get("pre_grasp_position"))
    if gripper_profile.get("neutral_position") is not None:
        positions["neutral_position"] = int(gripper_profile.get("neutral_position"))

    for name, value in positions.items():
        if value is None:
            continue
        if value < int(merged_limits["min"]) or value > int(merged_limits["max"]):
            raise OpenLoopPickPlaceError(
                "gripper {} target {} is outside merged limits {}..{}.".format(
                    name,
                    value,
                    merged_limits["min"],
                    merged_limits["max"],
                )
            )
    return {
        "joint_name": joint_name,
        "servo_id": servo_id,
        "limits": merged_limits,
        "positions": positions,
    }


def validate_inputs(args, config_loader=load_robot_config):
    source = robot_square_map.normalise_square_name(args.source)
    dest = robot_square_map.normalise_square_name(args.dest)
    if args.real and source == dest and not args.allow_same_square:
        raise OpenLoopPickPlaceError(
            "Real same-square pick/place requires --allow-same-square."
        )
    if args.real and args.confirm_text != EXPECTED_CONFIRM_TEXT:
        raise OpenLoopPickPlaceError(
            "Real mode requires exact confirmation text {!r}.".format(EXPECTED_CONFIRM_TEXT)
        )

    document = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits)
    servo_map = robot_square_map.load_servo_map(args.servo_map)
    gripper_profile = _load_gripper_profile(args.gripper_profile)
    joint_order = list(document.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)
    movement_joints = [joint_name for joint_name in joint_order if joint_name != "gripper"]
    known_joint_names = set(joint_order)
    known_joint_names.update((servo_map.get("joints") or {}).keys())
    known_joint_names.add("gripper")

    squares = document.get("squares") or {}
    source_square = squares.get(source)
    dest_square = squares.get(dest)
    if not isinstance(source_square, dict):
        raise OpenLoopPickPlaceError("Invalid source square {}.".format(source))
    if not isinstance(dest_square, dict):
        raise OpenLoopPickPlaceError("Invalid dest square {}.".format(dest))

    warnings = []
    source_above = source_square.get("above_pose")
    source_pick = source_square.get("pick_pose")
    dest_above = dest_square.get("above_pose")
    dest_place = dest_square.get("place_pose")
    dest_pick = dest_square.get("pick_pose")

    if source_above is None:
        raise OpenLoopPickPlaceError("source {} above_pose is missing.".format(source))
    if source_pick is None:
        raise OpenLoopPickPlaceError("source {} pick_pose is missing.".format(source))
    if dest_above is None:
        raise OpenLoopPickPlaceError("dest {} above_pose is missing.".format(dest))
    if dest_place is None and not args.allow_place_uses_pick:
        raise OpenLoopPickPlaceError(
            "dest {} place_pose is missing and --allow-place-uses-pick was not passed.".format(dest)
        )
    if dest_place is None:
        if dest_pick is None:
            raise OpenLoopPickPlaceError(
                "dest {} place_pose is missing and dest pick_pose is unavailable for fallback.".format(dest)
            )
        dest_place = dest_pick
        warnings.append("dest {} place_pose missing; fell back to pick_pose.".format(dest))

    source_above_entry = _validate_pose_entry(
        source,
        "above_pose",
        source_above,
        movement_joints,
        known_joint_names,
        joint_limits,
        False,
        warnings,
    )
    source_pick_entry = _validate_pose_entry(
        source,
        "pick_pose",
        source_pick,
        movement_joints,
        known_joint_names,
        joint_limits,
        True,
        warnings,
    )
    dest_above_entry = _validate_pose_entry(
        dest,
        "above_pose",
        dest_above,
        movement_joints,
        known_joint_names,
        joint_limits,
        False,
        warnings,
    )
    dest_place_name = "place_pose"
    if dest_square.get("place_pose") is None and args.allow_place_uses_pick:
        dest_place_name = "pick_pose"
    dest_place_entry = _validate_pose_entry(
        dest,
        dest_place_name,
        dest_place,
        movement_joints,
        known_joint_names,
        joint_limits,
        True,
        warnings,
    )

    gripper = _validate_gripper_positions(gripper_profile, joint_limits, servo_map)
    ids_by_joint = _servo_ids_by_joint(servo_map, movement_joints + ["gripper"])

    config = None
    if args.real:
        config = config_loader(args.robot_config)
        configured_joints = config.get("joints") or {}
        safety.validate_multi_joint_commanded_joints(
            configured_joints=configured_joints,
            commanded_joints=movement_joints + ["gripper"],
            include_gripper=True,
        )

    return {
        "source": source,
        "dest": dest,
        "piece": args.piece,
        "warnings": warnings,
        "document": document,
        "joint_limits": joint_limits,
        "servo_map": servo_map,
        "gripper_profile": gripper_profile,
        "movement_joints": movement_joints,
        "ids_by_joint": ids_by_joint,
        "source_above": source_above_entry,
        "source_pick": source_pick_entry,
        "dest_above": dest_above_entry,
        "dest_place": dest_place_entry,
        "gripper": gripper,
        "config": config,
    }


def build_stage_sequence(validation, pause_each):
    stages = []
    positions = validation["gripper"]["positions"]

    def add_arm_stage(name, pose_entry, pose_name, square_name):
        stages.append({
            "name": name,
            "kind": "arm",
            "square": square_name,
            "target_pose_name": pose_name,
            "target_joints": dict(pose_entry["target_joints"]),
            "warnings": [],
        })

    def add_gripper_stage(name, position_name):
        target_value = positions.get(position_name)
        if target_value is None:
            return
        stages.append({
            "name": name,
            "kind": "gripper",
            "target_gripper_name": position_name,
            "target_gripper_value": int(target_value),
            "warnings": [],
        })

    def add_pause_stage(name, prompt):
        if not pause_each:
            return
        stages.append({
            "name": name,
            "kind": "pause",
            "prompt": prompt,
            "warnings": [],
        })

    add_gripper_stage("open_gripper", "open_position")
    add_arm_stage("move_source_above", validation["source_above"], "above_pose", validation["source"])
    add_pause_stage("pause_before_pick_descent", "Pause before descent to pick_pose")
    if positions.get("pre_grasp_position") is not None:
        add_gripper_stage("move_pre_grasp", "pre_grasp_position")
    add_arm_stage("move_source_pick", validation["source_pick"], "pick_pose", validation["source"])
    add_pause_stage("pause_before_gripper_close", "Pause before gripper close")
    add_gripper_stage("close_gripper", "grasp_position")
    add_pause_stage("pause_before_lift_after_grasp", "Pause before lift after grasp")
    add_arm_stage("move_source_above_after_pick", validation["source_above"], "above_pose", validation["source"])
    add_arm_stage("move_dest_above", validation["dest_above"], "above_pose", validation["dest"])
    add_pause_stage("pause_before_place_descent", "Pause before descent to place_pose")
    add_arm_stage("move_dest_place", validation["dest_place"], validation["dest_place"]["pose_name"], validation["dest"])
    add_pause_stage("pause_before_gripper_release", "Pause before gripper release")
    add_gripper_stage("release_gripper", "release_position")
    add_pause_stage("pause_before_final_lift", "Pause before final lift")
    add_arm_stage("move_dest_above_after_place", validation["dest_above"], "above_pose", validation["dest"])
    if positions.get("neutral_position") is not None:
        add_gripper_stage("move_gripper_neutral", "neutral_position")

    last_arm_target = None
    last_gripper_target = None
    for stage in stages:
        estimated_steps = None
        if stage["kind"] == "arm":
            if last_arm_target is not None:
                estimated_steps = estimate_step_count(last_arm_target, stage["target_joints"], validation.get("step_size_ticks", 1))
            last_arm_target = dict(stage["target_joints"])
        elif stage["kind"] == "gripper":
            if last_gripper_target is not None:
                estimated_steps = estimate_position_step_count(last_gripper_target, stage["target_gripper_value"], validation.get("gripper_step_size_ticks", 1))
            last_gripper_target = int(stage["target_gripper_value"])
        stage["estimated_steps"] = estimated_steps
    return stages


def _display_path(path):
    absolute_path = os.path.abspath(path)
    root_prefix = ROOT + os.sep
    if absolute_path.startswith(root_prefix):
        return absolute_path[len(root_prefix):]
    return path


def _command_hint(args):
    output_json = args.output_json
    if output_json:
        output_json = output_json.replace("_dry_run.json", "_real.json")
    else:
        output_json = os.path.join("data", "debug", "open_loop_pick_place_c3_c3_real.json")
    command = [
        "python3",
        "tools/test_open_loop_pick_place.py",
        "--source", args.source,
        "--dest", args.dest,
    ]
    if args.allow_same_square:
        command.append("--allow-same-square")
    if args.allow_place_uses_pick:
        command.append("--allow-place-uses-pick")
    command.extend([
        "--targets", _display_path(args.targets),
        "--joint-limits", _display_path(args.joint_limits),
        "--servo-map", _display_path(args.servo_map),
        "--gripper-profile", _display_path(args.gripper_profile),
        "--robot-config", _display_path(args.robot_config),
        "--real",
        "--confirm-text", EXPECTED_CONFIRM_TEXT,
        "--pause-each",
        "--step-size-ticks", str(args.step_size_ticks),
        "--step-delay", str(args.step_delay),
        "--settle-time", str(args.settle_time),
        "--gripper-step-size-ticks", str(args.gripper_step_size_ticks),
        "--gripper-step-delay", str(args.gripper_step_delay),
        "--piece", args.piece,
        "--log", _display_path(args.log),
        "--output-json", _display_path(output_json),
    ])
    return " ".join([shlex.quote(part) for part in command])


def _build_result(args, validation, stages, pause_each):
    return {
        "started_at": _utc_timestamp(),
        "completed_at": None,
        "dry_run": not bool(args.real),
        "real": bool(args.real),
        "source": validation["source"],
        "dest": validation["dest"],
        "piece": args.piece,
        "allow_same_square": bool(args.allow_same_square),
        "sequence_stages": _stage_name_list(stages),
        "arm_movement_joints": list(validation["movement_joints"]),
        "gripper_commanded": True,
        "gripper_positions_used": dict(validation["gripper"]["positions"]),
        "step_size_ticks": int(args.step_size_ticks),
        "step_delay": float(args.step_delay),
        "gripper_step_size_ticks": int(args.gripper_step_size_ticks),
        "gripper_step_delay": float(args.gripper_step_delay),
        "pause_each": bool(pause_each),
        "warnings": list(validation["warnings"]),
        "per_stage_results": [],
        "aborted": False,
        "abort_reason": None,
        "final_torque_disable_attempted": False,
        "final_torque_disable_success": None,
    }


def mark_aborted(result, reason):
    result["aborted"] = True
    result["abort_reason"] = str(reason)
    return result


def _build_stage_result(stage, validation, args):
    return {
        "timestamp": _utc_timestamp(),
        "stage_name": stage["name"],
        "source": validation["source"],
        "dest": validation["dest"],
        "piece": args.piece,
        "target_pose_name": stage.get("target_pose_name"),
        "target_joints": dict(stage.get("target_joints") or {}),
        "target_gripper_name": stage.get("target_gripper_name"),
        "target_gripper_value": stage.get("target_gripper_value"),
        "current_readback_before": None,
        "observed_readback_after": None,
        "final_error": None,
        "status": None,
        "warnings": list(stage.get("warnings") or []),
        "exception": None,
        "estimated_steps": stage.get("estimated_steps"),
    }


def _record_stage_result(result, stage_result, log_path):
    result["per_stage_results"].append(stage_result)
    if log_path:
        append_text_log(log_path, stage_result)


def _print_safety_instructions(validation):
    print("Mode: REAL")
    print("Safety instructions:")
    print("  - empty board except the single test {}".format(validation["piece"]))
    print("  - supervised lab setup")
    print("  - keep hand near power switch")
    print("  - watch for cable snag")
    print("  - watch for gripper collision")
    print("  - watch for wrist dipping into board")
    print("  - press Ctrl+C to abort")
    print("  - use q at pauses to abort cleanly")
    print("Typed confirmation required: {}".format(EXPECTED_CONFIRM_TEXT))


def _print_dry_run_summary(validation, stages, args):
    print("Mode: DRY-RUN")
    print("Source: {}".format(validation["source"]))
    print("Dest: {}".format(validation["dest"]))
    print("Piece: {}".format(args.piece))
    print("Required poses found: source above/pick, dest above/place")
    print("Required poses missing: none")
    print("Movement joints: {}".format(", ".join(validation["movement_joints"])))
    print("Arm target sequence:")
    for stage in stages:
        if stage["kind"] != "arm":
            continue
        print(
            "  - {} {} {} estimated_steps={}".format(
                stage["name"],
                stage["square"],
                ", ".join(["{}={}".format(name, stage["target_joints"][name]) for name in validation["movement_joints"]]),
                stage.get("estimated_steps") if stage.get("estimated_steps") is not None else "live-read-required",
            )
        )
    print("Gripper target sequence:")
    for stage in stages:
        if stage["kind"] != "gripper":
            continue
        print(
            "  - {} {}={} estimated_steps={}".format(
                stage["name"],
                stage["target_gripper_name"],
                stage["target_gripper_value"],
                stage.get("estimated_steps") if stage.get("estimated_steps") is not None else "live-read-required",
            )
        )
    print("Validation summary: OK")
    if validation["warnings"]:
        print("Warnings:")
        for warning in validation["warnings"]:
            print("  - {}".format(warning))
    print("Real command (not executed):")
    print(_command_hint(args))
    print("Warning: this open-loop baseline is expected to expose backlash/compliance and may fail.")


def _pause_or_abort(stage, pause_input_fn):
    response = pause_input_fn("{} [Enter/q]: ".format(stage["prompt"]))
    if response is None:
        response = ""
    if str(response).strip().lower() == "q":
        raise OpenLoopPickPlaceAbort("operator_abort_at_{}".format(stage["name"]))


def _read_joint_positions(bus, ids_by_joint, joint_names):
    positions = {}
    for joint_name in joint_names:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_position(servo_id)
        if value is None:
            raise OpenLoopPickPlaceError(
                "Present position could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        positions[joint_name] = int(value)
    return positions


def _read_gripper_position(bus, servo_id):
    value = bus.read_position(servo_id)
    if value is None:
        raise OpenLoopPickPlaceError("Present position could not be read for gripper servo {}.".format(servo_id))
    return int(value)


def _read_hardware_errors(bus, ids_by_joint, joint_names):
    errors = {}
    address, length = REG_HARDWARE_ERROR_STATUS
    for joint_name in joint_names:
        servo_id = ids_by_joint[joint_name]
        value = bus.read_register(servo_id, address, length)
        if value is None:
            raise OpenLoopPickPlaceError(
                "Hardware error register could not be read for joint {} (servo {}).".format(joint_name, servo_id)
            )
        errors[joint_name] = int(value)
    return errors


def _check_zero_hardware_errors(bus, ids_by_joint, joint_names):
    errors = _read_hardware_errors(bus, ids_by_joint, joint_names)
    nonzero = dict((joint_name, code) for joint_name, code in errors.items() if int(code) != 0)
    if nonzero:
        raise OpenLoopPickPlaceError("Hardware error register is non-zero: {}".format(nonzero))
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


def _write_arm_pose(bus, ids_by_joint, movement_joints, pose):
    for joint_name in movement_joints:
        bus.write_goal_position(ids_by_joint[joint_name], int(pose[joint_name]))


def _execute_arm_stage(bus, stage, validation, args, sleep_fn):
    movement_joints = validation["movement_joints"]
    ids_by_joint = validation["ids_by_joint"]
    current_positions = _read_joint_positions(bus, ids_by_joint, movement_joints)
    validate_pose_within_limits(current_positions, validation["joint_limits"], "current arm position")
    steps = build_intermediate_poses(current_positions, stage["target_joints"], args.step_size_ticks)
    validate_step_sequence(current_positions, steps, validation["joint_limits"], args.step_size_ticks)
    for pose in steps:
        _write_arm_pose(bus, ids_by_joint, movement_joints, pose)
        if float(args.step_delay) > 0.0:
            sleep_fn(float(args.step_delay))
    if float(args.settle_time) > 0.0:
        sleep_fn(float(args.settle_time))
    observed = _read_joint_positions(bus, ids_by_joint, movement_joints)
    validate_pose_within_limits(observed, validation["joint_limits"], "observed arm position")
    _check_zero_hardware_errors(bus, ids_by_joint, movement_joints + ["gripper"])
    final_error = {}
    for joint_name in movement_joints:
        final_error[joint_name] = int(observed[joint_name]) - int(stage["target_joints"][joint_name])
    return {
        "current": current_positions,
        "observed": observed,
        "final_error": final_error,
    }


def _execute_gripper_stage(bus, stage, validation, args, sleep_fn):
    servo_id = validation["gripper"]["servo_id"]
    current_position = _read_gripper_position(bus, servo_id)
    gripper_limits = validation["gripper"]["limits"]
    if current_position < int(gripper_limits["min"]) or current_position > int(gripper_limits["max"]):
        raise OpenLoopPickPlaceError(
            "Current gripper position {} is outside limits {}..{}.".format(
                current_position,
                gripper_limits["min"],
                gripper_limits["max"],
            )
        )
    positions = build_intermediate_positions(current_position, stage["target_gripper_value"], args.gripper_step_size_ticks)
    validate_position_sequence(
        current_position,
        positions,
        gripper_limits,
        args.gripper_step_size_ticks,
        stage["name"],
    )
    for position in positions:
        bus.write_goal_position(servo_id, int(position))
        if float(args.gripper_step_delay) > 0.0:
            sleep_fn(float(args.gripper_step_delay))
    observed = _read_gripper_position(bus, servo_id)
    _check_zero_hardware_errors(bus, validation["ids_by_joint"], ["gripper"])
    final_error = int(observed) - int(stage["target_gripper_value"])
    return {
        "current": current_position,
        "observed": observed,
        "final_error": final_error,
    }


def run(args, bus_factory=build_servo_bus, config_loader=load_robot_config,
        sleep_fn=time.sleep, pause_input_fn=input):
    if int(args.step_size_ticks) <= 0:
        raise OpenLoopPickPlaceError("--step-size-ticks must be > 0.")
    if float(args.step_delay) < 0.0:
        raise OpenLoopPickPlaceError("--step-delay must be >= 0.")
    if float(args.settle_time) < 0.0:
        raise OpenLoopPickPlaceError("--settle-time must be >= 0.")
    if int(args.gripper_step_size_ticks) <= 0:
        raise OpenLoopPickPlaceError("--gripper-step-size-ticks must be > 0.")
    if float(args.gripper_step_delay) < 0.0:
        raise OpenLoopPickPlaceError("--gripper-step-delay must be >= 0.")

    pause_each = _resolve_pause_each(args)
    validation = validate_inputs(args, config_loader=config_loader)
    validation["piece"] = args.piece
    validation["step_size_ticks"] = int(args.step_size_ticks)
    validation["gripper_step_size_ticks"] = int(args.gripper_step_size_ticks)
    stages = build_stage_sequence(validation, pause_each)
    result = _build_result(args, validation, stages, pause_each)

    if not args.real:
        _print_dry_run_summary(validation, stages, args)
        for stage in stages:
            stage_result = _build_stage_result(stage, validation, args)
            stage_result["status"] = "dry_run"
            _record_stage_result(result, stage_result, args.log if args.log else None)
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        return 0, result

    _print_safety_instructions(validation)
    bus = None
    current_stage_result = None
    torque_joint_names = validation["movement_joints"] + ["gripper"]
    try:
        bus = bus_factory(
            config=validation["config"],
            config_path=args.robot_config,
            dry_run=False,
            backend_name="feetech",
            mock_ids=None,
        )
        _read_joint_positions(bus, validation["ids_by_joint"], validation["movement_joints"])
        _read_gripper_position(bus, validation["gripper"]["servo_id"])
        _check_zero_hardware_errors(bus, validation["ids_by_joint"], torque_joint_names)
        _enable_torque(bus, validation["ids_by_joint"], torque_joint_names)
        for stage in stages:
            current_stage_result = _build_stage_result(stage, validation, args)
            print("Stage: {}".format(stage["name"]))
            if stage["kind"] == "pause":
                current_stage_result["status"] = "waiting"
                _record_stage_result(result, current_stage_result, args.log if args.log else None)
                _pause_or_abort(stage, pause_input_fn)
                current_stage_result = None
                continue
            if stage["kind"] == "arm":
                print("Target pose {} {}".format(stage["square"], stage["target_pose_name"]))
                execution = _execute_arm_stage(bus, stage, validation, args, sleep_fn)
            else:
                print("Target gripper {}={}".format(stage["target_gripper_name"], stage["target_gripper_value"]))
                execution = _execute_gripper_stage(bus, stage, validation, args, sleep_fn)
            current_stage_result["current_readback_before"] = execution["current"]
            current_stage_result["observed_readback_after"] = execution["observed"]
            current_stage_result["final_error"] = execution["final_error"]
            current_stage_result["status"] = "ok"
            _record_stage_result(result, current_stage_result, args.log if args.log else None)
            print("Readback after movement: {}".format(execution["observed"]))
            current_stage_result = None
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        return 0, result
    except KeyboardInterrupt:
        if current_stage_result is not None and current_stage_result.get("status") is None:
            current_stage_result["status"] = "aborted"
            current_stage_result["exception"] = "KeyboardInterrupt"
            _record_stage_result(result, current_stage_result, args.log if args.log else None)
        mark_aborted(result, "keyboard_interrupt")
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        return 1, result
    except OpenLoopPickPlaceAbort as exc:
        if current_stage_result is not None and current_stage_result.get("status") is None:
            current_stage_result["status"] = "aborted"
            current_stage_result["exception"] = str(exc)
            _record_stage_result(result, current_stage_result, args.log if args.log else None)
        mark_aborted(result, str(exc))
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        return 1, result
    except Exception as exc:
        if current_stage_result is not None and current_stage_result.get("status") is None:
            current_stage_result["status"] = "failed"
            current_stage_result["exception"] = str(exc)
            _record_stage_result(result, current_stage_result, args.log if args.log else None)
        result["completed_at"] = _utc_timestamp()
        if args.output_json:
            write_json_result(args.output_json, result)
        raise
    finally:
        if bus is not None:
            result["final_torque_disable_attempted"] = True
            result["final_torque_disable_success"] = _disable_torque(
                bus,
                validation["ids_by_joint"],
                torque_joint_names,
            )
            bus.close()
            if args.output_json and result.get("completed_at") is not None:
                write_json_result(args.output_json, result)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Controlled open-loop pick/place baseline using taught square poses and gripper profile."
    )
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="Source square. Default: c3")
    parser.add_argument("--dest", default=DEFAULT_DEST, help="Destination square. Default: c3")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits", default=DEFAULT_JOINT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--servo-map", dest="servo_map", default=DEFAULT_SERVO_MAP_PATH,
                        help="Servo-map YAML path.")
    parser.add_argument("--gripper-profile", default=DEFAULT_GRIPPER_PROFILE_PATH,
                        help="Gripper profile YAML path.")
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG_PATH,
                        help="Robot config YAML path.")
    parser.add_argument("--real", action="store_true", help="Enable real hardware motion.")
    parser.add_argument("--confirm-text", default=None,
                        help="Exact required confirmation text for real mode.")
    parser.add_argument("--pause-each", action="store_true", default=None,
                        help="Pause between critical stages. Defaults to true in real mode.")
    parser.add_argument("--step-size-ticks", type=int, default=5,
                        help="Maximum per-joint delta per arm step.")
    parser.add_argument("--step-delay", type=float, default=0.15,
                        help="Delay between arm steps in seconds.")
    parser.add_argument("--settle-time", type=float, default=1.0,
                        help="Post-arm-settle delay in seconds.")
    parser.add_argument("--gripper-step-size-ticks", type=int, default=5,
                        help="Maximum gripper delta per step.")
    parser.add_argument("--gripper-step-delay", type=float, default=0.08,
                        help="Delay between gripper steps in seconds.")
    parser.add_argument("--log", default=DEFAULT_LOG_PATH, help="Text log path.")
    parser.add_argument("--output-json", default=None, help="Optional JSON result output path.")
    parser.add_argument("--piece", default=DEFAULT_PIECE, help="Piece label for logging/output.")
    parser.add_argument("--allow-same-square", action="store_true",
                        help="Allow same-square source/dest in real mode.")
    parser.add_argument("--allow-place-uses-pick", action="store_true",
                        help="Allow dest pick_pose fallback when place_pose is missing.")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        exit_code, result = run(args)
    except (BackendUnavailable, ServoBusError, OpenLoopPickPlaceError, ValueError, OSError) as exc:
        print("ERROR: {}".format(exc))
        raise SystemExit(1)
    print("Completed. Aborted: {}".format("yes" if result.get("aborted") else "no"))
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
