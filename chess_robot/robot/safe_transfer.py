from __future__ import absolute_import

import csv
import json
import math
import os
import time

import numpy as np

from chess_robot.robot.ik import robot_base_point_to_world
from chess_robot.robot.ik import world_point_to_robot_base
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.ik_validation import arm_joint_mapping
from chess_robot.robot.ik_validation import arm_servo_ids
from chess_robot.robot.ik_validation import build_approach_report
from chess_robot.robot.ik_validation import build_execution_bus
from chess_robot.robot.ik_validation import build_home_pose_ik_result
from chess_robot.robot.ik_validation import build_interpolated_tick_waypoints
from chess_robot.robot.ik_validation import build_static_safety_checks
from chess_robot.robot.ik_validation import calculate_safety_limit_margins_ticks
from chess_robot.robot.ik_validation import convert_margin_ticks_to_deg
from chess_robot.robot.ik_validation import ensure_parent_dir
from chess_robot.robot.ik_validation import first_failed_check_reason
from chess_robot.robot.ik_validation import inter_waypoint_delay_s
from chess_robot.robot.ik_validation import joint_angles_to_ticks
from chess_robot.robot.ik_validation import load_validation_context
from chess_robot.robot.ik_validation import make_check
from chess_robot.robot.ik_validation import read_current_ticks
from chess_robot.robot.ik_validation import solve_single_target_ik
from chess_robot.robot.ik_validation import unavailable_path_validation
from chess_robot.robot.ik_validation import utc_timestamp
from chess_robot.robot.ik_validation import validate_eeprom_limits_if_available
from chess_robot.robot.ik_validation import validate_motion_deltas
from chess_robot.robot.ik_validation import validate_readback
from chess_robot.robot.ik_seed_poses import apply_locked_joint_overrides
from chess_robot.robot.ik_seed_poses import load_ik_seed_poses
from chess_robot.robot.ik_seed_poses import prepare_square_ik_seed
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.motion_safety import approach_axis_world
from chess_robot.robot.motion_safety import approach_tilt_deg
from chess_robot.robot.motion_safety import board_top_z_m
from chess_robot.robot.motion_safety import low_zone_z_m
from chess_robot.robot.motion_safety import resolve_approach_axis_local
from chess_robot.robot.motion_safety import validate_joint_interpolated_tcp_path
from chess_robot.robot.reachability import generate_square_centers
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.urdf_model import DEFAULT_END_LINK


CONFIRM_TEXT = "EXECUTE_SAFE_SQUARE_TRANSFER"
DEFAULT_CSV_LOG_PATH = os.path.join("data", "logs", "safe_square_transfer_validation.csv")
CSV_FIELDNAMES = (
    "timestamp",
    "mode",
    "square",
    "aborted",
    "abort_reason",
    "command_sent_any",
    "segment_count",
    "last_segment",
    "max_ik_error_mm",
    "min_path_z_m",
    "board_clearance_m",
)
SQUARE_IK_SEED_SEGMENTS = (
    "target_high_above",
    "target_normal_above",
    "target_high_above_return",
)
RETURN_STRATEGY_REVERSE_REPLAY = "reverse_replay"
RETURN_STRATEGY_RESOLVE_NEW = "resolve_new"


class SafeTransferError(RuntimeError):
    pass


def run_safe_square_transfer(args, bus_factory=None, ik_solver=None, now_fn=None, sleep_fn=None):
    now_fn = now_fn or utc_timestamp
    sleep_fn = sleep_fn or time.sleep
    timestamp = now_fn()
    mode = "execute" if bool(args.execute) else "dry_run"

    context = load_validation_context(args)
    board_top = board_top_z_m(context["scene_geometry"])
    home_ticks = context.get("home_pose_ticks")
    if home_ticks is None:
        raise SafeTransferError("--home-pose is required for safe square transfer.")
    saved_home = saved_home_metadata(context)
    square_world = square_center_world(context["scene_geometry"], args.square)

    log = build_transfer_log(args, context, timestamp, mode, board_top)
    context["square_ik_seed"] = load_square_ik_seed_context(args, context)
    attach_square_ik_seed_log_metadata(log, context["square_ik_seed"])

    if bool(getattr(args, "start_from_readback", False)) and not bool(args.execute):
        set_transfer_abort(log, "--start-from-readback is only valid with --execute.")
        return finish_transfer_log(args, log)
    if not bool(args.execute) and not bool(getattr(args, "assume_start_home", True)):
        set_transfer_abort(log, "Dry-run mode currently requires --assume-start-home.")
        return finish_transfer_log(args, log)
    if bool(args.execute) and args.confirm != CONFIRM_TEXT:
        set_transfer_abort(log, "Execute mode requires --confirm %s." % CONFIRM_TEXT)
        return finish_transfer_log(args, log)

    bus = None
    config = None
    try:
        if bool(args.execute):
            bus, config = build_execution_bus(args, bus_factory=bus_factory)
            servo_ids = arm_servo_ids(config)
            if sorted(servo_ids.keys()) != sorted(ARM_JOINTS):
                set_transfer_abort(log, "Missing servo IDs for one or more arm joints.")
                return finish_transfer_log(args, log)
            current_ticks = read_current_ticks(bus, servo_ids)
        else:
            servo_ids = None
            current_ticks = arm_ticks_only(home_ticks)

        current_robot, current_world = tcp_from_ticks(context, current_ticks)
        plan = build_staged_plan(
            current_world,
            square_world,
            saved_home["saved_home_tcp_world_xyz_m"],
            board_top,
            args,
        )

        completed_segments = {}
        for segment_index, spec in enumerate(plan, start=1):
            if bool(args.execute):
                current_ticks = read_current_ticks(bus, servo_ids)
                current_robot, current_world = tcp_from_ticks(context, current_ticks)
            segment = evaluate_segment(
                segment_index,
                spec,
                current_ticks,
                context,
                args,
                completed_segments=completed_segments,
                ik_solver=ik_solver,
            )
            log["segments"].append(segment)
            completed_segments[segment["segment_name"]] = segment
            if segment.get("abort_reason"):
                set_transfer_abort(log, segment["abort_reason"])
                break

            if bool(args.execute):
                execute_segment(segment, current_ticks, bus, servo_ids, args, sleep_fn)
                log["command_sent_any"] = bool(log["command_sent_any"] or segment.get("command_sent"))
                if segment.get("abort_reason"):
                    set_transfer_abort(log, segment["abort_reason"])
                    break
            else:
                current_ticks = dict(segment["target_ticks"])

        return finish_transfer_log(args, log)
    except Exception as exc:
        set_transfer_abort(log, str(exc))
        return finish_transfer_log(args, log)
    finally:
        if bus is not None:
            bus.close()


def build_staged_plan(current_world_xyz, square_world_xyz, saved_home_world_xyz, board_top_z, args):
    current = np.asarray(current_world_xyz, dtype=float)
    square = np.asarray(square_world_xyz, dtype=float)
    home = np.asarray(saved_home_world_xyz, dtype=float)
    board_top_z = float(board_top_z)
    transit_z = board_top_z + float(args.transit_clearance_m)
    high_z = board_top_z + float(args.high_above_offset_m)
    normal_z = board_top_z + float(args.normal_above_offset_m)

    plan = [
        make_world_segment("current_lift", [current[0], current[1], max(float(current[2]), transit_z)]),
        make_world_segment("target_high_above", [square[0], square[1], high_z]),
        make_world_segment("target_normal_above", [square[0], square[1], normal_z]),
    ]
    if bool(getattr(args, "return_home", False)):
        plan.extend([
            make_return_world_segment(
                "target_high_above_return",
                [square[0], square[1], high_z],
                "target_high_above",
            ),
            make_return_world_segment(
                "home_high",
                [home[0], home[1], max(float(home[2]), transit_z)],
                "current_lift",
            ),
            {
                "segment_name": "home_pose",
                "target_mode": "home_pose",
                "target_world_xyz_m": [float(value) for value in home],
                "is_return_segment": True,
            },
        ])
    for segment in plan:
        segment["settle_time_s"] = resolve_segment_settle_time(segment["segment_name"], args)
    return plan


def make_world_segment(name, values):
    return {
        "segment_name": str(name),
        "target_mode": "world_xyz",
        "target_world_xyz_m": [float(value) for value in values],
    }


def make_return_world_segment(name, values, replay_source_segment):
    segment = make_world_segment(name, values)
    segment["is_return_segment"] = True
    segment["replay_source_segment"] = str(replay_source_segment)
    return segment


def segment_uses_approach_preference(spec, args):
    if not bool(getattr(args, "prefer_vertical_approach", False)):
        return False
    return str(spec.get("segment_name") or "").startswith("target_")


def segment_uses_approach_enforcement(spec, args):
    if not bool(getattr(args, "enforce_approach_angle", False)):
        return False
    return segment_uses_approach_preference(spec, args)


def resolve_return_replay_source(spec, args):
    if str(getattr(args, "return_strategy", RETURN_STRATEGY_REVERSE_REPLAY)) != RETURN_STRATEGY_REVERSE_REPLAY:
        return None
    replay_source_segment = spec.get("replay_source_segment")
    if replay_source_segment is None:
        return None
    return str(replay_source_segment)


def resolve_replayed_target(replay_source_segment, completed_segments, context, execute_mode):
    source_segment = completed_segments.get(replay_source_segment)
    if source_segment is None:
        raise SafeTransferError("Reverse replay source segment was not found: %s" % replay_source_segment)

    target_ticks = arm_ticks_only(source_segment.get("target_ticks") or {})
    if len(target_ticks) != len(ARM_JOINTS):
        raise SafeTransferError("Reverse replay source segment has no complete target ticks: %s" % replay_source_segment)
    if bool(execute_mode) and not bool(source_segment.get("command_sent")):
        raise SafeTransferError("Reverse replay source segment was not commanded in execute mode: %s" % replay_source_segment)

    final_tcp_robot, final_tcp_world = tcp_from_ticks(context, target_ticks)
    joint_positions_rad = convert_pose_ticks_to_urdf_radians(target_ticks, context["calibration"])
    return {
        "target_ticks": target_ticks,
        "joint_positions_rad": joint_positions_rad,
        "final_tcp_robot": final_tcp_robot,
        "final_tcp_world": final_tcp_world,
        "result": {
            "success": bool(source_segment.get("ik_success")),
            "status": "replayed_forward_target",
            "error_m": source_segment.get("ik_error_m"),
            "iterations": 0,
        },
    }


def evaluate_segment(segment_index, spec, current_ticks, context, args, completed_segments=None, ik_solver=None):
    completed_segments = completed_segments or {}
    target_world = np.asarray(spec["target_world_xyz_m"], dtype=float)
    target_robot = world_point_to_robot_base(target_world, context["scene_geometry"])
    segment = base_segment_log(segment_index, spec, target_world, target_robot, current_ticks)
    if bool(spec.get("is_return_segment")):
        segment["return_strategy"] = str(getattr(args, "return_strategy", RETURN_STRATEGY_REVERSE_REPLAY))
    replay_source_segment = resolve_return_replay_source(spec, args)
    try:
        if replay_source_segment is not None:
            replay = resolve_replayed_target(
                replay_source_segment,
                completed_segments,
                context,
                bool(args.execute),
            )
            target_ticks = replay["target_ticks"]
            joint_positions_rad = replay["joint_positions_rad"]
            segment["ik_seed_source"] = "not_applicable"
            segment["ik_seed_ticks_used"] = {}
            segment["ik_seed_joints_used"] = []
            segment.update({
                "ik_success": bool(replay["result"]["success"]),
                "ik_status": str(replay["result"]["status"]),
                "ik_error_m": None if replay["result"]["error_m"] is None else float(replay["result"]["error_m"]),
                "ik_iterations": int(replay["result"]["iterations"]),
                "final_tcp_world_xyz_m": xyz_list(replay["final_tcp_world"]),
                "final_tcp_robot_xyz_m": xyz_list(replay["final_tcp_robot"]),
                "target_ticks": dict((joint, int(target_ticks[joint])) for joint in target_ticks),
                "replayed_target_ticks": True,
            })
        else:
            current_rad = convert_pose_ticks_to_urdf_radians(current_ticks, context["calibration"])
            segment_seed = resolve_segment_ik_seed(spec, current_ticks, current_rad, context)
            segment["ik_seed_source"] = segment_seed["source"]
            segment["ik_seed_ticks_used"] = dict((joint, int(segment_seed["ticks_used"][joint])) for joint in segment_seed["ticks_used"])
            segment["ik_seed_joints_used"] = list(segment_seed["joints_used"])
            if spec.get("target_mode") == "home_pose":
                result = build_home_pose_ik_result(context, target_robot)
                target_ticks = arm_ticks_only(context["home_pose_ticks"])
            else:
                solver_context = dict(context)
                solver_context["home_seed"] = dict(segment_seed["joint_positions_rad"])
                segment_prefer = segment_uses_approach_preference(spec, args)
                segment_enforce = segment_uses_approach_enforcement(spec, args)
                result = solve_single_target_ik(
                    solver_context,
                    target_robot,
                    args,
                    ik_solver=ik_solver,
                    square=getattr(args, "square", None),
                    prefer_vertical_approach=segment_prefer,
                    enforce_approach_angle=segment_enforce,
                )
                target_ticks = joint_angles_to_ticks(result.joint_positions_rad, context["calibration"])

            joint_positions_rad = result.joint_positions_rad
            final_tcp_world = robot_base_point_to_world(result.final_xyz_robot, context["scene_geometry"])
            segment.update({
                "ik_success": bool(result.success),
                "ik_status": str(result.status),
                "ik_error_m": float(result.error_m),
                "ik_iterations": int(result.iterations),
                "final_tcp_world_xyz_m": xyz_list(final_tcp_world),
                "final_tcp_robot_xyz_m": xyz_list(result.final_xyz_robot),
                "target_ticks": dict((joint, int(target_ticks[joint])) for joint in target_ticks),
            })

        safety_checks = build_static_safety_checks(
            bool(segment["ik_success"]),
            target_ticks,
            context["joint_safety_limits"],
        )
        deltas, delta_checks = validate_motion_deltas(
            current_ticks,
            target_ticks,
            args.max_joint_delta_ticks,
            args.max_total_l1_delta_ticks,
            False,
        )
        safety_checks.extend(delta_checks)
        segment["motion_deltas_ticks"] = deltas
        path_validation = validate_segment_path(context, args, current_ticks, target_ticks)
        segment["path_validation"] = path_validation
        safety_checks.append(make_check(
            "path_validation",
            bool(path_validation.get("passed")),
            path_validation.get("failure_reason") or "Board-clearance path validation failed.",
        ))
        segment["safety_checks"] = safety_checks
        attach_approach_diagnostics(segment, context, args, joint_positions_rad, getattr(args, "square", None), segment_uses_approach_preference(spec, args), segment_uses_approach_enforcement(spec, args))

        if segment.get("approach_enforced") and not bool(segment.get("approach_angle_check", {}).get("passed")):
            segment["abort_reason"] = segment.get("approach_angle_check", {}).get("failure_reason")
        elif replay_source_segment is None and not bool(segment["ik_success"]):
            segment["abort_reason"] = "IK failed: %s" % segment["ik_status"]
        elif replay_source_segment is not None and not bool(segment["ik_success"]):
            segment["abort_reason"] = "Reverse replay source IK was not successful: %s" % replay_source_segment
        elif not all_checks_ok(safety_checks):
            segment["abort_reason"] = first_failed_check_reason(safety_checks)
    except Exception as exc:
        segment["abort_reason"] = str(exc)
    return segment


def execute_segment(segment, current_ticks, bus, servo_ids, args, sleep_fn):
    if segment.get("abort_reason"):
        return segment
    try:
        eeprom_checks = validate_eeprom_limits_if_available(bus, servo_ids, segment["target_ticks"])
        segment["safety_checks"].extend(eeprom_checks)
        if not all_checks_ok(segment["safety_checks"]):
            segment["abort_reason"] = first_failed_check_reason(segment["safety_checks"])
            return segment
        waypoints = build_interpolated_tick_waypoints(current_ticks, segment["target_ticks"], args.speed_scale)
        wrote_any = False
        for waypoint in waypoints:
            for joint_name in ARM_JOINTS:
                bus.write_goal_position(servo_ids[joint_name], waypoint[joint_name])
                wrote_any = True
            sleep_fn(inter_waypoint_delay_s(args.speed_scale))
        sleep_fn(float(segment["settle_time_s"]))
        final_ticks = read_current_ticks(bus, servo_ids)
        segment["final_ticks_after"] = final_ticks
        segment["readback_errors_ticks"] = calculate_readback_errors(final_ticks, segment["target_ticks"])
        readback_checks = validate_readback(final_ticks, segment["target_ticks"], args.readback_tolerance_ticks)
        segment["safety_checks"].extend(readback_checks)
        segment["command_sent"] = bool(wrote_any)
        if not all_checks_ok(readback_checks):
            segment["abort_reason"] = first_failed_check_reason(readback_checks)
    except Exception as exc:
        segment["abort_reason"] = str(exc)
    return segment


def validate_segment_path(context, args, current_ticks, target_ticks):
    low_zone = low_zone_z_m(context["scene_geometry"], args.board_clearance_m)
    try:
        current_rad = convert_pose_ticks_to_urdf_radians(current_ticks, context["calibration"])
        target_rad = convert_pose_ticks_to_urdf_radians(target_ticks, context["calibration"])
        missing = [joint for joint in ARM_JOINTS if joint not in current_rad or joint not in target_rad]
        if missing:
            summary = unavailable_path_validation("missing joint radians: %s" % ",".join(missing), low_zone, args.path_samples)
        else:
            summary = validate_joint_interpolated_tcp_path(
                context["model"],
                context["scene_geometry"],
                current_rad,
                target_rad,
                ARM_JOINTS,
                context.get("end_link", DEFAULT_END_LINK),
                context["tool_frame"],
                low_zone,
                float(args.xy_motion_epsilon_m),
                int(args.path_samples),
            )
    except Exception as exc:
        summary = unavailable_path_validation(str(exc), low_zone, args.path_samples)
    summary["current_ticks_source"] = "segment_start"
    return summary


def attach_approach_diagnostics(segment, context, args, joint_positions_rad, square, prefer_vertical_approach, enforce_approach_angle):
    report = build_approach_report(
        context,
        args,
        joint_positions_rad,
        square=square,
        prefer_vertical_approach=prefer_vertical_approach,
        enforce_approach_angle=enforce_approach_angle,
    )
    segment.update(report)


def base_segment_log(segment_index, spec, target_world, target_robot, current_ticks):
    return {
        "segment_index": int(segment_index),
        "segment_name": spec["segment_name"],
        "target_world_xyz_m": xyz_list(target_world),
        "target_robot_xyz_m": xyz_list(target_robot),
        "ik_success": False,
        "ik_status": None,
        "ik_error_m": None,
        "ik_iterations": None,
        "final_tcp_world_xyz_m": None,
        "final_tcp_robot_xyz_m": None,
        "target_ticks": {},
        "current_ticks_before": arm_ticks_only(current_ticks),
        "final_ticks_after": None,
        "motion_deltas_ticks": {},
        "readback_errors_ticks": None,
        "safety_checks": [],
        "path_validation": None,
        "approach_axis_local": None,
        "approach_axis_name": None,
        "approach_axis_source": None,
        "approach_axis_world": None,
        "approach_target_world_axis": None,
        "approach_tilt_deg": None,
        "approach_weight": None,
        "approach_preferred": False,
        "approach_enforced": False,
        "approach_angle_check": None,
        "selected_approach_tilt_limit_deg": None,
        "best_candidate_axis_name": None,
        "best_candidate_axis_local": None,
        "best_candidate_axis_world": None,
        "best_candidate_axis_tilt_deg": None,
        "ik_seed_source": None,
        "ik_seed_ticks_used": {},
        "ik_seed_joints_used": [],
        "return_strategy": str(spec.get("return_strategy")) if spec.get("return_strategy") is not None else None,
        "replay_source_segment": spec.get("replay_source_segment"),
        "replayed_target_ticks": False,
        "settle_time_s": float(spec["settle_time_s"]),
        "command_sent": False,
        "abort_reason": None,
    }



def resolve_segment_settle_time(segment_name, args):
    base = float(args.settle_time_s)
    if segment_name in ("target_normal_above", "home_pose"):
        value = getattr(args, "final_settle_time_s", None)
        return base if value is None else float(value)
    value = getattr(args, "intermediate_settle_time_s", None)
    return base if value is None else float(value)

def build_transfer_log(args, context, timestamp, mode, board_top):
    locked_ticks = context.get("locked_joint_ticks") or {}
    resolved_policy = getattr(args, "resolved_policy", None)
    if resolved_policy is not None:
        resolved_policy = dict((key, resolved_policy[key]) for key in sorted(resolved_policy.keys()))
    return {
        "timestamp": timestamp,
        "mode": mode,
        "square": str(args.square).lower(),
        "tcp_frame": args.tcp_frame,
        "approach_policy_path": getattr(args, "approach_policy_path", getattr(args, "approach_policy", None)),
        "approach_policy_square": getattr(args, "approach_policy_square", None),
        "resolved_policy": resolved_policy,
        "policy_override_applied": bool(getattr(args, "policy_override_applied", False)),
        "ik_seed_poses_path": getattr(args, "ik_seed_poses", None),
        "ik_seed_square": None,
        "ik_seed_applied": False,
        "ik_seed_notes": [],
        "locked_joints": dict((joint, int(value)) for joint, value in locked_ticks.items()),
        "locked_joint_sources": dict((joint, str(value)) for joint, value in (context.get("locked_joint_sources") or {}).items()),
        "board_top_z_m": float(board_top),
        "board_clearance_m": float(args.board_clearance_m),
        "transit_clearance_m": float(args.transit_clearance_m),
        "normal_above_offset_m": float(args.normal_above_offset_m),
        "high_above_offset_m": float(args.high_above_offset_m),
        "return_home": bool(getattr(args, "return_home", False)),
        "return_strategy": str(getattr(args, "return_strategy", RETURN_STRATEGY_REVERSE_REPLAY)),
        "command_sent_any": False,
        "aborted": False,
        "abort_reason": None,
        "segments": [],
    }


def finish_transfer_log(args, log):
    log["command_sent_any"] = any(bool(segment.get("command_sent")) for segment in log.get("segments", []))
    log["aborted"] = bool(log.get("abort_reason"))
    ensure_parent_dir(args.output)
    with open(args.output, "w") as handle:
        json.dump(log, handle, indent=2, sort_keys=True)
    append_csv_log(getattr(args, "csv_log", DEFAULT_CSV_LOG_PATH), log)
    return log


def append_csv_log(path, log):
    ensure_parent_dir(path)
    exists = os.path.exists(path)
    segments = log.get("segments") or []
    ik_errors = [float(segment["ik_error_m"]) * 1000.0 for segment in segments if segment.get("ik_error_m") is not None]
    min_z_values = [float(segment["path_validation"]["min_z_m"]) for segment in segments if segment.get("path_validation") and segment["path_validation"].get("min_z_m") is not None]
    row = {
        "timestamp": log.get("timestamp"),
        "mode": log.get("mode"),
        "square": log.get("square"),
        "aborted": bool(log.get("aborted")),
        "abort_reason": log.get("abort_reason") or "",
        "command_sent_any": bool(log.get("command_sent_any")),
        "segment_count": len(segments),
        "last_segment": segments[-1].get("segment_name") if segments else "",
        "max_ik_error_mm": max(ik_errors) if ik_errors else None,
        "min_path_z_m": min(min_z_values) if min_z_values else None,
        "board_clearance_m": log.get("board_clearance_m"),
    }
    with open(path, "a") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def set_transfer_abort(log, reason):
    log["abort_reason"] = str(reason)
    log["aborted"] = True


def all_checks_ok(checks):
    return all(bool(check.get("ok")) for check in checks)


def calculate_readback_errors(final_ticks, target_ticks):
    errors = {}
    for joint_name in ARM_JOINTS:
        if final_ticks.get(joint_name) is None or target_ticks.get(joint_name) is None:
            errors[joint_name] = None
        else:
            errors[joint_name] = int(final_ticks[joint_name]) - int(target_ticks[joint_name])
    return errors


def arm_ticks_only(ticks):
    return dict((joint, int(ticks[joint])) for joint in ARM_JOINTS if ticks.get(joint) is not None)


def tcp_from_ticks(context, ticks):
    joint_positions_rad = convert_pose_ticks_to_urdf_radians(ticks, context["calibration"])
    missing = [joint for joint in ARM_JOINTS if joint not in joint_positions_rad]
    if missing:
        raise SafeTransferError("Missing current ticks for FK: %s" % ",".join(missing))
    robot_t_tcp = compute_tcp_transform(
        context["model"],
        joint_positions_rad,
        end_link=context.get("end_link", DEFAULT_END_LINK),
        tool_frame=context["tool_frame"],
    )
    tcp_robot = robot_t_tcp[:3, 3].copy()
    tcp_world = robot_base_point_to_world(tcp_robot, context["scene_geometry"])
    return xyz_list(tcp_robot), xyz_list(tcp_world)


def saved_home_metadata(context):
    tcp_robot, tcp_world = tcp_from_ticks(context, context["home_pose_ticks"])
    return {
        "saved_home_tcp_robot_xyz_m": tcp_robot,
        "saved_home_tcp_world_xyz_m": tcp_world,
    }


def square_center_world(scene_geometry, square):
    requested = str(square).lower()
    for center in generate_square_centers(scene_geometry):
        if center["square"] == requested:
            return [float(center["x_m"]), float(center["y_m"]), float(center["z_m"])]
    raise SafeTransferError("Unknown board square: %s" % square)


def xyz_list(values):
    array = np.asarray(values, dtype=float)
    return [float(array[0]), float(array[1]), float(array[2])]


def load_square_ik_seed_context(args, context):
    info = {
        "path": getattr(args, "ik_seed_poses", None),
        "square": None,
        "seed_applied": False,
        "notes": [],
        "seed_ticks": {},
        "seed_ticks_used": {},
        "seed_positions_rad_used": {},
        "seed_joints_used": [],
    }
    if not getattr(args, "ik_seed_poses", None):
        return info
    if bool(getattr(args, "ignore_ik_seed_poses", False)):
        return info

    document = load_ik_seed_poses(args.ik_seed_poses)
    seed_info = prepare_square_ik_seed(
        document,
        getattr(args, "square", None),
        context["calibration"],
        context["joint_safety_limits"],
        context.get("locked_joint_positions_rad"),
        context.get("locked_joint_ticks"),
    )
    info.update(seed_info)
    return info


def attach_square_ik_seed_log_metadata(log, square_ik_seed):
    if square_ik_seed is None:
        return log
    log["ik_seed_poses_path"] = square_ik_seed.get("path")
    log["ik_seed_square"] = square_ik_seed.get("square")
    log["ik_seed_applied"] = bool(square_ik_seed.get("seed_applied"))
    log["ik_seed_notes"] = list(square_ik_seed.get("notes") or [])
    return log


def resolve_segment_ik_seed(spec, current_ticks, current_rad, context):
    segment_name = str(spec.get("segment_name") or "")
    if spec.get("target_mode") == "home_pose":
        return {
            "source": "not_applicable",
            "ticks_used": {},
            "joints_used": [],
            "joint_positions_rad": {},
        }

    seed_ticks = arm_ticks_only(current_ticks)
    seed_positions_rad = dict((joint_name, float(current_rad[joint_name])) for joint_name in current_rad)
    source = "current_state"

    square_ik_seed = context.get("square_ik_seed") or {}
    if segment_name in SQUARE_IK_SEED_SEGMENTS and bool(square_ik_seed.get("seed_applied")):
        source = "square_seed_pose"
        seed_ticks = dict((joint_name, int(square_ik_seed["seed_ticks_used"][joint_name])) for joint_name in square_ik_seed.get("seed_ticks_used", {}))
        seed_positions_rad = dict((joint_name, float(square_ik_seed["seed_positions_rad_used"][joint_name])) for joint_name in square_ik_seed.get("seed_positions_rad_used", {}))
    else:
        seed_ticks, seed_positions_rad = apply_locked_joint_overrides(
            seed_ticks,
            seed_positions_rad,
            context.get("locked_joint_positions_rad"),
            context.get("locked_joint_ticks"),
        )

    return {
        "source": source,
        "ticks_used": seed_ticks,
        "joints_used": [joint_name for joint_name in ARM_JOINTS if joint_name in seed_ticks],
        "joint_positions_rad": seed_positions_rad,
    }
