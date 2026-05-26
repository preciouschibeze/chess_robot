from __future__ import absolute_import

import json
import os
import time

import numpy as np

from chess_robot.robot.approach_policy import ApproachPolicyError
from chess_robot.robot.approach_policy import load_approach_policy
from chess_robot.robot.approach_policy import resolve_approach_policy
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.ik_validation import arm_servo_ids
from chess_robot.robot.ik_validation import build_execution_bus
from chess_robot.robot.ik_validation import ensure_parent_dir
from chess_robot.robot.ik_validation import load_validation_context
from chess_robot.robot.ik_validation import read_current_ticks
from chess_robot.robot.ik_validation import utc_timestamp
from chess_robot.robot.motion_safety import board_top_z_m
from chess_robot.robot.reachability import generate_square_centers


DEFAULT_RECOVERY_CLEARANCE_M = 0.160
DEFAULT_RECOVERY_ROUTE_SQUARES = ("e4",)
CONFIRM_TEXT_RECOVER_HOME = "EXECUTE_RECOVER_HOME"
RECOVERY_READBACK_FAILURE_MESSAGE = "Recovery unavailable: current servo readback failed."


class SafeRecoveryError(RuntimeError):
    pass


def run_safe_recovery(
    args,
    bus_factory=None,
    context=None,
    board_top=None,
    saved_home=None,
    bus=None,
    config=None,
    servo_ids=None,
    now_fn=None,
    sleep_fn=None,
    ik_solver=None,
    force_current_ticks=None,
    require_confirm=True,
    confirm_text=CONFIRM_TEXT_RECOVER_HOME,
):
    from chess_robot.robot import safe_transfer

    now_fn = now_fn or utc_timestamp
    sleep_fn = sleep_fn or time.sleep
    mode = "execute" if bool(getattr(args, "execute", False)) else "dry_run"
    timestamp = now_fn()
    context = context or load_validation_context(args)
    board_top = float(board_top_z_m(context["scene_geometry"]) if board_top is None else board_top)
    saved_home = saved_home or safe_transfer.saved_home_metadata(context)
    recovery_clearance_m = float(getattr(args, "recovery_clearance_m", DEFAULT_RECOVERY_CLEARANCE_M))
    route_squares = resolve_recovery_route_squares(args, default_route_squares=None)
    route_targets = build_recovery_route_targets(
        context["scene_geometry"],
        board_top,
        route_squares,
        recovery_clearance_m,
    )
    log = build_recovery_log(
        timestamp=timestamp,
        mode=mode,
        recovery_clearance_m=recovery_clearance_m,
        recovery_route_squares=route_squares,
    )

    if bool(getattr(args, "execute", False)) and require_confirm and getattr(args, "confirm", None) != confirm_text:
        log["abort_reason"] = "Execute mode requires --confirm %s." % str(confirm_text)
        log["aborted"] = True
        return finish_recovery_log(log, getattr(args, "output", None))

    created_bus = False
    try:
        if bus is None:
            bus, config = build_execution_bus(args, bus_factory=bus_factory)
            created_bus = True
        if servo_ids is None:
            servo_ids = arm_servo_ids(config)
        if sorted(servo_ids.keys()) != sorted(ARM_JOINTS):
            log["abort_reason"] = "Missing servo IDs for one or more arm joints."
            log["aborted"] = True
            log["recovery_available"] = False
            return finish_recovery_log(log, getattr(args, "output", None))

        if force_current_ticks is None:
            try:
                current_ticks = read_current_ticks(bus, servo_ids)
            except Exception:
                log["abort_reason"] = RECOVERY_READBACK_FAILURE_MESSAGE
                log["aborted"] = True
                log["recovery_available"] = False
                return finish_recovery_log(log, getattr(args, "output", None))
        else:
            current_ticks = arm_ticks_only(force_current_ticks)
            if len(current_ticks) != len(ARM_JOINTS):
                log["abort_reason"] = RECOVERY_READBACK_FAILURE_MESSAGE
                log["aborted"] = True
                log["recovery_available"] = False
                return finish_recovery_log(log, getattr(args, "output", None))

        _, current_world = safe_transfer.tcp_from_ticks(context, current_ticks)
        plan = build_recovery_plan(
            current_world_xyz=current_world,
            saved_home_world_xyz=saved_home["saved_home_tcp_world_xyz_m"],
            board_top_z=board_top,
            recovery_clearance_m=recovery_clearance_m,
            route_targets=route_targets,
            args=args,
        )

        completed_segments = {}
        for segment_index, spec in enumerate(plan, start=1):
            if bool(getattr(args, "execute", False)):
                try:
                    current_ticks = read_current_ticks(bus, servo_ids)
                except Exception:
                    log["abort_reason"] = RECOVERY_READBACK_FAILURE_MESSAGE
                    log["aborted"] = True
                    log["recovery_available"] = False
                    break
            segment = safe_transfer.evaluate_segment(
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
                log["abort_reason"] = str(segment["abort_reason"])
                log["aborted"] = True
                break
            if bool(getattr(args, "execute", False)):
                safe_transfer.execute_segment(segment, current_ticks, bus, servo_ids, args, sleep_fn)
                if segment.get("command_sent"):
                    log["command_sent_any"] = True
                if segment.get("abort_reason"):
                    log["abort_reason"] = str(segment["abort_reason"])
                    log["aborted"] = True
                    break
            else:
                current_ticks = dict(segment.get("target_ticks") or {})

        if log["abort_reason"] is None:
            log["aborted"] = False
            log["recovery_available"] = True
        elif log.get("recovery_available") is None:
            log["recovery_available"] = False
        return finish_recovery_log(log, getattr(args, "output", None))
    except Exception as exc:
        log["abort_reason"] = str(exc)
        log["aborted"] = True
        log["recovery_available"] = False
        return finish_recovery_log(log, getattr(args, "output", None))
    finally:
        if created_bus and bus is not None:
            bus.close()


def build_recovery_log(timestamp, mode, recovery_clearance_m, recovery_route_squares):
    return {
        "timestamp": str(timestamp),
        "mode": str(mode),
        "recovery_needed": True,
        "recovery_available": None,
        "recovery_clearance_m": float(recovery_clearance_m),
        "recovery_route_squares": [str(square).lower() for square in recovery_route_squares],
        "command_sent_any": False,
        "aborted": False,
        "abort_reason": None,
        "segments": [],
    }


def finish_recovery_log(log, output_path):
    log["command_sent_any"] = bool(log.get("command_sent_any")) or any(
        bool(segment.get("command_sent")) for segment in log.get("segments", [])
    )
    log["aborted"] = bool(log.get("abort_reason"))
    if output_path:
        ensure_parent_dir(output_path)
        with open(output_path, "w") as handle:
            json.dump(log, handle, indent=2, sort_keys=True)
    return log


def resolve_recovery_route_squares(args, default_route_squares=None):
    requested = list(getattr(args, "recovery_route_squares", None) or [])
    if requested:
        return _normalize_square_list(requested)
    defaults = _normalize_square_list(default_route_squares or [])
    if defaults:
        return defaults
    policy_squares = _policy_route_squares(args)
    if policy_squares:
        return policy_squares
    return list(DEFAULT_RECOVERY_ROUTE_SQUARES)


def _policy_route_squares(args):
    policy_path = getattr(args, "approach_policy", None) or getattr(args, "approach_policy_path", None)
    if not policy_path:
        return []
    square = getattr(args, "square", None)
    try:
        policy_document = load_approach_policy(policy_path)
        policy_info = resolve_approach_policy(policy_document, square)
        return _normalize_square_list(policy_info.get("resolved_policy", {}).get("return_route_squares", []))
    except (ApproachPolicyError, IOError, OSError, ValueError):
        return []


def build_recovery_route_targets(scene_geometry, board_top_z, route_squares, recovery_clearance_m):
    targets = []
    route_z = float(board_top_z) + float(recovery_clearance_m)
    for square_name in list(route_squares or []):
        square_world = square_center_world(scene_geometry, square_name)
        targets.append({
            "square": str(square_name).lower(),
            "target_world_xyz_m": [float(square_world[0]), float(square_world[1]), float(route_z)],
        })
    return targets


def build_recovery_plan(current_world_xyz, saved_home_world_xyz, board_top_z, recovery_clearance_m, route_targets, args):
    from chess_robot.robot import safe_transfer

    current = np.asarray(current_world_xyz, dtype=float)
    home = np.asarray(saved_home_world_xyz, dtype=float)
    safe_z = float(board_top_z) + float(recovery_clearance_m)
    plan = [
        make_recovery_world_segment("current_safe_lift", [current[0], current[1], max(float(current[2]), safe_z)]),
    ]
    for route_target in list(route_targets or []):
        segment = make_recovery_world_segment(
            "recovery_route_high_%s" % str(route_target["square"]).lower(),
            route_target["target_world_xyz_m"],
        )
        segment["route_waypoint"] = True
        segment["route_square"] = str(route_target["square"]).lower()
        plan.append(segment)
    plan.append(
        make_recovery_world_segment(
            "home_high",
            [home[0], home[1], max(float(home[2]), safe_z)],
        )
    )
    plan.append({
        "segment_name": "home_pose",
        "target_mode": "home_pose",
        "target_world_xyz_m": [float(home[0]), float(home[1]), float(home[2])],
    })
    for segment in plan:
        segment["settle_time_s"] = safe_transfer.resolve_segment_settle_time(segment["segment_name"], args)
    return plan


def make_recovery_world_segment(name, values):
    return {
        "segment_name": str(name),
        "target_mode": "world_xyz",
        "target_world_xyz_m": [float(values[0]), float(values[1]), float(values[2])],
    }


def square_center_world(scene_geometry, square):
    requested = str(square).lower()
    for center in generate_square_centers(scene_geometry):
        if center["square"] == requested:
            return [float(center["x_m"]), float(center["y_m"]), float(center["z_m"])]
    raise SafeRecoveryError("Unknown board square: %s" % square)


def arm_ticks_only(ticks):
    return dict((joint, int(ticks[joint])) for joint in ARM_JOINTS if ticks.get(joint) is not None)


def _normalize_square_list(values):
    normalized = []
    for value in values:
        name = str(value).strip().lower()
        if name:
            normalized.append(name)
    return normalized
