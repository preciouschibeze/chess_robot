from __future__ import absolute_import

import csv
import datetime
import json
import math
import os
import time

import numpy as np

from chess_robot.robot.ik import find_nearest_workspace_seed
from chess_robot.robot.ik import robot_base_point_to_world
from chess_robot.robot.ik import sample_position_workspace
from chess_robot.robot.ik import solve_position_ik_multi_seed
from chess_robot.robot.ik import world_point_to_robot_base
from chess_robot.robot.joint_calibration import angle_rad_to_tick
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.joint_calibration import tick_to_angle_rad
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.motion_safety import approach_axis_world
from chess_robot.robot.motion_safety import approach_tilt_deg
from chess_robot.robot.motion_safety import board_top_z_m
from chess_robot.robot.motion_safety import low_zone_z_m
from chess_robot.robot.motion_safety import make_approach_angle_check
from chess_robot.robot.motion_safety import resolve_approach_axis_local
from chess_robot.robot.motion_safety import validate_joint_interpolated_tcp_path
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.reachability import resolve_joint_limit_bounds
from chess_robot.robot.tool_frames import describe_tool_frame
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import load_scene_geometry


ARM_JOINTS = tuple(EXPECTED_ARM_JOINT_NAMES)
CONFIRM_TEXT = "EXECUTE_SINGLE_IK_POSE"
DEFAULT_CONFIG_PATH = os.path.join("configs", "robot.yaml")
DEFAULT_CSV_LOG_PATH = os.path.join("data", "logs", "single_ik_pose_validation.csv")
CSV_FIELDNAMES = (
    "timestamp",
    "mode",
    "target_name",
    "ik_success",
    "ik_error_mm",
    "command_sent",
    "abort_reason",
    "shoulder_pan_tick",
    "shoulder_lift_tick",
    "elbow_flex_tick",
    "wrist_flex_tick",
    "wrist_roll_tick",
)


class SinglePoseValidationError(RuntimeError):
    pass


def run_single_pose_validation(args, bus_factory=None, ik_solver=None, now_fn=None, sleep_fn=None):
    now_fn = now_fn or utc_timestamp
    sleep_fn = sleep_fn or time.sleep
    timestamp = now_fn()
    mode = "execute" if bool(args.execute) else "dry_run"

    context = load_validation_context(args)
    target = select_target_from_args(context, args)
    if target.get("target_robot_xyz_m") is not None:
        target_robot = np.asarray(target["target_robot_xyz_m"], dtype=float)
    else:
        target_robot = world_point_to_robot_base(target["target_world_xyz_m"], context["scene_geometry"])
    if target.get("target_world_xyz_m") is not None:
        target_world = np.asarray(target["target_world_xyz_m"], dtype=float)
    else:
        target_world = robot_base_point_to_world(target_robot, context["scene_geometry"])
        target["target_world_xyz_m"] = [float(value) for value in target_world]
    target["target_robot_xyz_m"] = [float(value) for value in target_robot]

    if target.get("target_mode") == "home_pose":
        result = build_home_pose_ik_result(context, target_robot)
    else:
        result = solve_single_target_ik(
            context,
            target_robot,
            args,
            ik_solver=ik_solver,
        )
    final_tcp_world = robot_base_point_to_world(result.final_xyz_robot, context["scene_geometry"])
    target_ticks = joint_angles_to_ticks(result.joint_positions_rad, context["calibration"])
    joint_angles_rad = arm_joint_mapping(result.joint_positions_rad)
    joint_angles_deg = dict((name, math.degrees(joint_angles_rad[name])) for name in ARM_JOINTS)
    safety_margins_ticks = calculate_safety_limit_margins_ticks(
        target_ticks,
        context["joint_safety_limits"],
    )
    safety_margins_deg = convert_margin_ticks_to_deg(
        safety_margins_ticks,
        context["calibration"],
    )
    safety_checks = build_static_safety_checks(
        ik_success=bool(result.success),
        target_ticks=target_ticks,
        joint_safety_limits=context["joint_safety_limits"],
    )

    log = build_base_log(
        mode=mode,
        timestamp=timestamp,
        target=target,
        target_robot=target_robot,
        result=result,
        final_tcp_world=final_tcp_world,
        joint_angles_rad=joint_angles_rad,
        joint_angles_deg=joint_angles_deg,
        target_ticks=target_ticks,
        safety_margins_ticks=safety_margins_ticks,
        safety_margins_deg=safety_margins_deg,
        safety_checks=safety_checks,
        tool_frame_description=context["tool_frame_description"],
        locked_joint_positions_rad=context["locked_joint_positions_rad"],
        locked_joint_ticks=context["locked_joint_ticks"],
        locked_joint_sources=context["locked_joint_sources"],
    )
    attach_motion_safety_metadata(log, context, args, result)
    if not bool(args.execute):
        attach_path_validation_from_reference_ticks(log, context, args, context.get("home_pose_ticks"), "saved_home_pose")
    if bool(args.execute) and bool(getattr(args, "enforce_approach_angle", False)) and not bool(log["approach_angle_check"]["passed"]):
        set_abort(log, log["approach_angle_check"]["failure_reason"])

    if not bool(result.success):
        set_abort(log, "IK failed: %s" % result.status)
    elif not all_checks_ok(safety_checks):
        set_abort(log, first_failed_check_reason(safety_checks))

    if bool(args.execute):
        execute_single_pose(
            args,
            log,
            context,
            bus_factory=bus_factory,
            sleep_fn=sleep_fn,
        )

    ensure_parent_dir(args.output)
    with open(args.output, "w") as handle:
        json.dump(log, handle, indent=2, sort_keys=True)
    append_csv_log(DEFAULT_CSV_LOG_PATH, log)
    return log


def load_validation_context(args):
    model = load_urdf_model(args.urdf)
    scene_geometry = load_scene_geometry(args.scene)
    calibration = load_joint_calibration(args.joint_calibration)
    joint_limits = load_joint_limits(args.joint_limits) if args.joint_limits else None
    joint_safety_limits = load_joint_safety_limits(args.joint_safety_limits)
    tool_frames = load_tool_frames(args.tool_frames)
    tool_frame = get_tool_frame(tool_frames, args.tcp_frame)
    tool_frame_description = describe_tool_frame(tool_frame, fallback_name=args.end_link)
    joint_limit_bounds = resolve_joint_limit_bounds(
        model,
        limit_source=args.limit_source,
        joint_limits=joint_limits,
        joint_safety_limits=joint_safety_limits,
        calibration=calibration,
        end_link=args.end_link,
    )
    home_pose_ticks = None
    home_seed = None
    if args.home_pose:
        home_pose_ticks = load_pose_ticks(args.home_pose)
        home_seed = convert_pose_ticks_to_urdf_radians(home_pose_ticks, calibration)
    locked_joint_positions_rad, locked_joint_ticks, locked_joint_sources = resolve_locked_joints(
        args,
        calibration,
        home_pose_ticks,
    )
    return {
        "model": model,
        "scene_geometry": scene_geometry,
        "calibration": calibration,
        "joint_safety_limits": joint_safety_limits,
        "tool_frame": tool_frame,
        "tool_frame_description": tool_frame_description,
        "joint_limit_bounds": joint_limit_bounds,
        "end_link": args.end_link,
        "home_pose_ticks": home_pose_ticks,
        "home_seed": home_seed,
        "locked_joint_positions_rad": locked_joint_positions_rad,
        "locked_joint_ticks": locked_joint_ticks,
        "locked_joint_sources": locked_joint_sources,
    }


def select_target_from_args(context, args):
    return select_target(
        context["scene_geometry"],
        square=args.square,
        capture=bool(args.capture),
        target_type=args.target_type,
        target_world=args.target_world,
        target_home_pose=bool(getattr(args, "target_home_pose", False)),
        target_world_offset_from_home=getattr(args, "target_world_offset_from_home", None),
        home_pose_ticks=context.get("home_pose_ticks"),
        home_joint_positions_rad=context.get("home_seed"),
        model=context["model"],
        tool_frame=context["tool_frame"],
        end_link=args.end_link,
        above_board_offset_m=args.above_board_offset_m,
        pick_offset_m=args.pick_offset_m,
        capture_above_offset_m=args.capture_above_offset_m,
    )


def build_saved_home_metadata(scene_geometry, model, home_pose_ticks, home_joint_positions_rad, tool_frame, end_link):
    if home_pose_ticks is None or home_joint_positions_rad is None:
        raise SinglePoseValidationError("Home target modes require --home-pose and --joint-calibration.")
    tcp_robot = compute_tcp_transform(
        model,
        home_joint_positions_rad,
        end_link=end_link,
        tool_frame=tool_frame,
    )[:3, 3].copy()
    tcp_world = robot_base_point_to_world(tcp_robot, scene_geometry)
    return {
        "saved_home_joint_ticks": dict((joint, int(home_pose_ticks[joint])) for joint in ARM_JOINTS if joint in home_pose_ticks),
        "saved_home_joint_angles_rad": arm_joint_mapping(home_joint_positions_rad),
        "saved_home_tcp_robot_xyz_m": [float(value) for value in tcp_robot],
        "saved_home_tcp_world_xyz_m": [float(value) for value in tcp_world],
    }


def select_target(
        scene_geometry,
        square=None,
        capture=False,
        target_type=None,
        target_world=None,
        target_home_pose=False,
        target_world_offset_from_home=None,
        home_pose_ticks=None,
        home_joint_positions_rad=None,
        model=None,
        tool_frame=None,
        end_link=DEFAULT_END_LINK,
        above_board_offset_m=0.080,
        pick_offset_m=0.030,
        capture_above_offset_m=0.080):
    if target_home_pose or target_world_offset_from_home is not None:
        metadata = build_saved_home_metadata(
            scene_geometry,
            model,
            home_pose_ticks,
            home_joint_positions_rad,
            tool_frame,
            end_link,
        )
        if target_home_pose:
            target = {
                "target_mode": "home_pose",
                "target_name": "home_pose",
                "target_type": "home_pose",
                "square": None,
                "target_robot_xyz_m": list(metadata["saved_home_tcp_robot_xyz_m"]),
                "target_world_xyz_m": list(metadata["saved_home_tcp_world_xyz_m"]),
            }
            target.update(metadata)
            return target
        offset = np.asarray(target_world_offset_from_home, dtype=float)
        if offset.shape != (3,):
            raise SinglePoseValidationError("--target-world-offset-from-home requires exactly three numbers.")
        home_world = np.asarray(metadata["saved_home_tcp_world_xyz_m"], dtype=float)
        target_world_from_home = home_world + offset
        target = {
            "target_mode": "world_offset_from_home",
            "target_name": "home_world_offset",
            "target_type": "world_offset_from_home",
            "square": None,
            "target_world_xyz_m": [float(value) for value in target_world_from_home],
            "requested_offset_world_m": [float(value) for value in offset],
        }
        target.update(metadata)
        return target

    if target_world is not None:
        vector = np.asarray(target_world, dtype=float)
        if vector.shape != (3,):
            raise SinglePoseValidationError("--target-world requires exactly three numbers.")
        return {
            "target_mode": "explicit_world",
            "target_name": "explicit_world",
            "target_type": "explicit_world",
            "square": None,
            "target_world_xyz_m": [float(value) for value in vector],
        }

    if target_type not in ("above", "surface"):
        raise SinglePoseValidationError("--target-type must be above or surface.")
    if capture and square:
        raise SinglePoseValidationError("Use either --capture or --square, not both.")
    if not capture and not square:
        raise SinglePoseValidationError("Provide exactly one target mode: --square, --capture, or --target-world.")

    internal_type = ("capture_%s" if capture else "square_%s") % target_type
    targets = generate_targets(
        scene_geometry,
        above_board_offset_m=above_board_offset_m,
        pick_offset_m=pick_offset_m,
        capture_above_offset_m=capture_above_offset_m,
    )
    for target in targets:
        if target["target_type"] != internal_type:
            continue
        if capture or target.get("square") == square:
            return {
                "target_mode": "capture" if capture else "square",
                "target_name": target["target_name"],
                "target_type": target_type,
                "square": target.get("square") or None,
                "target_world_xyz_m": [float(target["x_m"]), float(target["y_m"]), float(target["z_m"])],
                "reachability_target_type": target["target_type"],
            }
    raise SinglePoseValidationError("Target was not found for square=%r capture=%r type=%r." % (square, capture, target_type))


class MinimalIKResult(object):
    def __init__(self, success, status, final_xyz_robot, error_m, iterations, joint_positions_rad):
        self.success = bool(success)
        self.status = str(status)
        self.final_xyz_robot = np.asarray(final_xyz_robot, dtype=float)
        self.error_m = float(error_m)
        self.iterations = int(iterations)
        self.joint_positions_rad = dict(joint_positions_rad)


def build_home_pose_ik_result(context, target_robot):
    home_seed = context.get("home_seed")
    if home_seed is None:
        raise SinglePoseValidationError("--target-home-pose requires --home-pose.")
    final_xyz_robot = compute_tcp_transform(
        context["model"],
        home_seed,
        end_link=context.get("end_link", DEFAULT_END_LINK),
        tool_frame=context["tool_frame"],
    )[:3, 3].copy()
    error_m = float(np.linalg.norm(np.asarray(target_robot, dtype=float) - final_xyz_robot))
    return MinimalIKResult(
        success=True,
        status="saved_home_pose",
        final_xyz_robot=final_xyz_robot,
        error_m=error_m,
        iterations=0,
        joint_positions_rad=home_seed,
    )


def solve_single_target_ik(context, target_robot, args, ik_solver=None):
    workspace_seed = None
    if int(args.workspace_seed_samples) > 0:
        workspace_samples = sample_position_workspace(
            context["model"],
            context["joint_limit_bounds"],
            sample_count=args.workspace_seed_samples,
            seed=args.seed,
            end_link=args.end_link,
            tool_frame=context["tool_frame"],
        )
        workspace_seed = find_nearest_workspace_seed(target_robot, workspace_samples)["joint_positions_rad"]
    solver = ik_solver or solve_position_ik_multi_seed
    return solver(
        context["model"],
        target_robot,
        context["joint_limit_bounds"],
        end_link=args.end_link,
        tool_frame=context["tool_frame"],
        home_joint_positions_rad=context["home_seed"],
        workspace_seed_joint_positions_rad=workspace_seed,
        random_seeds=args.random_seeds,
        seed=args.seed,
        max_iters=args.max_iters,
        tolerance_m=args.tolerance_m,
        damping=args.damping,
        step_scale=args.step_scale,
        locked_joint_positions_rad=context["locked_joint_positions_rad"],
    )


def resolve_locked_joints(args, calibration, home_pose_ticks):
    locked_joint_positions_rad = {}
    locked_joint_ticks = {}
    locked_joint_sources = {}

    if bool(getattr(args, "lock_wrist_roll_home", False)):
        if home_pose_ticks is None:
            raise SinglePoseValidationError("--lock-wrist-roll-home requires --home-pose.")
        if home_pose_ticks.get("wrist_roll") is None:
            raise SinglePoseValidationError("Saved home pose is missing wrist_roll tick.")
        wrist_roll_tick = int(home_pose_ticks["wrist_roll"])
        set_locked_joint(
            locked_joint_positions_rad,
            locked_joint_ticks,
            locked_joint_sources,
            "wrist_roll",
            tick_to_angle_rad("wrist_roll", wrist_roll_tick, calibration),
            wrist_roll_tick,
            "home_pose",
        )

    for raw_entry in list(getattr(args, "lock_joint", None) or []):
        joint_key, separator, raw_value = str(raw_entry).partition("=")
        if separator != "=" or not joint_key or not raw_value:
            raise SinglePoseValidationError("--lock-joint must use joint=value or joint_rad=value syntax.")
        if joint_key.endswith("_rad"):
            joint_name = joint_key[:-4]
            joint_value_rad = float(raw_value)
            set_locked_joint(
                locked_joint_positions_rad,
                locked_joint_ticks,
                locked_joint_sources,
                joint_name,
                joint_value_rad,
                angle_rad_to_tick(joint_name, joint_value_rad, calibration),
                "cli_rad",
            )
        else:
            joint_name = joint_key
            joint_tick = int(raw_value)
            set_locked_joint(
                locked_joint_positions_rad,
                locked_joint_ticks,
                locked_joint_sources,
                joint_name,
                tick_to_angle_rad(joint_name, joint_tick, calibration),
                joint_tick,
                "cli_tick",
            )

    return locked_joint_positions_rad, locked_joint_ticks, locked_joint_sources


def set_locked_joint(locked_joint_positions_rad, locked_joint_ticks, locked_joint_sources, joint_name, angle_rad, tick_value, source):
    joint_name = str(joint_name)
    if joint_name not in ARM_JOINTS:
        raise SinglePoseValidationError("Locked joint %s is not part of the arm IK chain." % joint_name)
    angle_rad = float(angle_rad)
    tick_value = int(tick_value)
    existing_angle = locked_joint_positions_rad.get(joint_name)
    existing_tick = locked_joint_ticks.get(joint_name)
    if existing_angle is not None and abs(float(existing_angle) - angle_rad) > 1.0e-9:
        raise SinglePoseValidationError("Conflicting locked joint values for %s." % joint_name)
    if existing_tick is not None and int(existing_tick) != tick_value:
        raise SinglePoseValidationError("Conflicting locked joint ticks for %s." % joint_name)
    locked_joint_positions_rad[joint_name] = angle_rad
    locked_joint_ticks[joint_name] = tick_value
    locked_joint_sources[joint_name] = str(source)


def joint_angles_to_ticks(joint_positions_rad, calibration):
    ticks = {}
    for joint_name in ARM_JOINTS:
        if joint_name not in joint_positions_rad:
            continue
        ticks[joint_name] = int(angle_rad_to_tick(joint_name, joint_positions_rad[joint_name], calibration))
    return ticks


def arm_joint_mapping(mapping):
    return dict((joint_name, float(mapping[joint_name])) for joint_name in ARM_JOINTS if joint_name in mapping)


def calculate_safety_limit_margins_ticks(target_ticks, joint_safety_limits):
    joints = joint_safety_limits.get("joints") or {}
    margins = {}
    for joint_name in ARM_JOINTS:
        tick = target_ticks.get(joint_name)
        entry = joints.get(joint_name) or {}
        if tick is None or entry.get("min_tick") is None or entry.get("max_tick") is None:
            margins[joint_name] = None
            continue
        lower = int(tick) - int(entry["min_tick"])
        upper = int(entry["max_tick"]) - int(tick)
        margins[joint_name] = {
            "lower": int(lower),
            "upper": int(upper),
            "min": int(min(lower, upper)),
        }
    return margins


def convert_margin_ticks_to_deg(margins_ticks, calibration):
    ticks_per_rev = float(calibration["ticks_per_rev"])
    result = {}
    for joint_name, margins in margins_ticks.items():
        if margins is None:
            result[joint_name] = None
            continue
        result[joint_name] = dict(
            (key, float(value) * 360.0 / ticks_per_rev)
            for key, value in margins.items()
        )
    return result


def build_static_safety_checks(ik_success, target_ticks, joint_safety_limits):
    checks = []
    checks.append(make_check("ik_success", bool(ik_success), "IK failed."))
    missing = [joint for joint in ARM_JOINTS if target_ticks.get(joint) is None]
    checks.append(make_check("all_target_ticks_present", not missing, "Missing target ticks: %s" % ",".join(missing)))
    checks.extend(validate_target_ticks_in_limits(target_ticks, joint_safety_limits))
    checks.append(make_check("gripper_excluded", "gripper" not in target_ticks, "Gripper must not be commanded."))
    return checks


def validate_target_ticks_in_limits(target_ticks, joint_safety_limits):
    checks = []
    joints = joint_safety_limits.get("joints") or {}
    for joint_name in ARM_JOINTS:
        tick = target_ticks.get(joint_name)
        entry = joints.get(joint_name) or {}
        if tick is None:
            checks.append(make_check("%s_target_tick_present" % joint_name, False, "%s target tick is missing." % joint_name))
            continue
        if entry.get("min_tick") is None or entry.get("max_tick") is None:
            checks.append(make_check("%s_safety_limits_present" % joint_name, False, "%s safety limits are missing." % joint_name))
            continue
        minimum = int(entry["min_tick"])
        maximum = int(entry["max_tick"])
        ok = minimum <= int(tick) <= maximum
        checks.append(
            make_check(
                "%s_inside_safety_limits" % joint_name,
                ok,
                "%s target tick %s outside safety limits [%s, %s]." % (joint_name, tick, minimum, maximum),
            )
        )
    return checks


def validate_motion_deltas(current_ticks, target_ticks, max_joint_delta_ticks, max_total_l1_delta_ticks, allow_large_delta):
    deltas = {}
    checks = []
    missing = []
    for joint_name in ARM_JOINTS:
        if current_ticks.get(joint_name) is None or target_ticks.get(joint_name) is None:
            missing.append(joint_name)
            continue
        deltas[joint_name] = int(target_ticks[joint_name]) - int(current_ticks[joint_name])
    checks.append(make_check("current_ticks_present", not missing, "Missing current ticks: %s" % ",".join(missing)))
    if missing:
        return deltas, checks

    max_abs_delta = max(abs(delta) for delta in deltas.values()) if deltas else 0
    total_l1 = sum(abs(delta) for delta in deltas.values())
    if allow_large_delta:
        checks.append(make_check("max_joint_delta", True, "Large delta override supplied."))
        checks.append(make_check("max_total_l1_delta", True, "Large delta override supplied."))
    else:
        checks.append(
            make_check(
                "max_joint_delta",
                max_abs_delta <= int(max_joint_delta_ticks),
                "Max joint delta %s exceeds limit %s." % (max_abs_delta, max_joint_delta_ticks),
            )
        )
        checks.append(
            make_check(
                "max_total_l1_delta",
                total_l1 <= int(max_total_l1_delta_ticks),
                "Total L1 delta %s exceeds limit %s." % (total_l1, max_total_l1_delta_ticks),
            )
        )
    return deltas, checks


def execute_single_pose(args, log, context, bus_factory=None, sleep_fn=None):
    sleep_fn = sleep_fn or time.sleep
    if args.confirm != CONFIRM_TEXT:
        set_abort(log, "Execute mode requires --confirm %s." % CONFIRM_TEXT)
        return log
    if log.get("abort_reason"):
        return log

    bus = None
    try:
        bus, config = build_execution_bus(args, bus_factory=bus_factory)
        servo_ids = arm_servo_ids(config)
        if sorted(servo_ids.keys()) != sorted(ARM_JOINTS):
            set_abort(log, "Missing servo IDs for one or more arm joints.")
            return log
        current_ticks = read_current_ticks(bus, servo_ids)
        log["current_ticks_before"] = current_ticks
        attach_path_validation_from_reference_ticks(log, context, args, current_ticks, "live_readback")
        if should_enforce_board_clearance(args) and not bool(log["path_validation"].get("passed")):
            set_abort(log, log["path_validation"].get("failure_reason") or "Board-clearance path validation failed.")
            return log
        deltas, delta_checks = validate_motion_deltas(
            current_ticks,
            log["target_ticks"],
            args.max_joint_delta_ticks,
            args.max_total_l1_delta_ticks,
            bool(args.allow_large_delta),
        )
        log["tick_deltas"] = deltas
        log["safety_checks"].extend(delta_checks)
        eeprom_checks = validate_eeprom_limits_if_available(bus, servo_ids, log["target_ticks"])
        log["safety_checks"].extend(eeprom_checks)
        if not all_checks_ok(log["safety_checks"]):
            set_abort(log, first_failed_check_reason(log["safety_checks"]))
            return log

        print_motion_summary(current_ticks, log["target_ticks"], deltas)
        waypoints = build_interpolated_tick_waypoints(current_ticks, log["target_ticks"], args.speed_scale)
        log["waypoint_count"] = len(waypoints)
        for waypoint in waypoints:
            for joint_name in ARM_JOINTS:
                bus.write_goal_position(servo_ids[joint_name], waypoint[joint_name])
            sleep_fn(inter_waypoint_delay_s(args.speed_scale))
        sleep_fn(float(args.settle_time_s))
        final_ticks = read_current_ticks(bus, servo_ids)
        log["final_ticks_after"] = final_ticks
        readback_checks = validate_readback(final_ticks, log["target_ticks"], args.readback_tolerance_ticks)
        log["safety_checks"].extend(readback_checks)
        if not all_checks_ok(readback_checks):
            set_abort(log, first_failed_check_reason(readback_checks))
            log["command_sent"] = True
            return log
        log["command_sent"] = True
        return log
    except Exception as exc:
        set_abort(log, str(exc))
        return log
    finally:
        if bus is not None:
            bus.close()


def attach_motion_safety_metadata(log, context, args, result):
    board_top = board_top_z_m(context["scene_geometry"])
    board_clearance = float(getattr(args, "board_clearance_m", 0.060))
    low_zone = low_zone_z_m(context["scene_geometry"], board_clearance)
    transit_clearance = float(getattr(args, "transit_clearance_m", 0.090))
    approach_axis_local, defaulted = resolve_approach_axis_local(context["tool_frame"])
    tool_defaulted = bool(context["tool_frame"].get("approach_axis_local_defaulted", False)) if context.get("tool_frame") else bool(defaulted)
    robot_T_tcp = compute_tcp_transform(
        context["model"],
        result.joint_positions_rad,
        end_link=context.get("end_link", DEFAULT_END_LINK),
        tool_frame=context["tool_frame"],
    )
    world_T_tcp = np.dot(np.asarray(context["scene_geometry"]["world_T_robot_base"], dtype=float), robot_T_tcp)
    axis_world = approach_axis_world(world_T_tcp, approach_axis_local)
    tilt_deg = approach_tilt_deg(axis_world)
    max_tilt_deg = selected_max_approach_tilt_deg(args, log.get("square"))
    approach_check = make_approach_angle_check(tilt_deg, max_tilt_deg)
    approach_check["enforced"] = bool(getattr(args, "enforce_approach_angle", False))

    log["board_top_z_m"] = float(board_top)
    log["board_clearance_m"] = float(board_clearance)
    log["low_zone_z_m"] = float(low_zone)
    log["transit_clearance_m"] = float(transit_clearance)
    log["xy_motion_epsilon_m"] = float(getattr(args, "xy_motion_epsilon_m", 0.005))
    log["path_samples"] = int(getattr(args, "path_samples", 25))
    log["path_validation"] = unavailable_path_validation("not evaluated yet", low_zone, getattr(args, "path_samples", 25))
    log["path_validation_note"] = "Approximate joint-space FK safety gate only; not a full collision checker."
    log["approach_axis_local"] = [float(value) for value in approach_axis_local]
    log["approach_axis_local_defaulted"] = bool(tool_defaulted)
    log["approach_axis_local_warning"] = "approach_axis_local missing; defaulted to [0, 0, -1]." if tool_defaulted else None
    log["approach_axis_world"] = [float(value) for value in axis_world]
    log["approach_tilt_deg"] = float(tilt_deg)
    log["max_approach_tilt_deg"] = float(max_tilt_deg)
    log["max_edge_approach_tilt_deg"] = float(getattr(args, "max_edge_approach_tilt_deg", 25.0))
    log["approach_angle_check"] = approach_check


def attach_path_validation_from_reference_ticks(log, context, args, reference_ticks, reference_source):
    low_zone = float(log.get("low_zone_z_m", low_zone_z_m(context["scene_geometry"], getattr(args, "board_clearance_m", 0.060))))
    if reference_ticks is None:
        log["path_validation"] = unavailable_path_validation("reference ticks unavailable", low_zone, getattr(args, "path_samples", 25))
        log["path_validation"]["current_ticks_source"] = str(reference_source)
        return log["path_validation"]
    try:
        current_joint_positions_rad = convert_pose_ticks_to_urdf_radians(reference_ticks, context["calibration"])
        target_joint_positions_rad = convert_pose_ticks_to_urdf_radians(log["target_ticks"], context["calibration"])
        missing = [joint for joint in ARM_JOINTS if joint not in current_joint_positions_rad or joint not in target_joint_positions_rad]
        if missing:
            summary = unavailable_path_validation("missing joint radians: %s" % ",".join(missing), low_zone, getattr(args, "path_samples", 25))
        else:
            summary = validate_joint_interpolated_tcp_path(
                context["model"],
                context["scene_geometry"],
                current_joint_positions_rad,
                target_joint_positions_rad,
                ARM_JOINTS,
                context.get("end_link", DEFAULT_END_LINK),
                context["tool_frame"],
                low_zone,
                float(getattr(args, "xy_motion_epsilon_m", 0.005)),
                int(getattr(args, "path_samples", 25)),
            )
    except Exception as exc:
        summary = unavailable_path_validation(str(exc), low_zone, getattr(args, "path_samples", 25))
    summary["current_ticks_source"] = str(reference_source)
    log["path_validation"] = summary
    return summary


def unavailable_path_validation(reason, low_zone, samples_count):
    return {
        "xy_delta_m": None,
        "min_z_m": None,
        "low_zone_z_m": float(low_zone),
        "passed": False,
        "failure_reason": str(reason),
        "samples_count": int(samples_count),
        "xy_changing": None,
        "current_tcp_world_xyz_m": None,
        "target_tcp_world_xyz_m": None,
    }


def should_enforce_board_clearance(args):
    configured = getattr(args, "enforce_board_clearance", None)
    if configured is None:
        return bool(getattr(args, "execute", False))
    return bool(configured) and bool(getattr(args, "execute", False))


def selected_max_approach_tilt_deg(args, square):
    if is_edge_square(square):
        return float(getattr(args, "max_edge_approach_tilt_deg", 25.0))
    return float(getattr(args, "max_approach_tilt_deg", 15.0))


def is_edge_square(square):
    if not square:
        return False
    square = str(square).lower()
    if len(square) < 2:
        return False
    return square[0] in ("a", "h") or square[1:] in ("1", "8")


def build_execution_bus(args, bus_factory=None):
    if bus_factory is not None:
        return bus_factory(args)
    from chess_robot.robot.servo_bus import build_servo_bus
    from chess_robot.robot.servo_bus import load_robot_config

    config = load_robot_config(args.config)
    servo_config = config.get("servo_bus") or {}
    backend_name = servo_config.get("backend")
    if backend_name == "mock" and servo_config.get("feetech"):
        backend_name = "feetech"
    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=False,
        backend_name=backend_name,
    )
    return bus, config


def arm_servo_ids(config):
    joints = config.get("joints") or {}
    servo_ids = {}
    for joint_name in ARM_JOINTS:
        entry = joints.get(joint_name) or {}
        if entry.get("servo_id") is not None:
            servo_ids[joint_name] = int(entry["servo_id"])
    return servo_ids


def read_current_ticks(bus, servo_ids):
    ticks = {}
    for joint_name in ARM_JOINTS:
        position = bus.read_position(servo_ids[joint_name])
        if position is None:
            raise SinglePoseValidationError("Current position readback failed for %s." % joint_name)
        ticks[joint_name] = int(position)
    return ticks


def validate_eeprom_limits_if_available(bus, servo_ids, target_ticks):
    checks = []
    for joint_name in ARM_JOINTS:
        servo_id = servo_ids[joint_name]
        minimum = None
        maximum = None
        try:
            minimum = bus.read_register(servo_id, 9, 2)
            maximum = bus.read_register(servo_id, 11, 2)
        except Exception:
            checks.append(make_check("%s_eeprom_limits_available" % joint_name, True, "EEPROM limits unavailable; skipped."))
            continue
        if minimum is None or maximum is None:
            checks.append(make_check("%s_eeprom_limits_available" % joint_name, True, "EEPROM limits unavailable; skipped."))
            continue
        tick = int(target_ticks[joint_name])
        low = min(int(minimum), int(maximum))
        high = max(int(minimum), int(maximum))
        checks.append(
            make_check(
                "%s_inside_eeprom_limits" % joint_name,
                low <= tick <= high,
                "%s target tick %s outside EEPROM limits [%s, %s]." % (joint_name, tick, low, high),
            )
        )
    return checks


def build_interpolated_tick_waypoints(current_ticks, target_ticks, speed_scale):
    speed_scale = max(0.05, min(1.0, float(speed_scale)))
    max_step_ticks = max(5, int(round(80.0 * speed_scale)))
    max_delta = max(abs(int(target_ticks[joint]) - int(current_ticks[joint])) for joint in ARM_JOINTS)
    step_count = max(1, int(math.ceil(float(max_delta) / float(max_step_ticks))))
    waypoints = []
    for step_index in range(1, step_count + 1):
        fraction = float(step_index) / float(step_count)
        waypoint = {}
        for joint_name in ARM_JOINTS:
            start = int(current_ticks[joint_name])
            end = int(target_ticks[joint_name])
            waypoint[joint_name] = int(round(start + ((end - start) * fraction)))
        waypoints.append(waypoint)
    return waypoints


def inter_waypoint_delay_s(speed_scale):
    speed_scale = max(0.05, min(1.0, float(speed_scale)))
    return max(0.03, 0.12 * (1.0 - speed_scale))


def validate_readback(final_ticks, target_ticks, tolerance_ticks):
    checks = []
    for joint_name in ARM_JOINTS:
        final_tick = final_ticks.get(joint_name)
        target_tick = target_ticks.get(joint_name)
        ok = final_tick is not None and abs(int(final_tick) - int(target_tick)) <= int(tolerance_ticks)
        checks.append(
            make_check(
                "%s_final_readback_tolerance" % joint_name,
                ok,
                "%s final tick %s outside target %s +/- %s." % (joint_name, final_tick, target_tick, tolerance_ticks),
            )
        )
    return checks


def build_base_log(
        mode,
        timestamp,
        target,
        target_robot,
        result,
        final_tcp_world,
        joint_angles_rad,
        joint_angles_deg,
        target_ticks,
        safety_margins_ticks,
        safety_margins_deg,
        safety_checks,
        tool_frame_description,
        locked_joint_positions_rad,
        locked_joint_ticks,
        locked_joint_sources):
    log = {
        "mode": mode,
        "target_mode": target["target_mode"],
        "square": target.get("square"),
        "target_type": target["target_type"],
        "target_name": target["target_name"],
        "target_world_xyz_m": target["target_world_xyz_m"],
        "target_robot_xyz_m": [float(value) for value in np.asarray(target_robot, dtype=float)],
        "tcp_frame": tool_frame_description["tcp_frame"],
        "tool_offset_xyz_m": tool_frame_description["tool_offset_xyz_m"],
        "ik_success": bool(result.success),
        "ik_status": str(result.status),
        "ik_error_m": float(result.error_m),
        "ik_iterations": int(result.iterations),
        "final_tcp_robot_xyz_m": [float(value) for value in np.asarray(result.final_xyz_robot, dtype=float)],
        "final_tcp_world_xyz_m": [float(value) for value in np.asarray(final_tcp_world, dtype=float)],
        "joint_angles_rad": joint_angles_rad,
        "joint_angles_deg": joint_angles_deg,
        "target_ticks": target_ticks,
        "locked_joints_rad": arm_joint_mapping(locked_joint_positions_rad),
        "locked_joints_ticks": dict((joint_name, int(value)) for joint_name, value in locked_joint_ticks.items()),
        "locked_joint_sources": dict((joint_name, str(value)) for joint_name, value in locked_joint_sources.items()),
        "current_ticks_before": None,
        "final_ticks_after": None,
        "tick_deltas": None,
        "safety_limit_margins_ticks": safety_margins_ticks,
        "safety_limit_margins_deg": safety_margins_deg,
        "safety_checks": safety_checks,
        "abort_reason": None,
        "command_sent": False,
        "timestamp": timestamp,
    }
    for key in (
        "saved_home_joint_ticks",
        "saved_home_joint_angles_rad",
        "saved_home_tcp_robot_xyz_m",
        "saved_home_tcp_world_xyz_m",
        "requested_offset_world_m",
    ):
        if target.get(key) is not None:
            log[key] = target[key]
    if target.get("saved_home_joint_ticks") is not None:
        log["saved_home_tick_deltas"] = calculate_tick_deltas(
            target_ticks,
            target["saved_home_joint_ticks"],
        )
    return log


def make_check(name, ok, reason):
    return {
        "name": str(name),
        "ok": bool(ok),
        "reason": None if bool(ok) else str(reason),
    }


def all_checks_ok(checks):
    return all(bool(check.get("ok")) for check in checks)


def first_failed_check_reason(checks):
    for check in checks:
        if not bool(check.get("ok")):
            return check.get("reason") or check.get("name") or "safety check failed"
    return None


def set_abort(log, reason):
    log["abort_reason"] = str(reason)
    log["command_sent"] = False


def calculate_tick_deltas(target_ticks, reference_ticks):
    deltas = {}
    for joint_name in ARM_JOINTS:
        if target_ticks.get(joint_name) is None or reference_ticks.get(joint_name) is None:
            continue
        deltas[joint_name] = int(target_ticks[joint_name]) - int(reference_ticks[joint_name])
    return deltas


def append_csv_log(path, log):
    ensure_parent_dir(path)
    exists = os.path.exists(path)
    row = {
        "timestamp": log["timestamp"],
        "mode": log["mode"],
        "target_name": log["target_name"],
        "ik_success": bool(log["ik_success"]),
        "ik_error_mm": float(log["ik_error_m"]) * 1000.0,
        "command_sent": bool(log["command_sent"]),
        "abort_reason": log.get("abort_reason") or "",
    }
    target_ticks = log.get("target_ticks") or {}
    for joint_name in ARM_JOINTS:
        row["%s_tick" % joint_name] = target_ticks.get(joint_name)
    with open(path, "a") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def print_motion_summary(current_ticks, target_ticks, deltas):
    print("Motion summary:")
    for joint_name in ARM_JOINTS:
        print(
            "  %s: current=%s target=%s delta=%+d"
            % (joint_name, current_ticks[joint_name], target_ticks[joint_name], deltas[joint_name])
        )


def ensure_parent_dir(path):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)


def utc_timestamp():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
