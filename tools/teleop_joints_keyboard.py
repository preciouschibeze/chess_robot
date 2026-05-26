#!/usr/bin/env python3
"""Line-command keyboard teleoperation for wrist-camera hand-eye setup.

The interface is intentionally conservative over SSH: every command is a line
of text, every accepted jog validates one joint target, and dry-run mode never
writes a servo goal position.
"""

from __future__ import absolute_import

import argparse
import datetime
import json
import os
import select
import sys
import termios
import time
import tty

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot import safety
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import tick_to_angle_rad
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.servo_bus import BackendUnavailable
from chess_robot.robot.servo_bus import ServoBusError
from chess_robot.robot.servo_bus import build_servo_bus
from chess_robot.robot.servo_bus import load_robot_config
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.tool_frames import describe_tool_frame
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import load_urdf_model


DEFAULT_CONFIG = os.path.join(REPO_ROOT, "configs", "robot.yaml")
DEFAULT_JOINT_CALIBRATION = os.path.join(
    REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml"
)
DEFAULT_JOINT_SAFETY_LIMITS = os.path.join(
    REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml"
)
DEFAULT_URDF = os.path.join(
    REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf"
)
DEFAULT_TOOL_FRAMES = os.path.join(
    REPO_ROOT, "data", "calibration", "gripper", "tool_frames.yaml"
)
DEFAULT_POSE_OUTPUT_DIR = os.path.join(
    REPO_ROOT, "data", "calibration", "hand_eye", "teleop_poses"
)
DEFAULT_LOG_PATH = os.path.join(REPO_ROOT, "data", "logs", "keyboard_teleop.log")

CONFIRMATION_TEXT = "TELEOP"
DEFAULT_ACTIVE_JOINT = "base_yaw"
ARM_DEFAULT_STEP_TICKS = 10
WRIST_ROLL_DEFAULT_STEP_TICKS = 5

KEY_BINDINGS = {
    "1": "base_yaw",
    "2": "shoulder_pitch",
    "3": "elbow_pitch",
    "4": "wrist_pitch",
    "5": "wrist_roll",
    "6": "gripper",
}

HOTKEY_JOG_BINDINGS = {
    "a": ("base_yaw", -1),
    "d": ("base_yaw", 1),
    "z": ("shoulder_pitch", -1),
    "x": ("shoulder_pitch", 1),
    "c": ("elbow_pitch", -1),
    "v": ("elbow_pitch", 1),
    "b": ("wrist_pitch", -1),
    "n": ("wrist_pitch", 1),
    "j": ("wrist_roll", -1),
    "k": ("wrist_roll", 1),
    "u": ("gripper", -1),
    "i": ("gripper", 1),
}

DISPLAY_JOINTS = (
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
)


class TeleopError(RuntimeError):
    pass


class CommandRejected(ValueError):
    pass


class TeleopSession(object):
    def __init__(self, mode, session_id=None):
        self.mode = str(mode)
        self.session_id = session_id or make_session_id()
        self.pose_index = 0

    def next_pose_id(self):
        self.pose_index += 1
        return self.pose_index


def make_session_id():
    now = datetime.datetime.utcnow()
    stamp = now.strftime("%Y%m%d_%H%M%S_") + "%03d" % (now.microsecond // 1000)
    return "teleop_%s" % stamp


def build_parser():
    parser = argparse.ArgumentParser(
        description="Safe line-command joint teleoperation for hand-eye calibration poses."
    )
    parser.add_argument("--execute", action="store_true", help="Allow validated real servo writes.")
    parser.add_argument("--confirm-text", help="Required exact text for --execute: TELEOP")
    parser.add_argument("--dry-run", action="store_true", default=True, help="Dry-run mode is the default.")
    parser.add_argument("--mode", choices=("line", "hotkey"), default="line")
    parser.add_argument("--min-command-interval-sec", type=float, default=0.15)
    parser.add_argument("--step-ticks", type=int, default=ARM_DEFAULT_STEP_TICKS)
    parser.add_argument("--wrist-roll-step-ticks", type=int, default=WRIST_ROLL_DEFAULT_STEP_TICKS)
    parser.add_argument("--max-step-ticks", type=int, default=100)
    parser.add_argument("--allow-gripper", action="store_true", default=False)
    parser.add_argument("--tcp-frame", default="gripper_frame")
    parser.add_argument("--pose-output-dir", default=DEFAULT_POSE_OUTPUT_DIR)
    parser.add_argument("--log-path", default=DEFAULT_LOG_PATH)
    parser.add_argument("--no-torque-on-start", action="store_true", default=False)
    parser.add_argument("--torque-off-on-exit", action="store_true", default=False)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION)
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS)
    parser.add_argument("--urdf", default=DEFAULT_URDF)
    parser.add_argument("--tool-frames", default=DEFAULT_TOOL_FRAMES)
    return parser


def clamp_step_ticks(value, max_step_ticks):
    try:
        step = int(value)
    except (TypeError, ValueError):
        raise CommandRejected("Step size must be an integer.")
    try:
        maximum = int(max_step_ticks)
    except (TypeError, ValueError):
        maximum = 100
    if maximum <= 0:
        maximum = 100
    if step < 1:
        return 1
    if step > maximum:
        return maximum
    return step


def validate_args(args):
    args.max_step_ticks = clamp_step_ticks(args.max_step_ticks, args.max_step_ticks)
    args.step_ticks = clamp_step_ticks(args.step_ticks, args.max_step_ticks)
    args.wrist_roll_step_ticks = clamp_step_ticks(args.wrist_roll_step_ticks, args.max_step_ticks)
    try:
        args.min_command_interval_sec = float(args.min_command_interval_sec)
    except (TypeError, ValueError):
        raise TeleopError("--min-command-interval-sec must be numeric.")
    if args.min_command_interval_sec < 0.0:
        raise TeleopError("--min-command-interval-sec must be >= 0.")
    args.dry_run = not bool(args.execute)
    if args.execute and args.confirm_text != CONFIRMATION_TEXT:
        raise TeleopError(
            "Real movement requires --execute --confirm-text TELEOP."
        )
    return args


def build_joint_records(config, calibration, safety_limits, allow_gripper):
    configured_joints = config.get("joints") or {}
    limit_entries = safety_limits.get("joints") or {}
    records = {}

    for user_joint in calibration.get("joint_order", []):
        calibration_entry = calibration["joints"][user_joint]
        urdf_joint = calibration_entry["urdf_joint"]
        joint_config = configured_joints.get(urdf_joint)
        limit_entry = limit_entries.get(urdf_joint)
        record = {
            "user_joint": user_joint,
            "urdf_joint": urdf_joint,
            "servo_id": None,
            "limits": None,
            "calibrated": True,
            "joggable": False,
            "reason": None,
        }
        if not isinstance(joint_config, dict) or joint_config.get("servo_id") is None:
            record["reason"] = "missing configured servo ID"
        elif not isinstance(limit_entry, dict):
            record["reason"] = "missing joint safety limits"
        else:
            record["servo_id"] = int(joint_config["servo_id"])
            record["limits"] = {
                "min": int(limit_entry["min_tick"]),
                "max": int(limit_entry["max_tick"]),
            }
            record["joggable"] = True
        records[user_joint] = record

    if allow_gripper:
        joint_config = configured_joints.get("gripper")
        record = {
            "user_joint": "gripper",
            "urdf_joint": "gripper",
            "servo_id": None,
            "limits": None,
            "calibrated": False,
            "joggable": False,
            "reason": "missing joint calibration",
        }
        if isinstance(joint_config, dict) and joint_config.get("servo_id") is not None:
            record["servo_id"] = int(joint_config["servo_id"])
        records["gripper"] = record
    return records


def default_steps(args):
    steps = {}
    for joint in DISPLAY_JOINTS:
        if joint == "wrist_roll":
            steps[joint] = int(args.wrist_roll_step_ticks)
        else:
            steps[joint] = int(args.step_ticks)
    return steps


def allowed_joints(allow_gripper):
    result = ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]
    if allow_gripper:
        result.append("gripper")
    return result


def parse_operator_command(line, active_joint, current_step, allowed, max_step_ticks):
    command = (line or "").strip()
    if not command:
        return {"action": "noop"}
    lower = command.lower()

    if lower in ("h", "help", "?"):
        return {"action": "help"}
    if lower in ("q", "quit", "exit"):
        return {"action": "quit"}
    if lower in ("r", "read", "readback"):
        return {"action": "read"}
    if lower == "s" or lower == "save":
        return {"action": "save", "notes": ""}
    if lower.startswith("save "):
        return {"action": "save", "notes": command[5:].strip()}

    if lower in KEY_BINDINGS:
        selected = KEY_BINDINGS[lower]
        _require_allowed_joint(selected, allowed)
        return {"action": "select", "joint": selected}

    parts = command.split()
    if len(parts) == 2 and parts[0].lower() in ("select", "joint"):
        selected = parts[1]
        _require_allowed_joint(selected, allowed)
        return {"action": "select", "joint": selected}

    if lower in ("left", "a", "-"):
        return {"action": "jog", "joint": active_joint, "delta": -int(current_step)}
    if lower in ("right", "d", "+"):
        return {"action": "jog", "joint": active_joint, "delta": int(current_step)}

    if lower == "[":
        return {
            "action": "set_step",
            "step": clamp_step_ticks(max(1, int(current_step) // 2), max_step_ticks),
        }
    if lower == "]":
        return {
            "action": "set_step",
            "step": clamp_step_ticks(int(current_step) * 2, max_step_ticks),
        }

    if len(parts) == 2 and parts[0].lower() == "step":
        return {
            "action": "set_step",
            "step": clamp_step_ticks(parts[1], max_step_ticks),
        }

    if command[0] in ("+", "-"):
        delta = _parse_delta(command)
        if abs(delta) > int(current_step):
            raise CommandRejected(
                "Requested delta {} exceeds current step size {}.".format(delta, current_step)
            )
        return {"action": "jog", "joint": active_joint, "delta": delta}

    if parts and parts[0].lower() == "jog":
        if len(parts) == 2:
            delta = _parse_delta(parts[1])
            joint = active_joint
        elif len(parts) == 3:
            joint = parts[1]
            _require_allowed_joint(joint, allowed)
            delta = _parse_delta(parts[2])
        else:
            raise CommandRejected("Use 'jog DELTA' or 'jog JOINT DELTA'.")
        if abs(delta) > int(current_step):
            raise CommandRejected(
                "Requested delta {} exceeds current step size {}.".format(delta, current_step)
            )
        return {"action": "jog", "joint": joint, "delta": delta}

    raise CommandRejected("Unknown command: {}".format(command))


def _parse_delta(raw_value):
    try:
        delta = int(raw_value)
    except (TypeError, ValueError):
        raise CommandRejected("Jog delta must be an integer.")
    if delta == 0:
        raise CommandRejected("Jog delta must be non-zero.")
    return delta


def _require_allowed_joint(joint, allowed):
    if joint not in allowed:
        if joint == "gripper":
            raise CommandRejected("Gripper jogging requires --allow-gripper.")
        raise CommandRejected("Unknown or unavailable joint: {}".format(joint))


def validate_jog_request(records, current_ticks, active_joint, delta, max_delta):
    if abs(int(delta)) > int(max_delta):
        return {
            "ok": False,
            "reason": "Requested delta {} exceeds step size {}.".format(delta, max_delta),
            "joint": active_joint,
            "target_position": None,
            "current_position": current_ticks.get(active_joint),
            "delta": abs(int(delta)),
        }

    record = records.get(active_joint)
    if not isinstance(record, dict):
        return _rejected(active_joint, current_ticks, delta, "Unknown joint.")
    if not record.get("calibrated"):
        return _rejected(active_joint, current_ticks, delta, "Joint is missing calibration.")
    if not record.get("joggable"):
        return _rejected(active_joint, current_ticks, delta, record.get("reason") or "Joint is not joggable.")

    current_position = current_ticks.get(active_joint)
    if current_position is None:
        return _rejected(active_joint, current_ticks, delta, "Current servo position is unreadable.")

    target_position = int(current_position) + int(delta)
    configured_joints = {
        record["urdf_joint"]: {"servo_id": record["servo_id"]},
    }
    limits_by_joint = {
        record["urdf_joint"]: dict(record["limits"]),
    }
    validation = safety.validate_single_joint_move(
        joint=record["urdf_joint"],
        configured_joints=configured_joints,
        current_position=current_position,
        target_position=target_position,
        limits_by_joint=limits_by_joint,
        commanded_joints=[record["urdf_joint"]],
        max_delta=max_delta,
    )
    validation["user_joint"] = active_joint
    validation["urdf_joint"] = record["urdf_joint"]
    return validation


def _rejected(active_joint, current_ticks, delta, reason):
    current_position = current_ticks.get(active_joint)
    target_position = None
    if current_position is not None:
        target_position = int(current_position) + int(delta)
    return {
        "ok": False,
        "reason": reason,
        "joint": active_joint,
        "user_joint": active_joint,
        "current_position": current_position,
        "target_position": target_position,
        "delta": abs(int(delta)),
    }


def read_current_ticks(bus, records):
    ticks = {}
    failures = []
    for user_joint in DISPLAY_JOINTS:
        record = records.get(user_joint)
        if not isinstance(record, dict) or record.get("servo_id") is None:
            continue
        try:
            position = bus.read_position(record["servo_id"])
        except Exception as exc:
            failures.append("{} servo {} read failed: {}".format(user_joint, record["servo_id"], exc))
            continue
        if position is None:
            failures.append("{} servo {} returned no Present_Position".format(user_joint, record["servo_id"]))
            continue
        ticks[user_joint] = int(position)
    if failures:
        raise TeleopError("Current servo readback failed: " + "; ".join(failures))
    return ticks


def compute_joint_angles_rad(current_ticks, calibration):
    angles = {}
    for user_joint in calibration.get("joint_order", []):
        if user_joint in current_ticks:
            angles[user_joint] = float(tick_to_angle_rad(user_joint, current_ticks[user_joint], calibration))
    return angles


def joint_angles_for_fk(joint_angles_rad, calibration):
    converted = {}
    for user_joint, angle in joint_angles_rad.items():
        entry = calibration["joints"].get(user_joint)
        if entry:
            converted[entry["urdf_joint"]] = float(angle)
    return converted


def compute_transform_payload(args, calibration, current_ticks):
    joint_angles_rad = compute_joint_angles_rad(current_ticks, calibration)
    notes = []
    try:
        model = load_urdf_model(args.urdf)
        tool_frames = load_tool_frames(args.tool_frames)
        tool_frame = get_tool_frame(tool_frames, args.tcp_frame)
        transform = compute_tcp_transform(
            model,
            joint_angles_for_fk(joint_angles_rad, calibration),
            tool_frame=tool_frame,
        )
        return joint_angles_rad, _matrix_to_list(transform), describe_tool_frame(tool_frame, args.tcp_frame), notes
    except Exception as exc:
        notes.append("FK unavailable: {}".format(exc))
        return joint_angles_rad, None, None, notes



def build_pose_snapshot(pose_id, session_id, timestamp_iso, timestamp_unix, joint_ticks,
                        joint_angles_rad, tcp_frame, notes, transform=None,
                        tool_frame=None, mode="dry_run", source="keyboard_teleop_line"):
    payload = {
        "pose_id": int(pose_id),
        "session_id": str(session_id),
        "timestamp_iso": timestamp_iso,
        "timestamp": timestamp_iso,
        "timestamp_unix": float(timestamp_unix),
        "joint_ticks": _ordered_mapping(joint_ticks),
        "joint_angles_rad": _ordered_mapping(joint_angles_rad),
        "tcp_frame": tcp_frame,
        "T_base_gripper": transform,
        "notes": list(notes or []),
        "source": source,
        "mode": mode,
    }
    if tool_frame is not None:
        payload["tool_frame"] = tool_frame
    return payload


def save_pose_snapshot(args, session, calibration, current_ticks, extra_notes):
    if session is None:
        session = TeleopSession(getattr(args, "mode", "line"))
    pose_id = session.next_pose_id()
    path, timestamp_iso, timestamp_unix = next_pose_output_path(args.pose_output_dir, pose_id)
    joint_angles_rad, transform, tool_frame, fk_notes = compute_transform_payload(
        args, calibration, current_ticks
    )
    mode_name = getattr(session, "mode", getattr(args, "mode", "line"))
    notes = ["%s keyboard teleop snapshot" % mode_name]
    if args.dry_run:
        notes.append("dry-run snapshot from in-session tick state")
    else:
        notes.append("execute-mode snapshot from latest readback/session state")
    if extra_notes:
        notes.append(str(extra_notes))
    notes.extend(fk_notes)
    payload = build_pose_snapshot(
        pose_id=pose_id,
        session_id=session.session_id,
        timestamp_iso=timestamp_iso,
        timestamp_unix=timestamp_unix,
        joint_ticks=current_ticks,
        joint_angles_rad=joint_angles_rad,
        tcp_frame=args.tcp_frame,
        notes=notes,
        transform=transform,
        tool_frame=tool_frame,
        mode="dry_run" if args.dry_run else "execute",
        source="keyboard_teleop_%s" % mode_name,
    )
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return path, payload


def next_pose_output_path(output_dir, pose_id):
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    for _attempt in range(100):
        now = datetime.datetime.utcnow()
        timestamp_iso = _format_timestamp_iso(now)
        stamp = now.strftime("%Y%m%d_%H%M%S_") + "%03d" % (now.microsecond // 1000)
        filename = "pose_%04d_%s.json" % (int(pose_id), stamp)
        path = os.path.join(output_dir, filename)
        if not os.path.exists(path):
            return path, timestamp_iso, time.time()
        time.sleep(0.001)
    raise TeleopError("Could not allocate a unique pose snapshot filename in %s." % output_dir)


def _format_timestamp_iso(value):
    return value.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (value.microsecond // 1000)

def append_log(path, event_type, **fields):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    event = {
        "timestamp_utc": datetime.datetime.utcnow().isoformat() + "Z",
        "event": event_type,
    }
    event.update(fields)
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(event, sort_keys=True) + "\n")


def build_bus_for_teleop(config, config_path):
    servo_config = config.get("servo_bus") or {}
    backend_name = servo_config.get("backend")
    if backend_name == "mock" and servo_config.get("feetech"):
        backend_name = "feetech"
    try:
        return build_servo_bus(
            config=config,
            config_path=config_path,
            dry_run=False,
            backend_name=backend_name or "feetech",
        )
    except (BackendUnavailable, ServoBusError, OSError, ValueError) as exc:
        raise TeleopError("Could not open servo backend for readback/teleop: {}".format(exc))


def enable_torque(bus, records):
    for user_joint in DISPLAY_JOINTS:
        record = records.get(user_joint)
        if not isinstance(record, dict) or not record.get("joggable"):
            continue
        bus.torque_enable(record["servo_id"], True)


def disable_torque(bus, records):
    errors = []
    for user_joint in DISPLAY_JOINTS:
        record = records.get(user_joint)
        if not isinstance(record, dict) or not record.get("joggable"):
            continue
        try:
            bus.torque_enable(record["servo_id"], False)
        except Exception as exc:
            errors.append("{}: {}".format(user_joint, exc))
    return errors



def parse_hotkey(key, allow_gripper):
    if key == "\x03":
        return {"action": "quit"}
    if key == "q":
        return {"action": "quit"}
    if key == "h":
        return {"action": "help"}
    if key == " ":
        return {"action": "read"}
    if key == "s":
        return {"action": "save", "notes": ""}
    if key == "[":
        return {"action": "decrease_steps"}
    if key == "]":
        return {"action": "increase_steps"}
    if key in HOTKEY_JOG_BINDINGS:
        joint, direction = HOTKEY_JOG_BINDINGS[key]
        if joint == "gripper" and not allow_gripper:
            return {
                "action": "ignored",
                "reason": "gripper hotkeys require --allow-gripper",
                "key": key,
            }
        return {"action": "jog", "joint": joint, "direction": int(direction), "key": key}
    return {"action": "ignored", "reason": "unmapped key", "key": key}


def is_rate_limited(now, last_command_time, min_interval_sec):
    if last_command_time is None:
        return False
    return (float(now) - float(last_command_time)) < float(min_interval_sec)


def action_is_rate_limited(action):
    return action in ("jog", "read", "save", "decrease_steps", "increase_steps")


def adjust_all_steps(steps, increase, max_step_ticks):
    adjusted = {}
    for joint, step in steps.items():
        if increase:
            adjusted[joint] = clamp_step_ticks(int(step) * 2, max_step_ticks)
        else:
            adjusted[joint] = clamp_step_ticks(max(1, int(step) // 2), max_step_ticks)
    steps.update(adjusted)
    return adjusted


def print_hotkey_help(allow_gripper=False):
    print("")
    print("Hotkey controls:")
    print("  a/d = base_yaw -/+")
    print("  z/x = shoulder_pitch -/+")
    print("  c/v = elbow_pitch -/+")
    print("  b/n = wrist_pitch -/+")
    print("  j/k = wrist_roll -/+")
    if allow_gripper:
        print("  u/i = gripper -/+")
    else:
        print("  u/i = gripper -/+ only with --allow-gripper")
    print("  space = readback")
    print("  s = save pose and continue")
    print("  [ / ] = decrease/increase all step sizes")
    print("  h = help")
    print("  q = quit")
    print("")


def print_step_summary(steps):
    print("Current step sizes:")
    for joint in DISPLAY_JOINTS:
        if joint in steps:
            print("  {:<16} {} ticks".format(joint, steps[joint]))
    print("")

def print_help():
    print("")
    print("Commands:")
    print("  1..5              select base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll")
    print("  6                 select gripper only when --allow-gripper is set")
    print("  left/a/-          jog active joint by negative current step")
    print("  right/d/+         jog active joint by positive current step")
    print("  +N or -N          jog active joint by explicit delta not exceeding current step")
    print("  step N            set active joint step size, clamped to --max-step-ticks")
    print("  [ or ]            halve or double active joint step size")
    print("  read              read back current servo positions")
    print("  save [notes]      save a hand-eye pose snapshot")
    print("  h                 print this help")
    print("  q                 quit safely")
    print("")


def print_joint_table(records, ticks, calibration, active_joint, steps):
    angles = compute_joint_angles_rad(ticks, calibration)
    print("")
    print("Joint state:")
    print("{:<18} {:>8} {:>14} {:>10}  {}".format("joint", "tick", "angle_rad", "step", "status"))
    for joint in DISPLAY_JOINTS:
        record = records.get(joint)
        if not isinstance(record, dict):
            continue
        marker = "*" if joint == active_joint else " "
        tick = ticks.get(joint)
        angle = angles.get(joint)
        status = "ok" if record.get("joggable") else "blocked: {}".format(record.get("reason"))
        tick_text = "-" if tick is None else str(tick)
        angle_text = "-" if angle is None else "{:.6f}".format(angle)
        step_text = str(steps.get(joint, "-"))
        print("{}{:<17} {:>8} {:>14} {:>10}  {}".format(
            marker, joint, tick_text, angle_text, step_text, status
        ))
    print("")



def print_jog_result(validation, compact=False):
    if compact:
        status = "accepted" if validation.get("ok") else "rejected"
        message = "Jog {joint} delta={delta} target={target} {status}".format(
            joint=validation.get("user_joint") or validation.get("joint"),
            delta=validation.get("delta"),
            target=validation.get("target_position"),
            status=status,
        )
        if not validation.get("ok"):
            message += " reason={}".format(validation.get("reason"))
        print(message)
        return
    print("Jog proposal:")
    print("  joint: {}".format(validation.get("user_joint") or validation.get("joint")))
    print("  current tick: {}".format(validation.get("current_position")))
    print("  target tick: {}".format(validation.get("target_position")))
    print("  delta: {}".format(validation.get("delta")))
    print("  safety status: {}".format("accepted" if validation.get("ok") else "rejected"))
    if not validation.get("ok"):
        print("  reason: {}".format(validation.get("reason")))


def execute_jog(args, bus, records, current_ticks, active_joint, delta, step, compact=False):
    validation = validate_jog_request(records, current_ticks, active_joint, delta, step)
    print_jog_result(validation, compact=compact)
    append_log(
        args.log_path,
        "jog",
        accepted=bool(validation.get("ok")),
        dry_run=bool(args.dry_run),
        command_joint=active_joint,
        requested_delta=int(delta),
        validation=validation,
    )
    if not validation.get("ok"):
        return False

    target_position = int(validation["target_position"])

    if args.dry_run:
        current_ticks[active_joint] = target_position
        if compact:
            print("  readback tick: dry-run")
        else:
            print("  dry-run: no servo write performed")
        return True

    record = records[active_joint]
    try:
        bus.write_goal_position(record["servo_id"], target_position)
    except Exception as exc:
        print("  write failed: {}".format(exc))
        append_log(
            args.log_path,
            "jog_write_failed",
            dry_run=False,
            command_joint=active_joint,
            target_position=target_position,
            error=str(exc),
        )
        return False
    try:
        readback = bus.read_position(record["servo_id"])
        if readback is not None:
            current_ticks[active_joint] = int(readback)
            print("  readback tick: {}".format(readback))
        else:
            print("  readback tick: unavailable")
    except Exception as exc:
        print("  readback warning: {}".format(exc))
    return True

def interactive_loop(args, session, bus, records, calibration, current_ticks):
    active_joint = DEFAULT_ACTIVE_JOINT
    steps = default_steps(args)
    allowed = allowed_joints(args.allow_gripper)

    print_help()
    print_joint_table(records, current_ticks, calibration, active_joint, steps)

    while True:
        try:
            line = input("teleop:{} step {}> ".format(active_joint, steps[active_joint]))
        except EOFError:
            print("")
            return "eof"
        except KeyboardInterrupt:
            print("")
            return "interrupt"

        try:
            command = parse_operator_command(
                line=line,
                active_joint=active_joint,
                current_step=steps[active_joint],
                allowed=allowed,
                max_step_ticks=args.max_step_ticks,
            )
        except CommandRejected as exc:
            print("Rejected command: {}".format(exc))
            append_log(args.log_path, "command_rejected", command=line, reason=str(exc))
            continue

        action = command.get("action")
        if action == "noop":
            continue
        if action == "help":
            print_help()
            continue
        if action == "quit":
            append_log(args.log_path, "quit", mode=session.mode, reason="operator_key")
            return "quit"
        if action == "select":
            active_joint = command["joint"]
            print("Active joint: {}".format(active_joint))
            print_joint_table(records, current_ticks, calibration, active_joint, steps)
            continue
        if action == "set_step":
            steps[active_joint] = int(command["step"])
            print("Step for {}: {} ticks".format(active_joint, steps[active_joint]))
            continue
        if action == "read":
            current_ticks.clear()
            current_ticks.update(read_current_ticks(bus, records))
            print_joint_table(records, current_ticks, calibration, active_joint, steps)
            continue
        if action == "save":
            path, payload = save_pose_snapshot(args, session, calibration, current_ticks, command.get("notes"))
            print("Saved pose snapshot: {}".format(path))
            append_log(args.log_path, "save_pose", pose_path=path, pose_id=payload.get("pose_id"), session_id=session.session_id, source=payload.get("source"), dry_run=bool(args.dry_run))
            continue
        if action == "jog":
            jog_joint = command["joint"]
            execute_jog(
                args=args,
                bus=bus,
                records=records,
                current_ticks=current_ticks,
                active_joint=jog_joint,
                delta=command["delta"],
                step=steps[jog_joint],
            )
            print_joint_table(records, current_ticks, calibration, active_joint, steps)
            continue



def hotkey_loop(args, session, bus, records, calibration, current_ticks):
    steps = default_steps(args)
    last_command_time = None

    print_hotkey_help(args.allow_gripper)
    print_step_summary(steps)
    print_joint_table(records, current_ticks, calibration, DEFAULT_ACTIVE_JOINT, steps)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        try:
            while True:
                ready, _unused_write, _unused_error = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue

                key = sys.stdin.read(1)
                command = parse_hotkey(key, args.allow_gripper)
                action = command.get("action")

                if action == "ignored":
                    append_log(args.log_path, "ignored_key", key=repr(key), reason=command.get("reason"), mode=session.mode)
                    if command.get("reason") != "unmapped key":
                        print("Ignored key: {}".format(command.get("reason")))
                    continue

                now = time.time()
                if action_is_rate_limited(action) and is_rate_limited(now, last_command_time, args.min_command_interval_sec):
                    append_log(
                        args.log_path,
                        "ignored_rate_limited_key",
                        key=repr(key),
                        action=action,
                        min_command_interval_sec=args.min_command_interval_sec,
                        mode=session.mode,
                    )
                    print("Ignored key: rate limited")
                    continue
                if action_is_rate_limited(action):
                    last_command_time = now

                if action == "quit":
                    append_log(args.log_path, "quit", mode=session.mode, reason="operator_key")
                    return "quit"
                if action == "help":
                    print_hotkey_help(args.allow_gripper)
                    print_step_summary(steps)
                    continue
                if action == "read":
                    current_ticks.clear()
                    current_ticks.update(read_current_ticks(bus, records))
                    print("Readback complete.")
                    print_joint_table(records, current_ticks, calibration, DEFAULT_ACTIVE_JOINT, steps)
                    continue
                if action == "save":
                    path, payload = save_pose_snapshot(args, session, calibration, current_ticks, command.get("notes"))
                    print("Saved pose {:04d}: {}".format(int(payload.get("pose_id")), path))
                    append_log(args.log_path, "save_pose", pose_path=path, pose_id=payload.get("pose_id"), session_id=session.session_id, source=payload.get("source"), dry_run=bool(args.dry_run))
                    continue
                if action in ("decrease_steps", "increase_steps"):
                    adjust_all_steps(steps, increase=(action == "increase_steps"), max_step_ticks=args.max_step_ticks)
                    print_step_summary(steps)
                    continue
                if action == "jog":
                    joint = command["joint"]
                    step = int(steps[joint])
                    delta = int(command["direction"]) * step
                    execute_jog(
                        args=args,
                        bus=bus,
                        records=records,
                        current_ticks=current_ticks,
                        active_joint=joint,
                        delta=delta,
                        step=step,
                        compact=True,
                    )
                    continue
        except KeyboardInterrupt:
            append_log(args.log_path, "quit", mode=session.mode, reason="keyboard_interrupt")
            return "interrupt"
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def _ordered_mapping(mapping):
    ordered = {}
    for key in DISPLAY_JOINTS:
        if key in mapping:
            ordered[key] = mapping[key]
    for key in sorted(mapping):
        if key not in ordered:
            ordered[key] = mapping[key]
    return ordered


def _matrix_to_list(matrix):
    return [[float(value) for value in row] for row in matrix]


def main(argv=None, bus_factory=None):
    args = validate_args(build_parser().parse_args(argv))
    mode_text = "EXECUTE" if args.execute else "DRY-RUN"
    print("Keyboard teleop mode: {} ({})".format(mode_text, args.mode))
    if args.dry_run:
        print("WARNING: dry-run mode is active. Jog commands validate and update the in-session state only.")
    else:
        print("WARNING: execute mode is active. Each accepted jog writes one small servo target.")

    config = load_robot_config(args.config)
    calibration = load_joint_calibration(args.joint_calibration)
    safety_limits = load_joint_safety_limits(args.joint_safety_limits)
    records = build_joint_records(config, calibration, safety_limits, args.allow_gripper)
    if DEFAULT_ACTIVE_JOINT not in records or not records[DEFAULT_ACTIVE_JOINT].get("joggable"):
        raise TeleopError("Default active joint base_yaw is not joggable.")

    session = TeleopSession(args.mode)
    bus = None
    torque_enabled = False
    exit_reason = None
    try:
        if bus_factory is not None:
            bus = bus_factory(config=config, config_path=args.config)
        else:
            bus = build_bus_for_teleop(config, args.config)

        current_ticks = read_current_ticks(bus, records)

        if args.execute and not args.no_torque_on_start:
            enable_torque(bus, records)
            torque_enabled = True
            print("Torque enabled for joggable joints.")

        if args.mode == "hotkey":
            exit_reason = hotkey_loop(args, session, bus, records, calibration, current_ticks)
        else:
            exit_reason = interactive_loop(args, session, bus, records, calibration, current_ticks)
        return 0
    finally:
        if bus is not None:
            should_disable = bool(args.torque_off_on_exit)
            if torque_enabled and exit_reason in (None, "quit", "interrupt"):
                should_disable = True
            if should_disable and not args.dry_run:
                errors = disable_torque(bus, records)
                if errors:
                    print("Torque-off warnings: {}".format("; ".join(errors)))
                else:
                    print("Torque disabled for joggable joints.")
            bus.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except TeleopError as exc:
        raise SystemExit(str(exc))
