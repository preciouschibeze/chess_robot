#!/usr/bin/env python3
"""Export the current readback FK pose without commanding robot motion.

This tool intentionally performs only passive servo operations: ping and
Present_Position readback. It never enables torque, writes goal positions,
invokes gripper code, or calls safe transfer/motion modules.
"""

from __future__ import absolute_import

import argparse
import datetime
import json
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.servo_bus import BackendUnavailable
from chess_robot.robot.servo_bus import ServoBusError
from chess_robot.robot.servo_bus import build_servo_bus
from chess_robot.robot.servo_bus import configured_joint_servo_ids
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
DEFAULT_URDF = os.path.join(
    REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf"
)
DEFAULT_TOOL_FRAMES = os.path.join(
    REPO_ROOT, "data", "calibration", "gripper", "tool_frames.yaml"
)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Passively read current servo encoders and export calibrated TCP FK as JSON."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="Robot config YAML.")
    parser.add_argument(
        "--joint-calibration",
        default=DEFAULT_JOINT_CALIBRATION,
        help="Joint tick-to-URDF-angle calibration YAML.",
    )
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="Calibrated robot URDF.")
    parser.add_argument(
        "--tool-frames", default=DEFAULT_TOOL_FRAMES, help="Tool frame calibration YAML."
    )
    parser.add_argument(
        "--tcp-frame", default="gripper_frame", help="Tool frame to export. Default: gripper_frame"
    )
    parser.add_argument(
        "--json-stdout", action="store_true", help="Write the JSON payload to stdout."
    )
    parser.add_argument("--output", help="Optional path to write the JSON payload.")
    return parser


def main():
    args = build_parser().parse_args()

    config = load_robot_config(args.config)
    joint_name_by_servo_id = _joint_name_by_servo_id(config)
    servo_ids = configured_joint_servo_ids(config)
    if not servo_ids:
        raise SystemExit("No configured joint servo IDs found in %s" % args.config)

    servo_ids = _calibrated_servo_ids(config, load_joint_calibration(args.joint_calibration), servo_ids, joint_name_by_servo_id)
    joint_ticks = _read_current_joint_ticks(config, args.config, servo_ids, joint_name_by_servo_id)
    calibration = load_joint_calibration(args.joint_calibration)
    joint_angles_rad = convert_pose_ticks_to_urdf_radians(joint_ticks, calibration)

    model = load_urdf_model(args.urdf)
    tool_frames = load_tool_frames(args.tool_frames)
    tool_frame = get_tool_frame(tool_frames, args.tcp_frame)
    T_base_gripper = compute_tcp_transform(model, joint_angles_rad, tool_frame=tool_frame)

    notes = [
        "Passive readback only: Present_Position reads were used.",
        "No torque enable, goal-position write, gripper command, safe_transfer, or robot motion is performed.",
    ]
    notes.extend(calibration.get("warnings", []))

    payload = {
        "timestamp": datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z",
        "tcp_frame": args.tcp_frame,
        "joint_ticks": _ordered_joint_ticks(joint_ticks, calibration),
        "joint_angles_rad": _ordered_joint_angles(joint_angles_rad, model, args.tcp_frame),
        "T_base_gripper": _matrix_to_list(T_base_gripper),
        "source": "readback",
        "notes": notes,
        "tool_frame": describe_tool_frame(tool_frame, fallback_name=args.tcp_frame),
    }

    serialized = json.dumps(payload, indent=2, sort_keys=True)
    if args.output:
        output_dir = os.path.dirname(os.path.abspath(args.output))
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(serialized + "\n")
    if args.json_stdout or not args.output:
        print(serialized)


def _joint_name_by_servo_id(config):
    result = {}
    joints = config.get("joints") or {}
    if isinstance(joints, dict):
        for joint_name, joint_config in joints.items():
            if isinstance(joint_config, dict) and joint_config.get("servo_id") is not None:
                result[int(joint_config["servo_id"])] = str(joint_name)
    return result


def _calibrated_servo_ids(config, calibration, servo_ids, joint_name_by_servo_id):
    calibrated_names = set()
    for user_joint, entry in calibration.get("joints", {}).items():
        calibrated_names.add(str(user_joint))
        calibrated_names.add(str(entry.get("urdf_joint")))
    selected = [int(servo_id) for servo_id in servo_ids if joint_name_by_servo_id.get(int(servo_id)) in calibrated_names]
    if not selected:
        raise SystemExit("No configured servos match calibrated URDF joints.")
    return selected


def _read_current_joint_ticks(config, config_path, servo_ids, joint_name_by_servo_id):
    try:
        bus = build_servo_bus(
            config=config,
            config_path=config_path,
            dry_run=False,
            backend_name="feetech",
        )
    except (BackendUnavailable, ServoBusError, OSError, ValueError) as exc:
        raise SystemExit("Could not open read-only Feetech servo backend: %s" % exc)

    joint_ticks = {}
    failures = []
    try:
        for servo_id in servo_ids:
            joint_name = joint_name_by_servo_id.get(int(servo_id), "servo_%s" % servo_id)
            try:
                position = bus.read_position(servo_id)
            except Exception as exc:
                failures.append("servo %s (%s) Present_Position read failed: %s" % (servo_id, joint_name, exc))
                continue
            if position is None:
                failures.append("servo %s (%s) returned no Present_Position" % (servo_id, joint_name))
                continue
            joint_ticks[joint_name] = int(position)
    finally:
        bus.close()

    if failures:
        raise SystemExit("Passive FK export failed: " + "; ".join(failures))
    return joint_ticks


def _ordered_joint_ticks(joint_ticks, calibration):
    ordered = {}
    for user_joint in calibration.get("joint_order", []):
        entry = calibration["joints"][user_joint]
        if user_joint in joint_ticks:
            ordered[user_joint] = int(joint_ticks[user_joint])
        elif entry["urdf_joint"] in joint_ticks:
            ordered[entry["urdf_joint"]] = int(joint_ticks[entry["urdf_joint"]])
    for joint_name in sorted(joint_ticks):
        if joint_name not in ordered:
            ordered[joint_name] = int(joint_ticks[joint_name])
    return ordered


def _ordered_joint_angles(joint_angles_rad, model, tcp_frame):
    ordered = {}
    try:
        frame_name = "gripper_frame_link" if tcp_frame == "gripper_frame" else None
        chain = model.get_arm_chain(end_link=frame_name or "gripper_frame_link")
        for joint in chain:
            if joint.name in joint_angles_rad:
                ordered[joint.name] = float(joint_angles_rad[joint.name])
    except Exception:
        pass
    for joint_name in sorted(joint_angles_rad):
        if joint_name not in ordered:
            ordered[joint_name] = float(joint_angles_rad[joint_name])
    return ordered


def _matrix_to_list(matrix):
    array = np.asarray(matrix, dtype=float)
    if array.shape != (4, 4):
        raise ValueError("Expected 4x4 transform, got shape %s" % (array.shape,))
    return [[float(value) for value in row] for row in array]


if __name__ == "__main__":
    main()
