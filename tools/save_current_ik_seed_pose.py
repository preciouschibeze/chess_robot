#!/usr/bin/env python3
from __future__ import absolute_import, print_function

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.ik_seed_poses import IKSeedPoseError
from chess_robot.robot.ik_seed_poses import load_or_default_ik_seed_poses
from chess_robot.robot.ik_seed_poses import save_ik_seed_poses
from chess_robot.robot.ik_seed_poses import upsert_square_seed_entry
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.servo_bus import build_servo_bus
from chess_robot.robot.servo_bus import load_robot_config


DEFAULT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_IK_SEED_POSES_PATH = os.path.join(ROOT, "data", "calibration", "robot", "ik_seed_poses.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
CONFIRM_TEXT = "SAVE_IK_SEED_POSE"


class SaveCurrentIKSeedPoseError(RuntimeError):
    pass


def build_parser():
    parser = argparse.ArgumentParser(
        description="Read current servo positions and save one square-specific IK seed pose. Dry-run is the default."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Robot config path.")
    parser.add_argument("--ik-seed-poses", default=DEFAULT_IK_SEED_POSES_PATH, help="IK seed poses YAML path.")
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS_PATH, help="Joint safety limits YAML path.")
    parser.add_argument("--square", required=True, help="Board square to save, for example a1.")
    parser.add_argument("--notes", default=None, help="Optional square note to store with the saved seed.")
    parser.add_argument("--live", action="store_true", help="Use the configured Feetech backend for read-only live reads.")
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None, help="Optional backend override.")
    parser.add_argument("--confirm", default=None, help="Typed confirmation required before writing a live-read seed file.")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        result = save_current_ik_seed_pose(args)
    except (IKSeedPoseError, SaveCurrentIKSeedPoseError, IOError, OSError, ValueError) as exc:
        print("ERROR: %s" % exc)
        return 1

    print("Square: %s" % result["square"])
    print("IK seed poses path: %s" % result["ik_seed_poses_path"])
    print("Live read: %s" % bool(result["live"]))
    print("Saved: %s" % bool(result["saved"]))
    print("Seed ticks: %s" % result["seed_ticks"])
    if result.get("notes") is not None:
        print("Notes: %s" % result["notes"])
    if not result["saved"]:
        print("Dry-run only. Re-run with --live --confirm %s to write the file." % CONFIRM_TEXT)
    return 0


def save_current_ik_seed_pose(args):
    square = str(args.square).lower()
    if not bool(args.live):
        seed_ticks = {}
        saved = False
    else:
        if args.confirm != CONFIRM_TEXT:
            raise SaveCurrentIKSeedPoseError("Live save requires --confirm %s." % CONFIRM_TEXT)
        seed_ticks = read_live_arm_ticks(args)
        validate_seed_ticks_against_limits(square, seed_ticks, args.joint_safety_limits)
        document = load_or_default_ik_seed_poses(args.ik_seed_poses)
        serializable_document = build_serializable_document(document)
        updated = upsert_square_seed_entry(serializable_document, square, seed_ticks, notes=args.notes)
        save_ik_seed_poses(args.ik_seed_poses, updated)
        saved = True

    return {
        "square": square,
        "ik_seed_poses_path": args.ik_seed_poses,
        "live": bool(args.live),
        "saved": saved,
        "seed_ticks": dict((joint_name, int(seed_ticks[joint_name])) for joint_name in seed_ticks),
        "notes": args.notes,
    }


def read_live_arm_ticks(args):
    config = load_robot_config(args.config)
    backend_name = args.backend or "feetech"
    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=False,
        backend_name=backend_name,
        mock_ids=None,
    )
    try:
        joints_config = config.get("joints") or {}
        ticks = {}
        for joint_name in ARM_JOINTS:
            joint_entry = joints_config.get(joint_name)
            if not isinstance(joint_entry, dict) or joint_entry.get("servo_id") is None:
                raise SaveCurrentIKSeedPoseError("Robot config is missing servo_id for %s." % joint_name)
            servo_id = int(joint_entry["servo_id"])
            tick_value = bus.read_position(servo_id)
            if tick_value is None:
                raise SaveCurrentIKSeedPoseError("Live position read failed for %s (servo %d)." % (joint_name, servo_id))
            ticks[joint_name] = int(tick_value)
        return ticks
    finally:
        bus.close()


def validate_seed_ticks_against_limits(square, seed_ticks, path):
    joint_safety_limits = load_joint_safety_limits(path)
    joints = joint_safety_limits.get("joints") or {}
    for joint_name in ARM_JOINTS:
        if joint_name not in seed_ticks:
            raise SaveCurrentIKSeedPoseError("Live IK seed read is missing %s." % joint_name)
        limits = joints.get(joint_name)
        if not isinstance(limits, dict):
            raise SaveCurrentIKSeedPoseError("Joint safety limits are missing %s." % joint_name)
        minimum = limits.get("min_tick")
        maximum = limits.get("max_tick")
        if minimum is None or maximum is None:
            raise SaveCurrentIKSeedPoseError("Joint safety limits for %s are incomplete." % joint_name)
        tick_value = int(seed_ticks[joint_name])
        if tick_value < int(minimum) or tick_value > int(maximum):
            raise SaveCurrentIKSeedPoseError(
                "Live IK seed pose for %s joint %s tick %d is outside joint safety limits %d..%d."
                % (square, joint_name, tick_value, int(minimum), int(maximum))
            )


def build_serializable_document(document):
    return {
        "ik_seed_poses": {
            "version": int(document["version"]),
            "notes": list(document.get("notes") or []),
            "default": dict(document.get("default") or {}),
            "squares": dict((square_name, dict(entry)) for square_name, entry in (document.get("squares") or {}).items()),
        }
    }


if __name__ == "__main__":
    raise SystemExit(main())
