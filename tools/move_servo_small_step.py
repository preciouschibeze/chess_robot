#!/usr/bin/env python3
"""Safety-validated single-joint micro-motion tool."""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.arm_controller import ArmController, ArmControllerError
from chess_robot.robot import safety


REAL_MOVEMENT_DISABLED_MESSAGE = (
    "REAL MOVEMENT DISABLED: previous gripper command overshot present position. Audit required."
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plan or execute a single-joint micro-motion. Dry-run is the default."
    )
    parser.add_argument(
        "--config",
        default=os.path.join(ROOT, "configs", "robot.yaml"),
        help="Path to robot YAML config. Default: configs/robot.yaml",
    )
    parser.add_argument(
        "--joint",
        required=True,
        help="Joint name to move, e.g. gripper.",
    )
    movement_group = parser.add_mutually_exclusive_group(required=True)
    movement_group.add_argument(
        "--delta",
        type=int,
        help="Relative target offset in raw servo ticks.",
    )
    movement_group.add_argument(
        "--target",
        type=int,
        help="Absolute target position in raw servo ticks.",
    )
    parser.add_argument(
        "--max-delta",
        type=int,
        default=None,
        help="Maximum allowed absolute delta from current position. Default: conservative 10 ticks.",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="Disabled pending audit after gripper present-position overshoot.",
    )
    return parser


def _format_limits(limits) -> str:
    if not isinstance(limits, dict):
        return "unavailable"
    return "{}..{}".format(limits.get("min"), limits.get("max"))


def _print_plan(plan) -> None:
    validation = plan.get("validation") or {}
    print("Mode: {}".format("REAL" if not plan.get("dry_run", True) else "DRY-RUN"))
    print("Joint: {}".format(plan.get("joint")))
    print("Servo ID: {}".format(plan.get("servo_id")))
    print("Current position: {}".format(plan.get("current_position")))
    print("Target position: {}".format(plan.get("target_position")))
    print("Delta: {}".format(plan.get("delta")))
    print("Max delta: {}".format(plan.get("max_delta")))
    print("Limits: {}".format(_format_limits(plan.get("limits"))))
    print("Validation: {}".format("ok" if validation.get("ok") else "failed"))
    print("Reason: {}".format(validation.get("reason")))


def main() -> None:
    args = build_parser().parse_args()
    if args.real:
        # Real movement remains disabled because a previous gripper command exposed
        # unsafe goal-vs-present verification behaviour. The gripper has since been
        # recalibrated, but real motion must only be re-enabled through a dedicated
        # monitored gripper-only test path, not this general tool.
        print(REAL_MOVEMENT_DISABLED_MESSAGE)
        raise SystemExit(1)

    controller = ArmController(config_path=args.config)
    log_path = None
    try:
        plan = controller.plan_single_joint_move(
            joint=args.joint,
            delta=args.delta,
            target=args.target,
            max_delta=args.max_delta,
            real=args.real,
        )
        _print_plan(plan)
        if controller.bus is not None:
            log_path = controller.bus.logger.path

        confirmation_text = None
        if args.real and plan.get("validation", {}).get("ok"):
            safety.require_real_movement_joint_allowed(plan.get("joint"))
            expected = plan.get("expected_confirmation")
            print("Typed confirmation required: {}".format(expected))
            confirmation_text = input("> ")

        result = controller.execute_single_joint_move(
            plan=plan,
            confirmation_text=confirmation_text,
        )
    except (ArmControllerError, ValueError) as exc:
        print("ERROR: {}".format(exc))
        if log_path is None and controller.bus is not None:
            log_path = controller.bus.logger.path
        if log_path is not None:
            print("Log: {}".format(log_path))
        raise SystemExit(1)
    finally:
        controller.close()

    print(result.get("message"))
    print("Log: {}".format(log_path if log_path is not None else "unavailable"))


if __name__ == "__main__":
    main()
