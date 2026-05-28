#!/usr/bin/env python3
"""Run the configured demo movebook as a physical open-loop sequence.

This intentionally skips camera input, FEN state, UCI prompts, legality checks,
Stockfish, and board updates. The operator advances the sequence after making
each expected human move on the physical board.
"""

from __future__ import absolute_import
from __future__ import print_function

import argparse
import json
import os
import shlex
import sys
from datetime import datetime
from types import SimpleNamespace

try:
    import yaml
except ImportError:  # pragma: no cover - required on the Nano runtime image
    yaml = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map  # noqa: E402


DEFAULT_MOVEBOOK = "configs/demo_movebook.yaml"
DEFAULT_SQUARE_TARGETS = "data/calibration/robot/square_targets.yaml"
DEFAULT_JOINT_LIMITS = "data/calibration/robot/joint_limits.yaml"
DEFAULT_SERVO_MAP = "data/calibration/robot/servo_map.yaml"
DEFAULT_GRIPPER_PROFILE = "data/calibration/gripper/gripper_profile.yaml"
DEFAULT_ROBOT_CONFIG = "configs/robot.yaml"
DEFAULT_HOME_POSE = "data/calibration/robot/home_pose.yaml"
DEFAULT_OPEN_LOOP_LOG = "data/logs/movebook_physical_sequence.log"
DEFAULT_OUTPUT_JSON = "data/debug/movebook_physical_sequence_last.json"


class MovebookPhysicalSequenceError(RuntimeError):
    """Raised when the physical sequence cannot continue safely."""


def utc_now_text():
    return datetime.utcnow().isoformat() + "Z"


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def display_path(path):
    absolute_path = os.path.abspath(path)
    root_prefix = ROOT + os.sep
    if absolute_path.startswith(root_prefix):
        return absolute_path[len(root_prefix):]
    return path


def append_jsonl(path, payload):
    ensure_parent_dir(path)
    with open(path, "a") as handle:
        handle.write(json.dumps(payload, sort_keys=True))
        handle.write("\n")


def write_json(path, payload):
    ensure_parent_dir(path)
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def load_movebook_entries(path):
    if yaml is None:
        raise MovebookPhysicalSequenceError("PyYAML is required to load movebook files.")
    if not path:
        raise MovebookPhysicalSequenceError("Movebook path is required.")
    if not os.path.exists(path):
        raise MovebookPhysicalSequenceError("Movebook file does not exist: {}".format(path))
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise MovebookPhysicalSequenceError("Movebook YAML must contain a mapping.")
    mapping = data.get("demo_movebook")
    if not isinstance(mapping, dict) or not mapping:
        raise MovebookPhysicalSequenceError("Movebook YAML must contain non-empty demo_movebook mapping.")
    entries = []
    for human_move, robot_move in mapping.items():
        human = str(human_move).strip().lower()
        robot = str(robot_move).strip().lower()
        if not is_uci_like(human):
            raise MovebookPhysicalSequenceError("Invalid human move key in movebook: {!r}".format(human_move))
        if not is_uci_like(robot):
            raise MovebookPhysicalSequenceError("Invalid robot move value for {}: {!r}".format(human, robot_move))
        entries.append({"expected_human_move": human, "robot_move": robot})
    return entries


def is_uci_like(value):
    if not isinstance(value, str):
        return False
    value = value.strip().lower()
    if len(value) not in (4, 5):
        return False
    if value[0] not in "abcdefgh" or value[2] not in "abcdefgh":
        return False
    if value[1] not in "12345678" or value[3] not in "12345678":
        return False
    if len(value) == 5 and value[4] not in "qrbn":
        return False
    return True


def select_entries(entries, start_index, max_moves):
    if int(start_index) < 1:
        raise MovebookPhysicalSequenceError("--start-index must be >= 1.")
    if max_moves is not None and int(max_moves) < 1:
        raise MovebookPhysicalSequenceError("--max-moves must be >= 1 when provided.")
    selected = entries[int(start_index) - 1:]
    if max_moves is not None:
        selected = selected[:int(max_moves)]
    return selected


def required_config_paths(args):
    return [
        args.movebook,
        args.square_targets,
        args.joint_limits,
        args.servo_map,
        args.gripper_profile,
        args.robot_config,
        args.home_pose,
    ]


def validate_required_files(args):
    missing = [path for path in required_config_paths(args) if path and not os.path.exists(path)]
    if missing:
        raise MovebookPhysicalSequenceError("Required file(s) missing: {}".format(", ".join(missing)))


def validate_calibration_for_move(square_targets_path, move_uci):
    document = robot_square_map.load_square_targets(square_targets_path)
    squares = document.get("squares") or {}
    source = str(move_uci)[:2].lower()
    destination = str(move_uci)[2:4].lower()

    source_info = squares.get(source)
    if not isinstance(source_info, dict):
        raise MovebookPhysicalSequenceError("Move {} rejected: missing {}".format(move_uci, source))
    destination_info = squares.get(destination)
    if not isinstance(destination_info, dict):
        raise MovebookPhysicalSequenceError("Move {} rejected: missing {}".format(move_uci, destination))

    checks = [
        (source, "above_pose", source_info),
        (source, "pick_pose", source_info),
        (destination, "above_pose", destination_info),
        (destination, "place_pose", destination_info),
    ]
    for square, pose_name, square_info in checks:
        if square_info.get(pose_name) is None:
            raise MovebookPhysicalSequenceError(
                "Move {} rejected: missing {}.{}".format(move_uci, square, pose_name)
            )
    return {
        "source_square": source,
        "destination_square": destination,
        "source": source_info,
        "destination": destination_info,
    }


def build_open_loop_args(args, source_square, destination_square, open_loop_module):
    return SimpleNamespace(
        source=source_square,
        dest=destination_square,
        targets=args.square_targets,
        joint_limits=args.joint_limits,
        servo_map=args.servo_map,
        gripper_profile=args.gripper_profile,
        robot_config=args.robot_config,
        home_pose=args.home_pose,
        real=True,
        confirm_text=open_loop_module.EXPECTED_CONFIRM_TEXT,
        pause_each=bool(args.pause_each),
        step_size_ticks=args.step_size_ticks,
        step_delay=args.step_delay,
        settle_time=args.settle_time,
        gripper_step_size_ticks=args.gripper_step_size_ticks,
        gripper_step_delay=args.gripper_step_delay,
        log=args.open_loop_log,
        output_json=args.output_json,
        piece="piece",
        allow_same_square=False,
        allow_place_uses_pick=False,
        return_home_after=True,
    )


def build_open_loop_command(args, source_square, destination_square, confirm_text):
    command = [
        "python3",
        "tools/test_open_loop_pick_place.py",
        "--source", source_square,
        "--dest", destination_square,
        "--targets", display_path(args.square_targets),
        "--joint-limits", display_path(args.joint_limits),
        "--servo-map", display_path(args.servo_map),
        "--gripper-profile", display_path(args.gripper_profile),
        "--robot-config", display_path(args.robot_config),
        "--home-pose", display_path(args.home_pose),
        "--real",
        "--confirm-text", confirm_text,
        "--step-size-ticks", str(args.step_size_ticks),
        "--step-delay", str(args.step_delay),
        "--settle-time", str(args.settle_time),
        "--gripper-step-size-ticks", str(args.gripper_step_size_ticks),
        "--gripper-step-delay", str(args.gripper_step_delay),
        "--piece", "piece",
        "--log", display_path(args.open_loop_log),
        "--output-json", display_path(args.output_json),
        "--return-home-after",
    ]
    if args.pause_each:
        command.append("--pause-each")
    return " ".join(shlex.quote(part) for part in command)


def prompt_for_phrase(input_fn, phrase):
    typed = input_fn("> ")
    if typed is None:
        typed = ""
    return str(typed).strip() == phrase


def prompt_for_enter(input_fn, prompt_text, abort_label):
    print(prompt_text)
    response = input_fn("> ")
    if response is None:
        response = ""
    response = str(response).strip()
    if response == "":
        return
    if response.lower() == "q":
        raise MovebookPhysicalSequenceError("operator aborted at {}".format(abort_label))
    raise MovebookPhysicalSequenceError(
        "{} rejected: expected Enter or q, got {!r}".format(abort_label, response)
    )


def print_speed_settings(args):
    print("Speed settings:")
    print("  step_size_ticks: {}".format(args.step_size_ticks))
    print("  step_delay: {}".format(args.step_delay))
    print("  settle_time: {}".format(args.settle_time))
    print("  gripper_step_size_ticks: {}".format(args.gripper_step_size_ticks))
    print("  gripper_step_delay: {}".format(args.gripper_step_delay))


def execute_open_loop(args, source_square, destination_square, open_loop_module):
    run_args = build_open_loop_args(args, source_square, destination_square, open_loop_module)
    open_loop_module.validate_inputs(run_args)
    exit_code, result = open_loop_module.run(run_args)
    success = bool(exit_code == 0 and not result.get("aborted"))
    if not success:
        reason = result.get("abort_reason") or "open_loop_exit_code_{}".format(exit_code)
        raise MovebookPhysicalSequenceError(reason)
    return result


def build_record(move_number, total_moves, entry, mode, calibration_ok, execution_attempted,
                 execution_success, failure_reason, output_json_path):
    robot_move = entry["robot_move"]
    return {
        "timestamp": utc_now_text(),
        "move_index": int(move_number),
        "move_total": int(total_moves),
        "expected_human_move": entry["expected_human_move"],
        "robot_move": robot_move,
        "source_square": robot_move[:2],
        "destination_square": robot_move[2:4],
        "mode": mode,
        "calibration_ok": bool(calibration_ok),
        "execution_attempted": bool(execution_attempted),
        "execution_success": bool(execution_success),
        "failure_reason": failure_reason,
        "output_json_path": output_json_path,
    }


def print_move_header(move_number, total_moves, entry):
    robot_move = entry["robot_move"]
    print("")
    print("Move {}/{}".format(move_number, total_moves))
    print("Expected human move: {}".format(entry["expected_human_move"]))
    print("Robot move: {}".format(robot_move))
    print("Source: {}".format(robot_move[:2]))
    print("Destination: {}".format(robot_move[2:4]))


def run_sequence(args, input_fn=input, open_loop_module=None):
    mode = "real" if args.real else "dry_run"
    validate_required_files(args)
    all_entries = load_movebook_entries(args.movebook)
    entries = select_entries(all_entries, args.start_index, args.max_moves)
    if not entries:
        raise MovebookPhysicalSequenceError("No movebook entries selected.")

    if open_loop_module is None:
        import tools.test_open_loop_pick_place as open_loop_module

    records = []
    any_failure = False
    total_moves = len(entries)
    confirm_text = open_loop_module.EXPECTED_CONFIRM_TEXT

    print("Mode: {}".format("REAL" if args.real else "DRY-RUN"))
    print("Movebook: {}".format(args.movebook))
    print("Selected moves: {}".format(total_moves))

    for offset, entry in enumerate(entries):
        move_number = int(args.start_index) + offset
        robot_move = entry["robot_move"]
        source_square = robot_move[:2]
        destination_square = robot_move[2:4]
        calibration_ok = False
        execution_attempted = False
        execution_success = False
        failure_reason = None

        print_move_header(offset + 1, total_moves, entry)
        try:
            validate_calibration_for_move(args.square_targets, robot_move)
            calibration_ok = True
            print("Calibration: OK")
            print("Physical source: {}".format(source_square))
            print("Physical destination: {}".format(destination_square))
            print_speed_settings(args)
            print("Open-loop command:")
            print(build_open_loop_command(args, source_square, destination_square, confirm_text))

            if args.real:
                if args.enter_to_advance:
                    print("")
                    prompt_for_enter(
                        input_fn,
                        "Press Enter after you have physically made the human move.",
                        "human move confirmation for {}".format(entry["expected_human_move"]),
                    )
                    print("")
                    print("Robot will now execute {}.".format(robot_move))
                    prompt_for_enter(
                        input_fn,
                        "Press Enter to execute, or type q then Enter to abort.",
                        "robot execution confirmation for {}".format(robot_move),
                    )
                else:
                    human_phrase = "HUMAN_DONE {}".format(entry["expected_human_move"])
                    execute_phrase = "EXECUTE {}".format(robot_move)
                    print("")
                    print("After you physically make the human move, type:")
                    print(human_phrase)
                    if not prompt_for_phrase(input_fn, human_phrase):
                        raise MovebookPhysicalSequenceError(
                            "Move {} rejected: expected confirmation {}".format(robot_move, human_phrase)
                        )
                    print("")
                    print("Before robot motion, type:")
                    print(execute_phrase)
                    if not prompt_for_phrase(input_fn, execute_phrase):
                        raise MovebookPhysicalSequenceError(
                            "Move {} rejected: expected confirmation {}".format(robot_move, execute_phrase)
                        )
                execution_attempted = True
                execute_open_loop(args, source_square, destination_square, open_loop_module)
                execution_success = True
                print("Move {} succeeded.".format(robot_move))
            else:
                print("Dry-run: hardware execution skipped.")
                execution_success = True
        except Exception as exc:
            any_failure = True
            failure_reason = str(exc)
            print("ERROR: {}".format(failure_reason))

        record = build_record(
            move_number,
            len(all_entries),
            entry,
            mode,
            calibration_ok,
            execution_attempted,
            execution_success,
            failure_reason,
            args.output_json,
        )
        append_jsonl(args.open_loop_log, record)
        records.append(record)
        write_json(args.output_json, {"records": records, "completed_at": utc_now_text()})

        if failure_reason and not args.continue_on_failure:
            break

    return 1 if any_failure else 0, records


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="Execute real hardware motion after strict confirmations.")
    mode.add_argument("--dry-run", action="store_true", help="Validate and print the sequence without moving hardware.")
    parser.add_argument("--movebook", default=DEFAULT_MOVEBOOK)
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--max-moves", type=int, default=None)
    parser.add_argument("--pause-each", action="store_true")
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--enter-to-advance", action="store_true")
    parser.add_argument("--square-targets", default=DEFAULT_SQUARE_TARGETS)
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS)
    parser.add_argument("--servo-map", default=DEFAULT_SERVO_MAP)
    parser.add_argument("--gripper-profile", default=DEFAULT_GRIPPER_PROFILE)
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE)
    parser.add_argument("--open-loop-log", default=DEFAULT_OPEN_LOOP_LOG)
    parser.add_argument("--output-json", default=DEFAULT_OUTPUT_JSON)
    parser.add_argument("--step-size-ticks", type=int, default=15)
    parser.add_argument("--step-delay", type=float, default=0.05)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--gripper-step-size-ticks", type=int, default=15)
    parser.add_argument("--gripper-step-delay", type=float, default=0.03)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.real and args.dry_run:
        raise MovebookPhysicalSequenceError("Choose either --real or --dry-run.")
    try:
        exit_code, _records = run_sequence(args)
    except MovebookPhysicalSequenceError as exc:
        print("ERROR: {}".format(exc))
        return 1
    return int(exit_code)


if __name__ == "__main__":
    sys.exit(main())
