#!/usr/bin/env python3
"""Constrained perception-to-action demo runner using a fixed movebook."""

from __future__ import absolute_import

import argparse
import csv
import json
import os
import shutil
import sys
from datetime import datetime
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
from chess_robot.calibration.camera_profile import load_camera_profile
from chess_robot.chess_logic.board_state import ChessBoardState
from chess_robot.chess_logic.demo_movebook import DemoMovebook, DemoMovebookError
from chess_robot.chess_logic.legal_move_matcher import match_legal_moves, normalise_transition_result_json
from chess_robot.gui.demo_terminal import DemoTerminal
from chess_robot.planning.task_planner import plan_chess_move
from chess_robot.robot.motion_primitives import resolve_move_plan
from chess_robot.vision.camera import capture_frame, get_camera_config, save_image
from chess_robot.vision.occupancy import (
    analyse_image,
    save_debug_outputs,
    save_result_json,
)
from chess_robot.vision.state_transition import compare_occupancy_snapshots, normalise_occupancy_snapshot

DEFAULT_MOVEBOOK = "configs/demo_movebook.yaml"
DEFAULT_LOG_PATH = "data/logs/demo_game.log"
DEFAULT_CSV_PATH = "data/logs/demo_game.csv"
DEFAULT_GUI_DIR = "data/gui"
DEFAULT_DEBUG_DIR = "data/debug"
DEFAULT_BOARD_PROFILE = "data/calibration/board/board_profile.yaml"
DEFAULT_SQUARE_TARGETS = "data/calibration/robot/square_targets.yaml"
DEFAULT_JOINT_LIMITS = "data/calibration/robot/joint_limits.yaml"
DEFAULT_HOME_POSE = "data/calibration/robot/home_pose.yaml"
DEFAULT_GRIPPER_PROFILE = "data/calibration/gripper/gripper_profile.yaml"
DEFAULT_SERVO_MAP = "data/calibration/robot/servo_map.yaml"
DEFAULT_ROBOT_CONFIG = "configs/robot.yaml"
DEFAULT_FEN_FILE = "data/game/current_fen_demo.txt"
DEFAULT_OPEN_LOOP_LOG = "data/logs/open_loop_pick_place.log"
DEFAULT_OPEN_LOOP_JSON = "data/debug/open_loop_pick_place_last.json"

CSV_HEADERS = [
    "timestamp",
    "move_number",
    "ply_count",
    "mode",
    "previous_fen",
    "human_move",
    "manual_confirmation_used",
    "changed_squares",
    "robot_move",
    "primitive_plan",
    "execution_success",
    "verification_success",
    "failure_reason",
    "debug_paths",
    "resulting_fen",
]


class DemoRunError(RuntimeError):
    """Raised when demo flow cannot continue safely."""


def utc_now_text():
    return datetime.utcnow().isoformat() + "Z"


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def ensure_dirs(paths):
    for path in paths:
        if path and not os.path.isdir(path):
            os.makedirs(path)


def copy_if_exists(src, dst):
    if src and os.path.exists(src):
        ensure_parent_dir(dst)
        shutil.copyfile(src, dst)
        return dst
    return None


def append_json_log(path, payload):
    ensure_parent_dir(path)
    with open(path, "a") as handle:
        handle.write(json.dumps(payload, sort_keys=True))
        handle.write("\n")


def append_csv_log(path, row):
    ensure_parent_dir(path)
    file_exists = os.path.exists(path)
    with open(path, "a") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_HEADERS)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="Run symbolic demo only (default mode).")
    parser.add_argument("--vision-only", action="store_true", help="Use vision/state tracking and never move hardware.")
    parser.add_argument("--real", action="store_true", help="Execute physical robot response with strict safety gates.")
    parser.add_argument("--movebook", default=DEFAULT_MOVEBOOK, help="Movebook YAML path.")
    parser.add_argument("--max-plies", type=int, default=None, help="Maximum plies to apply before exiting.")
    parser.add_argument("--max-moves", type=int, default=4, help="Maximum human turns to process.")
    parser.add_argument("--require-confirmation", dest="require_confirmation", action="store_true", default=True)
    parser.add_argument("--no-require-confirmation", dest="require_confirmation", action="store_false")
    parser.add_argument("--skip-camera", action="store_true", help="Disable camera/vision and use manual UCI entry.")
    parser.add_argument("--fen", default=None, help="Starting FEN or startpos.")
    parser.add_argument("--fen-file", default=DEFAULT_FEN_FILE, help="Optional FEN file path for load/save.")
    parser.add_argument("--board-profile", default=DEFAULT_BOARD_PROFILE, help="Board profile for occupancy analysis.")
    parser.add_argument("--empty-reference", default=None, help="Optional empty-board reference image for occupancy scoring.")
    parser.add_argument("--camera-config", default="configs/cameras.yaml", help="Camera config path.")
    parser.add_argument("--logs", default=DEFAULT_LOG_PATH, help="JSONL log path.")
    parser.add_argument("--csv", default=DEFAULT_CSV_PATH, help="CSV log path.")
    parser.add_argument("--gui-dir", default=DEFAULT_GUI_DIR, help="GUI/debug image directory.")
    parser.add_argument("--debug-dir", default=DEFAULT_DEBUG_DIR, help="Debug output directory.")
    parser.add_argument("--square-targets", default=DEFAULT_SQUARE_TARGETS)
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS)
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE)
    parser.add_argument("--gripper-profile", default=DEFAULT_GRIPPER_PROFILE)
    parser.add_argument("--servo-map", default=DEFAULT_SERVO_MAP)
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--open-loop-log", default=DEFAULT_OPEN_LOOP_LOG)
    parser.add_argument("--open-loop-json", default=DEFAULT_OPEN_LOOP_JSON)
    parser.add_argument("--step-size-ticks", type=int, default=5)
    parser.add_argument("--step-delay", type=float, default=0.15)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--gripper-step-size-ticks", type=int, default=5)
    parser.add_argument("--gripper-step-delay", type=float, default=0.08)
    return parser


def resolve_mode(args):
    selected = []
    if args.dry_run:
        selected.append("dry_run")
    if args.vision_only:
        selected.append("vision_only")
    if args.real:
        selected.append("real")
    if len(selected) > 1:
        raise DemoRunError("Choose only one mode flag among --dry-run, --vision-only, --real.")
    if not selected:
        return "dry_run"
    return selected[0]


def load_board_state(args):
    if args.fen:
        return ChessBoardState(args.fen)
    state = ChessBoardState()
    if args.fen_file and os.path.exists(args.fen_file):
        state.load_fen_file(args.fen_file, default_startpos=True)
    return state


def require_paths(paths):
    missing = [path for path in paths if path and (not os.path.exists(path))]
    if missing:
        raise DemoRunError("Required file(s) missing: {}".format(", ".join(missing)))


def verify_move_is_supported_quiet(board_state, move_uci, actor_label):
    move_type = board_state.classify_uci(move_uci)
    if move_type == "illegal":
        raise DemoRunError("{} move is illegal: {}".format(actor_label, move_uci))
    if move_type in ("capture", "castle", "en_passant", "promotion"):
        raise DemoRunError(
            "{} move {} is unsupported for constrained demo (type={}).".format(
                actor_label,
                move_uci,
                move_type,
            )
        )
    return move_type


def build_primitive_plan(board_state, move_uci, square_targets_path, home_pose_path, gripper_profile_path):
    plan = plan_chess_move(board_state.fen(), move_uci)
    if (not plan.supported) or plan.move_type != "quiet":
        raise DemoRunError(
            "Robot move {} could not be planned as supported non-capture action (type={}).".format(
                move_uci,
                plan.move_type,
            )
        )

    resolved = resolve_move_plan(
        plan_dict=plan.to_dict(),
        square_targets_path=square_targets_path,
        home_pose_path=home_pose_path,
        gripper_profile_path=gripper_profile_path,
    )
    primitive_names = [action.name for action in plan.actions]
    return plan, resolved, primitive_names


def _pick_or_place_pose(square_info):
    if not isinstance(square_info, dict):
        return None
    if square_info.get("pick_pose") is not None:
        return "pick_pose"
    if square_info.get("place_pose") is not None:
        return "place_pose"
    return None


def validate_square_targets_for_move(square_targets_path, move_uci):
    document = robot_square_map.load_square_targets(square_targets_path)
    squares = document.get("squares") or {}
    source = str(move_uci)[:2].lower()
    destination = str(move_uci)[2:4].lower()

    source_info = squares.get(source)
    if not isinstance(source_info, dict):
        raise DemoRunError("source square {} is not calibrated in square_targets.".format(source))
    destination_info = squares.get(destination)
    if not isinstance(destination_info, dict):
        raise DemoRunError("destination square {} is not calibrated in square_targets.".format(destination))

    missing = []
    if source_info.get("above_pose") is None:
        missing.append("squares.{}.above_pose".format(source))
    source_pick_key = _pick_or_place_pose(source_info)
    if source_pick_key is None:
        missing.append("squares.{}.pick_pose_or_place_pose".format(source))

    if destination_info.get("above_pose") is None:
        missing.append("squares.{}.above_pose".format(destination))
    destination_place_key = "place_pose" if destination_info.get("place_pose") is not None else _pick_or_place_pose(destination_info)
    if destination_place_key is None:
        missing.append("squares.{}.place_pose_or_pick_pose".format(destination))

    if missing:
        raise DemoRunError("Move {} rejected: calibration missing {}".format(move_uci, ", ".join(missing)))

    return {
        "source": source,
        "destination": destination,
        "source_pick_key": source_pick_key,
        "destination_place_key": destination_place_key,
    }


def save_board_state_png(board_state, output_path):
    try:
        import cv2
        import numpy as np
    except Exception:
        return None

    lines = ["Robot perspective: black", "FEN: {}".format(board_state.fen()), ""]
    lines.extend(board_state.ascii().splitlines())
    width = 1200
    height = max(240, 42 + 28 * len(lines))
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:] = (28, 28, 28)

    y = 32
    for line in lines:
        cv2.putText(canvas, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 1, cv2.LINE_AA)
        y += 28

    ensure_parent_dir(output_path)
    cv2.imwrite(output_path, canvas)
    return output_path


def capture_snapshot(args, label):
    camera_config = get_camera_config(config_path=args.camera_config)
    camera_index = camera_config.get("camera_index", camera_config.get("index", 0))
    width = camera_config.get("width")
    height = camera_config.get("height")

    frame = capture_frame(camera_index, width=width, height=height)
    raw_path = os.path.join(args.debug_dir, "{}_raw.png".format(label))
    save_image(raw_path, frame)

    analysis_path = raw_path
    undistorted_path = None
    calibration_path = camera_config.get("calibration_path")
    if calibration_path:
        calibration_abs = calibration_path
        if not os.path.isabs(calibration_abs):
            calibration_abs = os.path.join(ROOT, calibration_abs)
        if os.path.exists(calibration_abs):
            profile = load_camera_profile(calibration_abs)
            undistorted = profile.undistort(frame)
            undistorted_path = os.path.join(args.debug_dir, "{}_undistorted.png".format(label))
            save_image(undistorted_path, undistorted)
            analysis_path = undistorted_path

    occupancy_result = analyse_image(
        analysis_path,
        profile_path=args.board_profile,
        empty_reference_path=args.empty_reference,
    )
    snapshot = normalise_occupancy_snapshot(occupancy_result)

    occupancy_json = os.path.join(args.debug_dir, "{}_occupancy.json".format(label))
    save_result_json(occupancy_result, occupancy_json)
    snapshot_json = os.path.join(args.debug_dir, "{}_occupancy_snapshot.json".format(label))
    save_result_json(snapshot, snapshot_json)

    outputs = save_debug_outputs(analysis_path, args.board_profile, occupancy_result, args.debug_dir)
    latest_occ = copy_if_exists(outputs.get("occupancy_grid"), os.path.join(args.gui_dir, "latest_occupancy_grid.png"))

    return {
        "snapshot": snapshot,
        "analysis_image": analysis_path,
        "raw_image": raw_path,
        "undistorted_image": undistorted_path,
        "occupancy_json": occupancy_json,
        "snapshot_json": snapshot_json,
        "occupancy_grid": latest_occ,
        "debug_outputs": outputs,
    }


def render_transition_grid_if_available(previous_snapshot, current_snapshot, transition_result, output_path):
    try:
        from tools.detect_changed_squares import render_transition_grid
    except Exception:
        return None
    render_transition_grid(previous_snapshot, current_snapshot, transition_result, output_path)
    return output_path


def infer_human_move_candidate(board_state, previous_snapshot, current_snapshot):
    transition = compare_occupancy_snapshots(previous_snapshot, current_snapshot)
    evidence = normalise_transition_result_json(transition)
    match = match_legal_moves(board_state.board, evidence)
    candidate = match.accepted_move if match.status == "unique" else None
    changed = [entry.get("square") for entry in transition.get("changed_squares", []) if isinstance(entry, dict)]
    changed = [value for value in changed if isinstance(value, str)]
    return {
        "transition": transition,
        "match": match,
        "candidate": candidate,
        "changed_squares": changed,
    }


def read_user_input(prompt):
    try:
        return raw_input(prompt)
    except NameError:
        return input(prompt)


def choose_human_move(ui, args, board_state, candidate_info, can_rescan):
    candidate = candidate_info.get("candidate") if isinstance(candidate_info, dict) else None
    changed = candidate_info.get("changed_squares") if isinstance(candidate_info, dict) else []
    if changed is None:
        changed = []

    while True:
        ui.render_turn_status(
            changed_squares=changed,
            candidate_move=candidate,
            confirmed_human_move=None,
            movebook_reply=None,
            primitive_plan=[],
            execution_status="waiting_for_human_move",
            verification_status="n/a",
        )
        command = read_user_input("Human move command [Enter=accept candidate, m=manual, r=rescan, q=quit]: ").strip()

        if command.lower() == "q":
            return {"status": "quit"}

        if command.lower() == "r":
            if can_rescan:
                return {"status": "rescan"}
            ui.warn("Vision is unavailable; cannot rescan.")
            continue

        if command.lower() == "m":
            manual = read_user_input("Enter manual human move in UCI (example e2e4): ").strip().lower()
            if not manual:
                continue
            return {"status": "chosen", "move": manual, "manual": True}

        if command == "":
            if not candidate:
                ui.warn("No unique candidate is available; use m for manual move or r to rescan.")
                continue
            if args.require_confirmation:
                confirm = read_user_input("Accept detected candidate {}? [y/N]: ".format(candidate)).strip().lower()
                if confirm not in ("y", "yes"):
                    continue
            return {"status": "chosen", "move": candidate, "manual": False}

        proposed = command.lower()
        if len(proposed) in (4, 5):
            return {"status": "chosen", "move": proposed, "manual": True}


def prompt_real_execution_confirmation(robot_move):
    phrase = "EXECUTE_DEMO_MOVE"
    typed = read_user_input(
        "Type {} to execute real robot move {}: ".format(phrase, robot_move)
    ).strip()
    return typed == phrase


def run_open_loop_real(source_square, destination_square, args):
    import tools.test_open_loop_pick_place as open_loop_pick_place

    run_args = SimpleNamespace(
        source=source_square,
        dest=destination_square,
        targets=args.square_targets,
        joint_limits=args.joint_limits,
        servo_map=args.servo_map,
        gripper_profile=args.gripper_profile,
        robot_config=args.robot_config,
        real=True,
        confirm_text=open_loop_pick_place.EXPECTED_CONFIRM_TEXT,
        pause_each=False,
        step_size_ticks=args.step_size_ticks,
        step_delay=args.step_delay,
        settle_time=args.settle_time,
        gripper_step_size_ticks=args.gripper_step_size_ticks,
        gripper_step_delay=args.gripper_step_delay,
        log=args.open_loop_log,
        output_json=args.open_loop_json,
        piece="piece",
        allow_same_square=False,
        allow_place_uses_pick=False,
    )

    open_loop_pick_place.validate_inputs(run_args)
    exit_code, result = open_loop_pick_place.run(run_args)
    success = bool(exit_code == 0 and not result.get("aborted"))
    return success, result


def verify_robot_move_with_vision(board_before_robot, expected_robot_move, previous_snapshot, args):
    capture = capture_snapshot(args, "robot_verify")
    current_snapshot = capture["snapshot"]
    transition = compare_occupancy_snapshots(previous_snapshot, current_snapshot)
    evidence = normalise_transition_result_json(transition)
    match = match_legal_moves(board_before_robot.board, evidence)
    transition_grid = render_transition_grid_if_available(
        previous_snapshot,
        current_snapshot,
        transition,
        os.path.join(args.gui_dir, "latest_changed_squares.png"),
    )

    verified = bool(match.status == "unique" and match.accepted_move == expected_robot_move)
    reason = None
    if not verified:
        reason = "vision verification mismatch (status={}, detected={})".format(
            match.status,
            match.accepted_move,
        )

    changed = [entry.get("square") for entry in transition.get("changed_squares", []) if isinstance(entry, dict)]
    changed = [value for value in changed if isinstance(value, str)]

    debug_paths = {
        "robot_verify_snapshot": capture.get("snapshot_json"),
        "robot_verify_occupancy": capture.get("occupancy_json"),
        "robot_verify_grid": capture.get("occupancy_grid"),
        "robot_verify_transition_grid": transition_grid,
    }
    return verified, reason, changed, current_snapshot, debug_paths


def maybe_save_fen(board_state, fen_file):
    if fen_file:
        board_state.save_fen_file(fen_file)


def maybe_manual_verify_after_real_failure():
    typed = read_user_input("Verification failed. Type yes to accept robot move anyway, or anything else to stop: ").strip().lower()
    return typed == "yes"


def main():
    args = build_parser().parse_args()
    mode = resolve_mode(args)
    if mode == "dry_run":
        args.dry_run = True
        args.vision_only = False
        args.real = False
    elif mode == "vision_only":
        args.dry_run = False
        args.vision_only = True
        args.real = False
    else:
        args.dry_run = False
        args.vision_only = False
        args.real = True

    ensure_dirs([args.gui_dir, args.debug_dir, os.path.dirname(args.logs), os.path.dirname(args.csv)])

    ui = DemoTerminal()
    board_state = load_board_state(args)
    movebook = DemoMovebook.from_path(args.movebook)

    vision_enabled = not bool(args.skip_camera)
    if vision_enabled:
        require_paths([args.board_profile])
    if args.real:
        require_paths([
            args.joint_limits,
            args.home_pose,
            args.gripper_profile,
            args.square_targets,
            args.servo_map,
            args.robot_config,
        ])
        if vision_enabled:
            require_paths([args.board_profile])

    latest_snapshot = None
    if vision_enabled:
        try:
            baseline = capture_snapshot(args, "baseline")
            latest_snapshot = baseline["snapshot"]
            ui.info("Captured baseline occupancy snapshot.")
        except Exception as exc:
            ui.warn("Vision baseline unavailable: {}. Manual fallback enabled.".format(exc))
            latest_snapshot = None

    move_number = 0
    ply_count = 0
    running = True

    while running:
        if args.max_moves is not None and move_number >= int(args.max_moves):
            ui.info("Reached --max-moves {}. Stopping.".format(args.max_moves))
            break
        if args.max_plies is not None and ply_count >= int(args.max_plies):
            ui.info("Reached --max-plies {}. Stopping.".format(args.max_plies))
            break
        if board_state.board.is_game_over():
            ui.info("Game over by chess rules: {}".format(board_state.board.result()))
            break

        move_number += 1
        previous_fen = board_state.fen()
        ui.render_board_state(board_state, "robot_black_side")
        save_board_state_png(board_state, os.path.join(args.gui_dir, "latest_board_state.png"))

        command = read_user_input("Make a human move physically, then press Enter (q to quit): ").strip().lower()
        if command == "q":
            ui.info("Operator requested quit.")
            break

        candidate_info = {"candidate": None, "changed_squares": []}
        human_snapshot = None
        debug_paths = {}

        if vision_enabled and latest_snapshot is not None:
            try:
                capture = capture_snapshot(args, "human_turn_{}_scan".format(move_number))
                human_snapshot = capture["snapshot"]
                debug_paths.update({
                    "human_snapshot": capture.get("snapshot_json"),
                    "human_occupancy": capture.get("occupancy_json"),
                    "human_grid": capture.get("occupancy_grid"),
                })
                candidate_info = infer_human_move_candidate(board_state, latest_snapshot, human_snapshot)
                transition_grid = render_transition_grid_if_available(
                    latest_snapshot,
                    human_snapshot,
                    candidate_info.get("transition"),
                    os.path.join(args.gui_dir, "latest_changed_squares.png"),
                )
                if transition_grid:
                    debug_paths["human_transition_grid"] = transition_grid
            except Exception as exc:
                ui.warn("Vision scan failed: {}. Manual fallback enabled.".format(exc))
                candidate_info = {"candidate": None, "changed_squares": []}

        while True:
            decision = choose_human_move(ui, args, board_state, candidate_info, can_rescan=vision_enabled)
            if decision.get("status") == "quit":
                running = False
                break
            if decision.get("status") == "rescan":
                if not vision_enabled or latest_snapshot is None:
                    ui.warn("Cannot rescan without prior baseline snapshot.")
                    continue
                try:
                    capture = capture_snapshot(args, "human_turn_{}_rescan".format(move_number))
                    human_snapshot = capture["snapshot"]
                    debug_paths.update({
                        "human_snapshot": capture.get("snapshot_json"),
                        "human_occupancy": capture.get("occupancy_json"),
                        "human_grid": capture.get("occupancy_grid"),
                    })
                    candidate_info = infer_human_move_candidate(board_state, latest_snapshot, human_snapshot)
                    transition_grid = render_transition_grid_if_available(
                        latest_snapshot,
                        human_snapshot,
                        candidate_info.get("transition"),
                        os.path.join(args.gui_dir, "latest_changed_squares.png"),
                    )
                    if transition_grid:
                        debug_paths["human_transition_grid"] = transition_grid
                except Exception as exc:
                    ui.warn("Rescan failed: {}".format(exc))
                continue
            if decision.get("status") == "chosen":
                human_move = decision.get("move")
                manual_confirmation_used = bool(decision.get("manual"))
                try:
                    verify_move_is_supported_quiet(board_state, human_move, "Human")
                except DemoRunError as exc:
                    ui.warn(str(exc))
                    continue
                break

        if not running:
            break

        human_move_obj = board_state.apply_human_move(human_move)
        ply_count += 1
        latest_snapshot = human_snapshot if human_snapshot is not None else latest_snapshot

        try:
            robot_move = movebook.robot_reply(human_move)
        except DemoMovebookError as exc:
            failure_reason = str(exc)
            ui.error(failure_reason)
            row = {
                "timestamp": utc_now_text(),
                "move_number": move_number,
                "ply_count": ply_count,
                "mode": mode,
                "previous_fen": previous_fen,
                "human_move": human_move,
                "manual_confirmation_used": manual_confirmation_used,
                "changed_squares": ",".join(candidate_info.get("changed_squares") or []),
                "robot_move": "",
                "primitive_plan": "",
                "execution_success": False,
                "verification_success": False,
                "failure_reason": failure_reason,
                "debug_paths": json.dumps(debug_paths, sort_keys=True),
                "resulting_fen": board_state.fen(),
            }
            append_csv_log(args.csv, row)
            append_json_log(args.logs, row)
            break

        try:
            verify_move_is_supported_quiet(board_state, robot_move, "Robot")
            plan, resolved, primitive_names = build_primitive_plan(
                board_state,
                robot_move,
                args.square_targets,
                args.home_pose,
                args.gripper_profile,
            )
            if args.real and (not resolved.ready_for_execution):
                raise DemoRunError(
                    "Robot move {} failed primitive readiness: {}".format(
                        robot_move,
                        ", ".join(resolved.missing_calibration or ["unknown"])
                    )
                )
        except DemoRunError as exc:
            failure_reason = str(exc)
            ui.error(failure_reason)
            row = {
                "timestamp": utc_now_text(),
                "move_number": move_number,
                "ply_count": ply_count,
                "mode": mode,
                "previous_fen": previous_fen,
                "human_move": human_move,
                "manual_confirmation_used": manual_confirmation_used,
                "changed_squares": ",".join(candidate_info.get("changed_squares") or []),
                "robot_move": robot_move,
                "primitive_plan": "",
                "execution_success": False,
                "verification_success": False,
                "failure_reason": failure_reason,
                "debug_paths": json.dumps(debug_paths, sort_keys=True),
                "resulting_fen": board_state.fen(),
            }
            append_csv_log(args.csv, row)
            append_json_log(args.logs, row)
            break

        ui.render_turn_status(
            changed_squares=candidate_info.get("changed_squares") or [],
            candidate_move=candidate_info.get("candidate"),
            confirmed_human_move=human_move,
            movebook_reply=robot_move,
            primitive_plan=primitive_names,
            execution_status="pending",
            verification_status="pending",
        )

        if args.max_plies is not None and ply_count >= int(args.max_plies):
            ui.info("Reached --max-plies after human move; stopping before robot response.")
            maybe_save_fen(board_state, args.fen_file)
            break

        execution_success = True
        verification_success = not args.real
        verification_reason = None
        execution_payload = {}

        if args.real:
            try:
                move_targets = validate_square_targets_for_move(args.square_targets, robot_move)
                if not prompt_real_execution_confirmation(robot_move):
                    raise DemoRunError("Real execution cancelled: confirmation phrase mismatch.")
                execution_success, execution_payload = run_open_loop_real(
                    move_targets["source"],
                    move_targets["destination"],
                    args,
                )
                if not execution_success:
                    raise DemoRunError("Real execution failed or aborted by open-loop executor.")
            except Exception as exc:
                execution_success = False
                verification_success = False
                verification_reason = str(exc)

            if execution_success:
                board_before_robot = ChessBoardState(board_state.fen())
                if vision_enabled and latest_snapshot is not None:
                    try:
                        verified, reason, changed, robot_snapshot, verify_paths = verify_robot_move_with_vision(
                            board_before_robot,
                            robot_move,
                            latest_snapshot,
                            args,
                        )
                        debug_paths.update(verify_paths)
                        candidate_info["changed_squares"] = changed
                        if verified:
                            verification_success = True
                            latest_snapshot = robot_snapshot
                        else:
                            verification_success = False
                            verification_reason = reason
                    except Exception as exc:
                        verification_success = False
                        verification_reason = "vision verification failed: {}".format(exc)
                else:
                    verification_success = False
                    verification_reason = "vision verification unavailable"

                if not verification_success:
                    if maybe_manual_verify_after_real_failure():
                        verification_success = True
                        if verification_reason:
                            verification_reason = verification_reason + "; accepted manually"
                        else:
                            verification_reason = "accepted manually"

        if not args.real:
            execution_status = "dry_run_no_motion" if mode == "dry_run" else "vision_only_no_motion"
            ui.info("{}: symbolic robot move only, no hardware command sent.".format(execution_status))
            board_state.apply_robot_move(robot_move)
            ply_count += 1
            latest_snapshot = None
            verification_success = True
        else:
            if execution_success and verification_success:
                board_state.apply_robot_move(robot_move)
                ply_count += 1
            else:
                ui.error("Robot move not committed to board state: {}".format(verification_reason or "unknown failure"))
                running = False

        ui.render_turn_status(
            changed_squares=candidate_info.get("changed_squares") or [],
            candidate_move=candidate_info.get("candidate"),
            confirmed_human_move=human_move,
            movebook_reply=robot_move,
            primitive_plan=primitive_names,
            execution_status="ok" if execution_success else "failed",
            verification_status="ok" if verification_success else "failed",
        )

        maybe_save_fen(board_state, args.fen_file)
        save_board_state_png(board_state, os.path.join(args.gui_dir, "latest_board_state.png"))

        failure_reason = ""
        if not execution_success:
            failure_reason = verification_reason or "robot_execution_failed"
        elif not verification_success:
            failure_reason = verification_reason or "robot_verification_failed"

        if not candidate_info.get("changed_squares"):
            changed_csv = ""
        else:
            changed_csv = ",".join(candidate_info.get("changed_squares") or [])

        row = {
            "timestamp": utc_now_text(),
            "move_number": move_number,
            "ply_count": ply_count,
            "mode": mode,
            "previous_fen": previous_fen,
            "human_move": human_move,
            "manual_confirmation_used": manual_confirmation_used,
            "changed_squares": changed_csv,
            "robot_move": robot_move,
            "primitive_plan": "|".join(primitive_names),
            "execution_success": bool(execution_success),
            "verification_success": bool(verification_success),
            "failure_reason": failure_reason,
            "debug_paths": json.dumps(debug_paths, sort_keys=True),
            "resulting_fen": board_state.fen(),
        }
        append_csv_log(args.csv, row)
        append_json_log(args.logs, row)

        if not execution_success or not verification_success:
            break

    ui.info("Demo runner finished. Current FEN: {}".format(board_state.fen()))


if __name__ == "__main__":
    try:
        main()
    except (DemoRunError, DemoMovebookError) as exc:
        print("[demo][error] {}".format(exc))
        sys.exit(1)
