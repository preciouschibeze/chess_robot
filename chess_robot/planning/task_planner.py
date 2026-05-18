"""Convert legal chess moves into symbolic dry-run physical action plans."""

from __future__ import absolute_import

import chess

from chess_robot.planning.move_plan import MovePlan, PhysicalAction


def classify_move(board, move):
    """Classify a move by type against the current board state."""
    if move not in board.legal_moves:
        return "illegal"
    if board.is_castling(move):
        return "castle"
    if board.is_en_passant(move):
        return "en_passant"
    if move.promotion is not None:
        return "promotion"
    if board.is_capture(move):
        return "capture"
    return "quiet"


def _quiet_actions(source, destination):
    return [
        PhysicalAction("move_home"),
        PhysicalAction("move_above_square", {"square": source}),
        PhysicalAction("descend_to_pick", {"square": source}),
        PhysicalAction("close_gripper"),
        PhysicalAction("lift_from_square", {"square": source}),
        PhysicalAction("move_above_square", {"square": destination}),
        PhysicalAction("descend_to_place", {"square": destination}),
        PhysicalAction("open_gripper"),
        PhysicalAction("lift_from_square", {"square": destination}),
        PhysicalAction("move_home"),
    ]


def _capture_actions(source, destination, capture_zone_name):
    return [
        PhysicalAction("move_home"),
        PhysicalAction("move_above_square", {"square": destination}),
        PhysicalAction("descend_to_pick", {"square": destination}),
        PhysicalAction("close_gripper"),
        PhysicalAction("lift_from_square", {"square": destination}),
        PhysicalAction("move_to_capture_zone", {"zone": capture_zone_name}),
        PhysicalAction("open_gripper"),
        PhysicalAction("move_above_square", {"square": source}),
        PhysicalAction("descend_to_pick", {"square": source}),
        PhysicalAction("close_gripper"),
        PhysicalAction("lift_from_square", {"square": source}),
        PhysicalAction("move_above_square", {"square": destination}),
        PhysicalAction("descend_to_place", {"square": destination}),
        PhysicalAction("open_gripper"),
        PhysicalAction("lift_from_square", {"square": destination}),
        PhysicalAction("move_home"),
    ]


def plan_chess_move(fen, move_uci, capture_zone_name="capture_zone"):
    """Plan a legal chess move into symbolic physical actions only."""
    board = chess.Board(fen)
    fen_before = board.fen()

    try:
        move = chess.Move.from_uci(move_uci)
    except ValueError:
        return MovePlan(
            move_uci=move_uci,
            move_san="",
            move_type="illegal",
            actions=[],
            requires_capture_zone=False,
            supported=False,
            notes=["Invalid UCI format."],
            fen_before=fen_before,
            fen_after_if_applied=None,
        )

    move_type = classify_move(board, move)
    if move_type == "illegal":
        return MovePlan(
            move_uci=move_uci,
            move_san="",
            move_type="illegal",
            actions=[],
            requires_capture_zone=False,
            supported=False,
            notes=["Move is not legal in the provided FEN."],
            fen_before=fen_before,
            fen_after_if_applied=None,
        )

    move_san = board.san(move)

    if move_type in ("castle", "en_passant", "promotion"):
        return MovePlan(
            move_uci=move_uci,
            move_san=move_san,
            move_type=move_type,
            actions=[],
            requires_capture_zone=False,
            supported=False,
            notes=["Move type '%s' is not yet supported by dry-run task planner." % move_type],
            fen_before=fen_before,
            fen_after_if_applied=None,
        )

    source = chess.square_name(move.from_square)
    destination = chess.square_name(move.to_square)

    if move_type == "quiet":
        actions = _quiet_actions(source, destination)
        requires_capture_zone = False
    else:
        actions = _capture_actions(source, destination, capture_zone_name)
        requires_capture_zone = True

    board_after = chess.Board(fen_before)
    board_after.push(move)

    return MovePlan(
        move_uci=move_uci,
        move_san=move_san,
        move_type=move_type,
        actions=actions,
        requires_capture_zone=requires_capture_zone,
        supported=True,
        notes=[],
        fen_before=fen_before,
        fen_after_if_applied=board_after.fen(),
    )
