import chess

from chess_robot.planning.task_planner import plan_chess_move


def test_quiet_d7d5_supported_from_post_e2e4():
    fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"
    plan = plan_chess_move(fen, "d7d5")

    assert plan.supported is True
    assert plan.move_type == "quiet"
    assert plan.move_san == "d5"
    assert len(plan.actions) == 10


def test_quiet_plan_has_source_and_destination_actions():
    fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"
    plan = plan_chess_move(fen, "d7d5")

    assert plan.actions[1].name == "move_above_square"
    assert plan.actions[1].params.get("square") == "d7"
    assert plan.actions[5].name == "move_above_square"
    assert plan.actions[5].params.get("square") == "d5"


def test_capture_e4d5_supported_after_e2e4_d7d5():
    fen = "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
    plan = plan_chess_move(fen, "e4d5")

    assert plan.supported is True
    assert plan.move_type == "capture"
    assert plan.requires_capture_zone is True


def test_capture_sequence_removes_destination_first_and_uses_capture_zone():
    fen = "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
    plan = plan_chess_move(fen, "e4d5", capture_zone_name="capture_zone")

    assert plan.actions[1].name == "move_above_square"
    assert plan.actions[1].params.get("square") == "d5"
    assert plan.actions[5].name == "move_to_capture_zone"
    assert plan.actions[5].params.get("zone") == "capture_zone"
    assert plan.actions[7].name == "move_above_square"
    assert plan.actions[7].params.get("square") == "e4"


def test_illegal_move_returns_unsupported_illegal():
    fen = chess.STARTING_FEN
    plan = plan_chess_move(fen, "e2e5")

    assert plan.supported is False
    assert plan.move_type == "illegal"
    assert plan.actions == []


def test_castling_returns_unsupported_castle():
    fen = "r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1"
    plan = plan_chess_move(fen, "e1g1")

    assert plan.supported is False
    assert plan.move_type == "castle"


def test_promotion_returns_unsupported_promotion():
    fen = "4k3/P7/8/8/8/8/8/4K3 w - - 0 1"
    plan = plan_chess_move(fen, "a7a8q")

    assert plan.supported is False
    assert plan.move_type == "promotion"


def test_en_passant_returns_unsupported_en_passant():
    fen = "4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1"
    plan = plan_chess_move(fen, "e5d6")

    assert plan.supported is False
    assert plan.move_type == "en_passant"
