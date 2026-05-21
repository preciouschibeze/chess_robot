"""Match changed-square transition evidence to legal chess moves."""

import chess

try:
    from dataclasses import dataclass
except ImportError:
    def dataclass(*args, **kwargs):
        def _wrap(cls):
            return cls
        return _wrap


VALID_SQUARES = frozenset(chess.SQUARE_NAMES)


@dataclass(frozen=True)
class TransitionEvidence(object):
    def __init__(self, removed, added, uncertain, source_path=None):
        self.removed = removed
        self.added = added
        self.uncertain = uncertain
        self.source_path = source_path


@dataclass(frozen=True)
class MoveCandidate(object):
    def __init__(
        self,
        uci,
        san,
        move_type,
        expected_removed,
        expected_added,
        score,
        notes,
    ):
        self.uci = uci
        self.san = san
        self.move_type = move_type
        self.expected_removed = expected_removed
        self.expected_added = expected_added
        self.score = score
        self.notes = notes


@dataclass(frozen=True)
class MatchResult(object):
    def __init__(self, status, accepted_move, candidates, evidence, fen_before, message):
        self.status = status
        self.accepted_move = accepted_move
        self.candidates = candidates
        self.evidence = evidence
        self.fen_before = fen_before
        self.message = message


def _to_square_set(values):
    if values is None:
        return frozenset()
    normalized = set()
    for value in values:
        if not isinstance(value, str):
            continue
        square = value.strip().lower()
        if square in VALID_SQUARES:
            normalized.add(square)
    return frozenset(normalized)


def evidence_from_sets(removed, added, uncertain=None, source_path=None):
    return TransitionEvidence(
        removed=_to_square_set(removed),
        added=_to_square_set(added),
        uncertain=_to_square_set(uncertain),
        source_path=source_path,
    )


def normalise_transition_result_json(raw, source_path=None):
    removed = set(raw.get("removed_squares") or [])
    added = set(raw.get("added_squares") or [])
    uncertain = set(raw.get("uncertain_squares") or [])

    changed_squares = raw.get("changed_squares")
    if isinstance(changed_squares, list):
        for entry in changed_squares:
            if not isinstance(entry, dict):
                continue
            square = entry.get("square")
            if not isinstance(square, str):
                continue
            square = square.strip().lower()
            if square not in VALID_SQUARES:
                continue

            transition = entry.get("transition") or entry.get("type") or entry.get("change_type") or ""
            transition = str(transition).lower()
            before = entry.get("before")
            if before is None:
                before = entry.get("state_before")
            if before is None:
                before = entry.get("previous_state")
            after = entry.get("after")
            if after is None:
                after = entry.get("state_after")
            if after is None:
                after = entry.get("current_state")
            before_text = str(before).lower() if before is not None else ""
            after_text = str(after).lower() if after is not None else ""

            if "uncertain" in transition or "unknown" in transition:
                uncertain.add(square)
            elif "remove" in transition or "from" in transition or (before_text == "occupied" and after_text == "empty"):
                removed.add(square)
            elif "add" in transition or "to" in transition or (before_text == "empty" and after_text == "occupied"):
                added.add(square)

    changes = raw.get("changes")
    if isinstance(changes, list):
        for entry in changes:
            if not isinstance(entry, dict):
                continue
            square = entry.get("square")
            if not isinstance(square, str):
                continue
            square = square.strip().lower()
            if square not in VALID_SQUARES:
                continue
            transition = str(entry.get("transition") or entry.get("type") or "").lower()
            if "remove" in transition:
                removed.add(square)
            elif "add" in transition:
                added.add(square)
            elif "uncertain" in transition:
                uncertain.add(square)

    squares = raw.get("squares")
    if isinstance(squares, dict):
        for square, value in squares.items():
            if not isinstance(square, str):
                continue
            square = square.strip().lower()
            if square not in VALID_SQUARES:
                continue
            text = str(value).lower()
            if text in ("removed", "empty"):
                removed.add(square)
            elif text in ("added", "occupied"):
                added.add(square)
            elif text == "uncertain":
                uncertain.add(square)

    return evidence_from_sets(removed, added, uncertain, source_path=source_path)


def _move_type(board, move):
    if board.is_castling(move):
        return "castle"
    if move.promotion:
        return "promotion"
    if board.is_en_passant(move):
        return "en_passant"
    if board.is_capture(move):
        return "capture"
    return "quiet"


def expected_occupancy_delta(board, move):
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)
    move_type = _move_type(board, move)

    if move_type == "quiet":
        return set([from_sq]), set([to_sq]), move_type
    if move_type == "capture":
        return set([from_sq]), set(), move_type
    if move_type == "promotion":
        if board.is_capture(move):
            return set([from_sq]), set(), move_type
        return set([from_sq]), set([to_sq]), move_type
    if move_type == "en_passant":
        return set([from_sq]), set(), move_type
    return set([from_sq]), set([to_sq]), move_type


def _candidate(board, move, move_type, expected_removed, expected_added, notes=()):
    return MoveCandidate(
        uci=move.uci(),
        san=board.san(move),
        move_type=move_type,
        expected_removed=frozenset(expected_removed),
        expected_added=frozenset(expected_added),
        score=1.0,
        notes=tuple(notes),
    )


def _invalid_reason(evidence):
    r_count = len(evidence.removed)
    a_count = len(evidence.added)
    if r_count == 0 and a_count == 0:
        return "No removed or added squares in evidence"
    if r_count == 1 and a_count in (0, 1):
        return None
    if r_count == 2 and a_count == 2:
        return "Evidence looks like multiple moves; not supported"
    if r_count > 2 or a_count > 2:
        return "Too many changed squares for single-move matching"
    return "Unsupported removed/added square count for minimal matcher"


def match_legal_moves(board, evidence):
    invalid_reason = _invalid_reason(evidence)
    fen_before = board.fen()
    if invalid_reason:
        return MatchResult("invalid_evidence", None, tuple(), evidence, fen_before, invalid_reason)

    capture_like = len(evidence.removed) == 1 and len(evidence.added) == 0
    quiet_like = len(evidence.removed) == 1 and len(evidence.added) == 1

    candidates = []
    unsupported_seen = False
    for move in board.legal_moves:
        expected_removed, expected_added, move_type = expected_occupancy_delta(board, move)

        if move_type in ("castle", "en_passant"):
            unsupported_seen = True
            continue

        expected_removed_fs = frozenset(expected_removed)
        expected_added_fs = frozenset(expected_added)

        if quiet_like:
            if expected_removed_fs == evidence.removed and expected_added_fs == evidence.added:
                notes = ()
                if move_type == "promotion":
                    notes = ("promotion_variant",)
                candidates.append(_candidate(board, move, move_type, expected_removed, expected_added, notes=notes))
        elif capture_like:
            src_square = next(iter(evidence.removed))
            if move_type in ("capture", "promotion") and chess.square_name(move.from_square) == src_square:
                notes = ()
                if move_type == "promotion" and move.promotion:
                    notes = ("promotion_variant",)
                candidates.append(_candidate(board, move, move_type, expected_removed, expected_added, notes=notes))

    if not candidates:
        if unsupported_seen:
            return MatchResult(
                "unsupported",
                None,
                tuple(),
                evidence,
                fen_before,
                "No candidates in minimal matcher; only unsupported special moves may fit",
            )
        return MatchResult("no_match", None, tuple(), evidence, fen_before, "No legal move matched evidence")

    candidates_sorted = tuple(sorted(candidates, key=lambda c: c.uci))
    if len(candidates_sorted) == 1:
        return MatchResult("unique", candidates_sorted[0].uci, candidates_sorted, evidence, fen_before, "Unique legal move matched")

    has_promotion_mix = any(c.move_type == "promotion" for c in candidates_sorted)
    message = "Multiple legal moves matched evidence"
    if has_promotion_mix:
        message = "Ambiguous promotion variants matched same occupancy evidence"
    return MatchResult("ambiguous", None, candidates_sorted, evidence, fen_before, message)
