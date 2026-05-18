"""Data structures for dry-run chess move physical planning."""

from __future__ import absolute_import


class PhysicalAction(object):
    """Symbolic physical action; no hardware execution."""

    def __init__(self, name, params=None):
        self.name = name
        self.params = params or {}

    def to_dict(self):
        return {
            "name": self.name,
            "params": dict(self.params),
        }


class MovePlan(object):
    """Dry-run plan describing how a move would be executed physically."""

    def __init__(
        self,
        move_uci,
        move_san,
        move_type,
        actions,
        requires_capture_zone,
        supported,
        notes,
        fen_before,
        fen_after_if_applied,
    ):
        self.move_uci = move_uci
        self.move_san = move_san
        self.move_type = move_type
        self.actions = list(actions)
        self.requires_capture_zone = bool(requires_capture_zone)
        self.supported = bool(supported)
        self.notes = list(notes)
        self.fen_before = fen_before
        self.fen_after_if_applied = fen_after_if_applied

    def to_dict(self):
        return {
            "move_uci": self.move_uci,
            "move_san": self.move_san,
            "move_type": self.move_type,
            "supported": self.supported,
            "requires_capture_zone": self.requires_capture_zone,
            "fen_before": self.fen_before,
            "fen_after_if_applied": self.fen_after_if_applied,
            "actions": [action.to_dict() for action in self.actions],
            "notes": list(self.notes),
        }
