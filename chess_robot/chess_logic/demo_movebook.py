"""Load and validate constrained demo movebook responses."""

from __future__ import absolute_import

import os
import re

try:
    import yaml
except ImportError:  # pragma: no cover - runtime image dependency
    yaml = None


_UCI_RE = re.compile(r"^[a-h][1-8][a-h][1-8][qrbn]?$")
DEFAULT_MOVEBOOK_PATH = "configs/demo_movebook.yaml"


class DemoMovebookError(RuntimeError):
    """Raised when movebook configuration is invalid or missing."""


class DemoMovebook(object):
    """Dictionary-backed mapping from confirmed human move to robot reply."""

    def __init__(self, mapping, source_path=None):
        self._mapping = dict(mapping)
        self.source_path = source_path

    @staticmethod
    def is_uci_like(value):
        if not isinstance(value, str):
            return False
        return bool(_UCI_RE.match(value.strip().lower()))

    @classmethod
    def from_path(cls, path=DEFAULT_MOVEBOOK_PATH):
        if yaml is None:
            raise DemoMovebookError("PyYAML is required to load movebook files.")

        if not path:
            raise DemoMovebookError("Movebook path is required.")
        if not os.path.exists(path):
            raise DemoMovebookError("Movebook file does not exist: {}".format(path))

        with open(path, "r") as handle:
            data = yaml.safe_load(handle) or {}

        if not isinstance(data, dict):
            raise DemoMovebookError("Movebook YAML must contain a mapping.")

        mapping = data.get("demo_movebook")
        if not isinstance(mapping, dict) or not mapping:
            raise DemoMovebookError("Movebook YAML must contain non-empty demo_movebook mapping.")

        normalized = {}
        for human_move, robot_move in mapping.items():
            if not cls.is_uci_like(human_move):
                raise DemoMovebookError(
                    "Invalid human move key in movebook: {!r}".format(human_move)
                )
            if not cls.is_uci_like(robot_move):
                raise DemoMovebookError(
                    "Invalid robot move value for {}: {!r}".format(human_move, robot_move)
                )
            key = human_move.strip().lower()
            value = robot_move.strip().lower()
            normalized[key] = value

        return cls(normalized, source_path=path)

    def has_human_move(self, human_move):
        key = str(human_move).strip().lower()
        return key in self._mapping

    def robot_reply(self, human_move):
        key = str(human_move).strip().lower()
        if key not in self._mapping:
            raise DemoMovebookError(
                "No robot reply configured for confirmed human move {}.".format(key)
            )
        return self._mapping[key]

    def to_dict(self):
        return dict(self._mapping)
