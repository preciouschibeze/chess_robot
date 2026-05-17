#!/usr/bin/env python3
"""Verify the Jetson Nano Python runtime without accessing hardware."""

from __future__ import print_function

import sys


def version_of(module, *names):
    for name in names:
        value = getattr(module, name, None)
        if value:
            return value
    return "unknown"


def main():
    print("Python: {}".format(sys.version.replace("\n", " ")))

    import cv2
    import numpy
    import yaml
    import chess
    import matplotlib
    import serial

    packages = [
        ("cv2", version_of(cv2, "__version__")),
        ("numpy", version_of(numpy, "__version__")),
        ("yaml", version_of(yaml, "__version__")),
        ("chess", version_of(chess, "__version__")),
        ("matplotlib", version_of(matplotlib, "__version__")),
        ("serial", version_of(serial, "__version__", "VERSION")),
    ]

    for name, version in packages:
        print("{}: {}".format(name, version))

    board = chess.Board()
    move = chess.Move.from_uci("e2e4")
    if move not in board.legal_moves:
        raise RuntimeError("Expected e2e4 to be legal from the starting position")

    print("chess.Board: ok")
    print("e2e4 legal: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
