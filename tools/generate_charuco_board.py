#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0


def get_aruco_dict(name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available. Install opencv-contrib-python.")

    mapping = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }
    if name not in mapping:
        raise ValueError("Unsupported dictionary: {}".format(name))

    return cv2.aruco.getPredefinedDictionary(mapping[name])


def create_charuco_board(squares_x, squares_y, square_length_m, marker_length_m, dictionary):
    # OpenCV API differs between versions.
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length_m,
            marker_length_m,
            dictionary,
        )

    return cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_length_m,
        marker_length_m,
        dictionary,
    )


def draw_board(board, width_px, height_px):
    # OpenCV API differs between versions.
    if hasattr(board, "generateImage"):
        img = board.generateImage((width_px, height_px))
    else:
        img = board.draw((width_px, height_px))

    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    return img


def add_a4_canvas(board_img, dpi, margin_mm, label_text):
    page_w_px = int(round(A4_WIDTH_MM / 25.4 * dpi))
    page_h_px = int(round(A4_HEIGHT_MM / 25.4 * dpi))
    margin_px = int(round(margin_mm / 25.4 * dpi))

    canvas = np.full((page_h_px, page_w_px, 3), 255, dtype=np.uint8)

    h, w = board_img.shape[:2]
    x0 = (page_w_px - w) // 2
    y0 = margin_px

    canvas[y0:y0 + h, x0:x0 + w] = board_img

    cv2.putText(
        canvas,
        label_text,
        (margin_px, page_h_px - margin_px),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )

    return canvas


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="data/calibration/hand_eye/charuco_board")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-mm", type=float, default=30.0)
    parser.add_argument("--marker-mm", type=float, default=22.0)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--margin-mm", type=float, default=15.0)
    args = parser.parse_args()

    if args.marker_mm >= args.square_mm:
        raise ValueError("--marker-mm must be smaller than --square-mm")

    board_w_mm = args.squares_x * args.square_mm
    board_h_mm = args.squares_y * args.square_mm

    max_w_mm = A4_WIDTH_MM - 2 * args.margin_mm
    max_h_mm = A4_HEIGHT_MM - 2 * args.margin_mm

    if board_w_mm > max_w_mm or board_h_mm > max_h_mm:
        raise ValueError(
            "Board does not fit A4 with margins. "
            "Board = {:.1f} x {:.1f} mm, usable A4 = {:.1f} x {:.1f} mm".format(
                board_w_mm, board_h_mm, max_w_mm, max_h_mm
            )
        )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dictionary = get_aruco_dict(args.dictionary)

    board = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_mm / 1000.0,
        args.marker_mm / 1000.0,
        dictionary,
    )

    board_w_px = int(round(board_w_mm / 25.4 * args.dpi))
    board_h_px = int(round(board_h_mm / 25.4 * args.dpi))

    board_img = draw_board(board, board_w_px, board_h_px)

    label = (
        "ChArUco {}x{} | square={:.1f}mm | marker={:.1f}mm | {}".format(
            args.squares_x,
            args.squares_y,
            args.square_mm,
            args.marker_mm,
            args.dictionary,
        )
    )

    a4_img = add_a4_canvas(board_img, args.dpi, args.margin_mm, label)

    png_path = output_dir / "charuco_a4.png"
    meta_path = output_dir / "charuco_a4_metadata.json"

    cv2.imwrite(str(png_path), a4_img)

    metadata = {
        "target_type": "charuco",
        "paper": "A4",
        "dpi": args.dpi,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_length_m": args.square_mm / 1000.0,
        "marker_length_m": args.marker_mm / 1000.0,
        "square_length_mm": args.square_mm,
        "marker_length_mm": args.marker_mm,
        "board_width_mm": board_w_mm,
        "board_height_mm": board_h_mm,
        "dictionary": args.dictionary,
        "margin_mm": args.margin_mm,
        "output_png": str(png_path),
    }

    with open(str(meta_path), "w") as f:
        json.dump(metadata, f, indent=2, sort_keys=True)

    print("Saved:", png_path)
    print("Saved:", meta_path)
    print("Board size: {:.1f} mm x {:.1f} mm".format(board_w_mm, board_h_mm))
    print("square_length_m:", metadata["square_length_m"])
    print("marker_length_m:", metadata["marker_length_m"])


if __name__ == "__main__":
    main()
