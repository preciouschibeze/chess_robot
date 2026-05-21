import pytest

from chess_robot.calibration.board_profile import grid_to_square, square_names


def test_robot_black_corner_mapping():
    assert grid_to_square(0, 0) == "h1"
    assert grid_to_square(0, 7) == "a1"
    assert grid_to_square(7, 0) == "h8"
    assert grid_to_square(7, 7) == "a8"


@pytest.mark.parametrize(
    "row,col",
    [
        (-1, 0),
        (8, 0),
        (0, -1),
        (0, 8),
        (0.0, 0),
        (0, 0.0),
        ("0", 0),
        (0, "0"),
        (True, 0),
        (0, False),
    ],
)
def test_invalid_grid_indices_are_rejected(row, col):
    with pytest.raises(ValueError):
        grid_to_square(row, col)


def test_generated_square_names_are_unique():
    names = square_names()
    assert len(names) == 64
    assert len(set(names)) == 64
