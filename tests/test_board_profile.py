import pytest

from chess_robot.calibration.board_profile import (
    BOARD_ORIENTATION,
    BoardProfile,
    grid_to_square,
    square_names,
)


def make_grid_points():
    return [[[col * 10, row * 10] for col in range(9)] for row in range(9)]


def make_squares():
    return {name: {} for name in square_names()}


def make_profile():
    return BoardProfile(
        board_orientation=BOARD_ORIENTATION,
        grid_points=make_grid_points(),
        squares=make_squares(),
        ignored_regions=[{"name": "fixture", "polygon": [[1, 1], [2, 1], [2, 2], [1, 2]]}],
    )


def test_board_profile_derives_square_geometry_from_grid():
    profile = make_profile()

    assert profile.square_polygon("h1") == [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
    assert profile.square_centre("h1") == [5.0, 5.0]
    assert profile.square_center("h1") == [5.0, 5.0]
    assert profile.square_crop_polygon("h1") == profile.square_polygon("h1")
    assert profile.ignored_regions == [
        {"name": "fixture", "polygon": [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]}
    ]


def test_board_profile_loads_and_saves_yaml(tmp_path):
    profile_path = tmp_path / "board_profile.yaml"
    saved = make_profile().save(profile_path)

    loaded = BoardProfile.load(saved)

    assert loaded.board_orientation == BOARD_ORIENTATION
    assert loaded.square_polygon("a8") == [[70.0, 70.0], [80.0, 70.0], [80.0, 80.0], [70.0, 80.0]]
    assert loaded.centre("a8") == [75.0, 75.0]
    assert loaded.crop_polygon("a8") == loaded.square_polygon("a8")


def test_board_profile_rejects_wrong_orientation():
    with pytest.raises(ValueError):
        BoardProfile(
            board_orientation="white_side",
            grid_points=make_grid_points(),
            squares=make_squares(),
        )


def test_board_profile_rejects_non_9x9_grid_points():
    with pytest.raises(ValueError):
        BoardProfile(
            board_orientation=BOARD_ORIENTATION,
            grid_points=make_grid_points()[:8],
            squares=make_squares(),
        )


def test_board_profile_rejects_missing_square_entries():
    squares = make_squares()
    squares.pop(grid_to_square(0, 0))

    with pytest.raises(ValueError):
        BoardProfile(
            board_orientation=BOARD_ORIENTATION,
            grid_points=make_grid_points(),
            squares=squares,
        )


def test_board_profile_rejects_unknown_square_lookup():
    profile = make_profile()

    with pytest.raises(ValueError):
        profile.square_polygon("z9")
