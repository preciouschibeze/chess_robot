from __future__ import absolute_import

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.audit_joint_calibration_limits as audit_joint_calibration_limits
from chess_robot.robot.joint_calibration import angle_rad_to_tick
from chess_robot.robot.joint_calibration import tick_to_angle_rad


def _calibration():
    return {
        "ticks_per_rev": 4096,
        "provisional": True,
        "joint_order": ["base_yaw"],
        "warnings": [],
        "urdf_to_user": {"shoulder_pan": "base_yaw"},
        "joints": {
            "base_yaw": {
                "user_joint": "base_yaw",
                "urdf_joint": "shoulder_pan",
                "direction_sign": 1,
                "zero_tick": 2048,
            }
        },
    }


def _joint_limits(minimum, maximum):
    return {
        "shoulder_pan": {
            "provisional_min": int(minimum),
            "provisional_max": int(maximum),
        }
    }


def _target_for_tick(tick):
    calibration = _calibration()
    return {
        "target_name": "e4_above",
        "target_type": "square_above",
        "square": "e4",
        "success": True,
        "error_mm": 0.25,
        "shoulder_pan_rad": tick_to_angle_rad("base_yaw", tick, calibration),
    }


def test_angle_rad_to_tick_inverse_works_with_calibration_helper():
    calibration = _calibration()
    source_tick = 2633
    angle_rad = tick_to_angle_rad("base_yaw", source_tick, calibration)
    recovered_tick = angle_rad_to_tick("base_yaw", angle_rad, calibration)
    assert abs(recovered_tick - source_tick) <= 1


def test_no_violation_when_converted_tick_is_inside_limits():
    rows, target_records, joint_specs = audit_joint_calibration_limits.audit_targets(
        [_target_for_tick(2055)],
        _calibration(),
        _joint_limits(2000, 2100),
    )
    assert len(target_records) == 1
    assert len(joint_specs) == 1
    assert rows[0]["violation_type"] == "none"
    assert rows[0]["violation_ticks"] == 0
    assert rows[0]["margin_to_min_tick"] == 55
    assert rows[0]["margin_to_max_tick"] == 45


def test_below_min_violation_detected():
    rows, _, _ = audit_joint_calibration_limits.audit_targets(
        [_target_for_tick(1992)],
        _calibration(),
        _joint_limits(2000, 2100),
    )
    assert rows[0]["violation_type"] == "below_min"
    assert rows[0]["violation_ticks"] == 8
    assert rows[0]["margin_to_min_tick"] == -8
    assert rows[0]["margin_to_max_tick"] == 108


def test_above_max_violation_detected():
    rows, _, _ = audit_joint_calibration_limits.audit_targets(
        [_target_for_tick(2114)],
        _calibration(),
        _joint_limits(2000, 2100),
    )
    assert rows[0]["violation_type"] == "above_max"
    assert rows[0]["violation_ticks"] == 14
    assert rows[0]["margin_to_min_tick"] == 114
    assert rows[0]["margin_to_max_tick"] == -14


def test_output_rows_include_required_fields():
    rows, _, _ = audit_joint_calibration_limits.audit_targets(
        [_target_for_tick(2055)],
        _calibration(),
        _joint_limits(2000, 2100),
    )
    assert set(rows[0].keys()) == set(audit_joint_calibration_limits.CSV_FIELDNAMES)
