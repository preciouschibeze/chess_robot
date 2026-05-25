from __future__ import absolute_import

import os
import sys
import tempfile

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.recalibrate_zero_ticks_from_safety_limits as midpoint_tool
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_limits import convert_joint_safety_limits_to_angle_limits
from chess_robot.robot.joint_limits import load_joint_safety_limits


def _write_yaml(path, data):
    with open(path, "w") as handle:
        try:
            yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)
        except TypeError:
            yaml.safe_dump(data, handle, default_flow_style=False)


def _sample_joint_calibration():
    return {
        "joint_calibration": {
            "ticks_per_rev": 4096,
            "provisional": False,
            "joints": {
                "base_yaw": {
                    "urdf_joint": "shoulder_pan",
                    "direction_sign": 1,
                    "zero_tick": 2100,
                },
                "wrist_roll": {
                    "urdf_joint": "wrist_roll",
                    "direction_sign": -1,
                    "zero_tick": 1234,
                },
            },
        }
    }


def _sample_joint_safety_limits():
    return {
        "safety_limits": {
            "source": "unit_test",
            "calibrated": True,
            "joints": {
                "shoulder_pan": {
                    "min_tick": 1000,
                    "max_tick": 3000,
                    "status": "test",
                },
                "wrist_roll": {
                    "min_tick": 600,
                    "max_tick": 2600,
                    "status": "test",
                },
            },
        }
    }


def _write_fixture_files(directory):
    calibration_path = os.path.join(directory, "joint_calibration.yaml")
    safety_limits_path = os.path.join(directory, "joint_safety_limits.yaml")
    _write_yaml(calibration_path, _sample_joint_calibration())
    _write_yaml(safety_limits_path, _sample_joint_safety_limits())
    return calibration_path, safety_limits_path


def test_midpoint_zero_tick_calculation_handles_even_and_odd_spans():
    assert midpoint_tool.compute_midpoint_zero_tick(100, 200) == 150
    assert midpoint_tool.compute_midpoint_zero_tick(100, 201) == 150
    assert midpoint_tool.compute_midpoint_zero_tick(101, 202) == 152


def test_direction_sign_and_urdf_joint_mapping_are_preserved():
    workdir = tempfile.mkdtemp(prefix="midpoint_zero_preserve_")
    calibration_path, safety_limits_path = _write_fixture_files(workdir)

    updated_document, _ = midpoint_tool.build_midpoint_zero_calibration_document(
        calibration_path,
        safety_limits_path,
    )
    joints = updated_document["joint_calibration"]["joints"]
    assert joints["base_yaw"]["direction_sign"] == 1
    assert joints["wrist_roll"]["direction_sign"] == -1
    assert joints["base_yaw"]["urdf_joint"] == "shoulder_pan"
    assert joints["wrist_roll"]["urdf_joint"] == "wrist_roll"


def test_backup_is_written_and_output_contains_zero_tick_source():
    workdir = tempfile.mkdtemp(prefix="midpoint_zero_backup_")
    calibration_path, safety_limits_path = _write_fixture_files(workdir)
    backup_path = os.path.join(workdir, "joint_calibration.backup.yaml")

    with open(calibration_path, "r") as handle:
        original_text = handle.read()

    midpoint_tool.main(
        [
            "--joint-calibration",
            calibration_path,
            "--joint-safety-limits",
            safety_limits_path,
            "--output",
            calibration_path,
            "--backup",
            backup_path,
        ]
    )

    assert os.path.exists(backup_path)
    with open(backup_path, "r") as handle:
        assert handle.read() == original_text

    with open(calibration_path, "r") as handle:
        output_data = yaml.safe_load(handle)
    joints = output_data["joint_calibration"]["joints"]
    assert joints["base_yaw"]["zero_tick_source"] == midpoint_tool.ZERO_TICK_SOURCE
    assert joints["wrist_roll"]["zero_tick_note"] == midpoint_tool.ZERO_TICK_NOTE


def test_converted_safety_range_becomes_symmetric_around_zero_for_simple_limits():
    workdir = tempfile.mkdtemp(prefix="midpoint_zero_symmetric_")
    calibration_path, safety_limits_path = _write_fixture_files(workdir)
    output_path = os.path.join(workdir, "joint_calibration.updated.yaml")
    backup_path = os.path.join(workdir, "joint_calibration.backup.yaml")

    midpoint_tool.apply_midpoint_zero_calibration(
        joint_calibration_path=calibration_path,
        joint_safety_limits_path=safety_limits_path,
        output_path=output_path,
        backup_path=backup_path,
    )

    calibration = load_joint_calibration(output_path)
    joint_safety_limits = load_joint_safety_limits(safety_limits_path)
    converted = convert_joint_safety_limits_to_angle_limits(joint_safety_limits, calibration)
    shoulder_pan = converted["shoulder_pan"]

    assert abs(shoulder_pan["lower_deg"] + shoulder_pan["upper_deg"]) < 1.0e-9
