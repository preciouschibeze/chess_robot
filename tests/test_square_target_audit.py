from __future__ import absolute_import

import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.audit_square_targets as audit_square_targets
from chess_robot.calibration import robot_square_map


JOINT_ORDER = list(robot_square_map.DEFAULT_JOINT_ORDER)


def _joint_limits(minimum=0, maximum=4095):
    result = {}
    for joint_name in JOINT_ORDER:
        result[joint_name] = {
            "provisional_min": minimum,
            "provisional_max": maximum,
        }
    return result


def _manual_pose_from_base(base):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = int(base + (index * 7))
    return {
        "source": "manual",
        "confidence": "taught",
        "joints": joints,
        "recorded_at": "2026-05-21T00:00:00Z",
        "notes": ["manual note"],
    }


def _generated_pose_from_base(base, anchors_used):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = int(base + (index * 7))
    return {
        "source": "generated",
        "confidence": "high",
        "joints": joints,
        "generated_at": "2026-05-21T00:00:00Z",
        "interpolation": {
            "method": "joint_space_idw",
            "anchors_used": list(anchors_used),
        },
        "notes": ["requires manual validation before playback"],
    }


def _full_document_with_function(value_fn, source="manual"):
    document = robot_square_map.default_square_targets()
    for square_name in robot_square_map.square_names():
        row, col = robot_square_map.square_to_grid(square_name)
        value = int(value_fn(row, col))
        if source == "manual":
            pose = _manual_pose_from_base(value)
        else:
            pose = _generated_pose_from_base(value, ["a1", "a3", "c1", "c3"])
        document["squares"][square_name] = {"above_pose": pose}
    return document


def _gradient_fixture_document():
    # High vertical gradient (~400 / square) and lower horizontal gradient (~50 / square).
    def value_fn(row, col):
        return 1000 + (row * 400) + (col * 50)

    document = _full_document_with_function(value_fn, source="generated")
    manual_anchors = ["a1", "a3", "a5", "c1", "c3", "c5"]
    for square_name in manual_anchors:
        row, col = robot_square_map.square_to_grid(square_name)
        value = value_fn(row, col)
        document["squares"][square_name] = {"above_pose": _manual_pose_from_base(value)}
    return document


def test_complete_64_pose_file_produces_valid_summary():
    document = _full_document_with_function(lambda row, col: 1800 + row + col, source="manual")
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    summary = report["summary"]
    assert summary["total_squares_with_above_pose"] == 64
    assert summary["manual_count"] == 64
    assert summary["generated_count"] == 0
    assert summary["missing_count"] == 0
    assert summary["validation_error_count"] == 0


def test_missing_square_is_reported():
    document = _full_document_with_function(lambda row, col: 1800 + row + col, source="manual")
    del document["squares"]["e4"]
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    assert report["summary"]["missing_count"] == 1
    record = [row for row in report["per_square_records"] if row["square"] == "e4"][0]
    assert record["error_count"] >= 1


def test_joint_outside_limit_is_error():
    document = _full_document_with_function(lambda row, col: 1800 + row + col, source="manual")
    document["squares"]["a1"]["above_pose"]["joints"]["shoulder_pan"] = 5000
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits(maximum=3000))
    record = [row for row in report["per_square_records"] if row["square"] == "a1"][0]
    assert record["error_count"] >= 1


def test_low_joint_limit_margin_is_warning():
    document = _full_document_with_function(lambda row, col: 1200 + row + col, source="manual")
    document["squares"]["a1"]["above_pose"]["joints"]["shoulder_pan"] = 1010
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(minimum=1000, maximum=3000),
        limit_margin_warning=20,
    )
    record = [row for row in report["per_square_records"] if row["square"] == "a1"][0]
    assert record["warning_count"] >= 1


def test_manual_anchor_gradients_are_computed_and_separated():
    document = _gradient_fixture_document()
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(),
        baseline_percentile=90,
    )
    baselines = report["gradient_baselines"]
    assert baselines["vertical"]["count"] > 0
    assert baselines["horizontal"]["count"] > 0
    assert baselines["vertical"]["total"]["high"] is not None
    assert baselines["horizontal"]["total"]["high"] is not None


def test_large_raw_jump_normal_relative_to_baseline_is_not_gradient_warning():
    document = _gradient_fixture_document()
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(),
        joint_jump_warning=180,
        total_jump_warning=450,
        gradient_ratio_warning=1.50,
        gradient_ratio_error=2.50,
        baseline_percentile=90,
    )
    match = None
    for record in report["neighbour_delta_records"]:
        pair = sorted([record["square_a"], record["square_b"]])
        if pair == ["a1", "a2"]:
            match = record
            break
    assert match is not None
    assert match["fixed_threshold_warning"] is True
    assert match["gradient_warning"] is False
    assert match["gradient_ratio"] is not None
    assert match["gradient_ratio"] <= 1.5


def test_jump_much_larger_than_baseline_becomes_gradient_warning():
    document = _gradient_fixture_document()
    # Inflate one generated square to force a local outlier jump.
    document["squares"]["b2"]["above_pose"]["joints"]["shoulder_pan"] += 900
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(),
        joint_jump_warning=180,
        total_jump_warning=450,
        gradient_ratio_warning=1.50,
        gradient_ratio_error=2.50,
        baseline_percentile=90,
    )
    warnings = [entry for entry in report["warnings"] if entry.get("code") == "neighbour_gradient_jump"]
    assert warnings


def test_existing_fixed_threshold_warnings_still_exist():
    document = _gradient_fixture_document()
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(),
        joint_jump_warning=180,
        total_jump_warning=450,
    )
    assert report["summary"]["fixed_threshold_neighbour_warning_count"] > 0
    assert any(entry.get("code") == "neighbour_jump" for entry in report["warnings"])


def test_generated_pose_missing_interpolation_metadata_is_warned():
    document = _gradient_fixture_document()
    document["squares"]["b2"]["above_pose"].pop("interpolation")
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    record = [row for row in report["per_square_records"] if row["square"] == "b2"][0]
    assert any(item["code"] == "missing_interpolation" for item in record["warnings"])


def test_generated_pose_anchors_used_not_manual_are_warned():
    document = _gradient_fixture_document()
    document["squares"]["b2"]["above_pose"]["interpolation"]["anchors_used"] = ["d4"]
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    record = [row for row in report["per_square_records"] if row["square"] == "b2"][0]
    assert any(item["code"] == "anchor_not_manual" for item in record["warnings"])


def test_manual_pose_missing_notes_is_warning_only():
    document = _full_document_with_function(lambda row, col: 1800 + row + col, source="manual")
    document["squares"]["c3"]["above_pose"].pop("notes")
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    record = [row for row in report["per_square_records"] if row["square"] == "c3"][0]
    assert record["warning_count"] >= 1
    assert record["error_count"] == 0


def test_json_markdown_csv_include_gradient_fields(tmpdir):
    document = _gradient_fixture_document()
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())

    json_path = str(tmpdir.join("audit.json"))
    csv_path = str(tmpdir.join("audit.csv"))
    md_path = str(tmpdir.join("audit.md"))

    audit_square_targets.write_json_report(json_path, report)
    audit_square_targets.write_csv_report(csv_path, report)
    audit_square_targets.write_markdown_report(md_path, report)

    with open(json_path, "r") as handle:
        payload = json.load(handle)
    assert "gradient_baselines" in payload
    assert "orientation" in payload["neighbour_delta_records"][0]
    assert "fixed_threshold_warning" in payload["neighbour_delta_records"][0]
    assert "gradient_baseline_used" in payload["neighbour_delta_records"][0]
    assert "gradient_ratio" in payload["neighbour_delta_records"][0]
    assert "gradient_warning" in payload["neighbour_delta_records"][0]

    with open(csv_path, "r") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames
        rows = list(reader)
    assert "max_gradient_ratio" in columns
    assert "gradient_warning_count" in columns
    assert len(rows) == 64

    with open(md_path, "r") as handle:
        content = handle.read()
    assert "## Fixed-Threshold Neighbour Warnings" in content
    assert "## Gradient-Aware Neighbour Warnings" in content


def test_no_gradient_aware_disables_new_checks():
    document = _gradient_fixture_document()
    report = audit_square_targets.audit_square_targets_document(
        document,
        _joint_limits(),
        gradient_aware=False,
    )
    assert report["summary"]["gradient_aware_enabled"] is False
    assert report["summary"]["gradient_aware_neighbour_warning_count"] == 0
    for record in report["neighbour_delta_records"]:
        assert record["gradient_warning"] is False


def test_black_side_grid_coordinates_are_used_correctly():
    document = _full_document_with_function(lambda row, col: 1800 + row + col, source="manual")
    report = audit_square_targets.audit_square_targets_document(document, _joint_limits())
    by_square = {}
    for row in report["per_square_records"]:
        by_square[row["square"]] = row
    assert by_square["h1"]["row"] == 0
    assert by_square["h1"]["col"] == 0
    assert by_square["a8"]["row"] == 7
    assert by_square["a8"]["col"] == 7
