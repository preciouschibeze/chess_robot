from __future__ import absolute_import

import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
import tools.audit_square_targets as audit_square_targets
import tools.explain_square_audit as explain_square_audit


JOINT_ORDER = list(robot_square_map.DEFAULT_JOINT_ORDER)


def _joint_limits(minimum=0, maximum=4095):
    limits = {}
    for joint_name in JOINT_ORDER:
        limits[joint_name] = {
            "provisional_min": minimum,
            "provisional_max": maximum,
        }
    return limits


def _manual_pose(base):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = int(base + (index * 10))
    return {
        "source": "manual",
        "confidence": "taught",
        "joints": joints,
        "recorded_at": "2026-05-21T00:00:00Z",
        "notes": ["manual note"],
    }


def _generated_pose(base, anchors):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = int(base + (index * 10))
    return {
        "source": "generated",
        "confidence": "high",
        "joints": joints,
        "generated_at": "2026-05-21T00:00:00Z",
        "interpolation": {
            "method": "joint_space_idw",
            "anchors_used": list(anchors),
        },
        "notes": ["requires manual validation before playback"],
    }


def _fixture_targets():
    def value_at(row, col):
        return 1000 + (row * 200) + (col * 60)

    document = robot_square_map.default_square_targets()
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "h6", "a3", "c3", "f3", "h3", "a1", "c1", "f1", "h1"]
    for square_name in robot_square_map.square_names():
        row, col = robot_square_map.square_to_grid(square_name)
        value = value_at(row, col)
        if square_name in anchors:
            pose = _manual_pose(value)
        else:
            pose = _generated_pose(value, ["a1", "a3", "c1", "c3"])
        document["squares"][square_name] = {"above_pose": pose}
    # Make c8 a sharper manual anchor so it is involved in warnings.
    for joint_name in JOINT_ORDER:
        document["squares"]["c8"]["above_pose"]["joints"][joint_name] += 450
    return document


def _fixture_audit_report(targets, limits):
    return audit_square_targets.audit_square_targets_document(
        targets,
        limits,
        joint_jump_warning=180,
        total_jump_warning=450,
        gradient_ratio_warning=1.50,
        gradient_ratio_error=2.50,
        baseline_percentile=90,
    )


def test_explaining_manual_square():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("c8", targets, _joint_limits(), report, top=10)
    assert explanation["basic_info"]["source"] == "manual"
    assert explanation["manual_context"] is not None
    assert explanation["interpolation"] is None


def test_explaining_generated_square():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    assert explanation["basic_info"]["source"] == "generated"
    assert explanation["interpolation"] is not None


def test_neighbour_delta_computation():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    neighbours = {item["square"]: item for item in explanation["neighbours"]}
    assert "c8" in neighbours
    assert neighbours["c8"]["total_l1_delta"] is not None
    assert neighbours["c8"]["max_joint_delta"] is not None


def test_limit_margin_reporting():
    targets = _fixture_targets()
    limits = _joint_limits(minimum=900, maximum=3000)
    explanation = explain_square_audit.build_square_explanation(
        "b8",
        targets,
        limits,
        _fixture_audit_report(targets, limits),
        top=10,
    )
    nearest = explanation["pose"]["nearest_software_limit"]
    assert nearest is not None
    assert nearest["nearest_margin"] is not None


def test_interpolation_anchors_listed_for_generated_pose():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    anchors = explanation["interpolation"]["anchors"]
    assert anchors
    assert anchors[0].get("grid_distance") is not None


def test_manual_anchor_diagnosis_when_involved_in_gradient_warning():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("c8", targets, _joint_limits(), report, top=10)
    assert explanation["diagnosis"] == "manual anchor suspicious"


def test_fixed_threshold_only_warning_diagnosed_as_lower_priority():
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    # force a fixed-only pattern on b8
    for entry in report["neighbour_delta_records"]:
        if "b8" in (entry.get("square_a"), entry.get("square_b")):
            entry["fixed_threshold_warning"] = True
            entry["gradient_warning"] = False
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    assert explanation["diagnosis"] == "likely fixed-threshold false positive"


def test_json_output_writer(tmpdir):
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    out_path = str(tmpdir.join("explain.json"))
    explain_square_audit.write_json(out_path, explanation)
    with open(out_path, "r") as handle:
        payload = json.load(handle)
    assert payload["square"] == "b8"


def test_markdown_output_writer(tmpdir):
    targets = _fixture_targets()
    report = _fixture_audit_report(targets, _joint_limits())
    explanation = explain_square_audit.build_square_explanation("b8", targets, _joint_limits(), report, top=10)
    out_path = str(tmpdir.join("explain.md"))
    explain_square_audit.write_markdown(out_path, explanation)
    with open(out_path, "r") as handle:
        content = handle.read()
    assert "# Square Audit Explanation" in content
    assert "## Local Diagnosis" in content


def test_invalid_square_rejected():
    with pytest.raises(ValueError):
        explain_square_audit.build_square_explanation("z9", _fixture_targets(), _joint_limits(), {"per_square_records": []}, top=10)
