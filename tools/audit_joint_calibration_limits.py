#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import csv
import json
import math
import os
import sys
from collections import OrderedDict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.joint_calibration import angle_rad_to_tick
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import tick_to_angle_rad

DEFAULT_IK_PATH = os.path.join(ROOT, "data", "debug", "so101_square_ik_urdf_limits.json")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_OUTPUT_PREFIX = os.path.join(ROOT, "data", "debug", "so101_joint_limit_audit")

FOCUS_TARGET_NAMES = [
    "e8_above",
    "d8_above",
    "f8_above",
    "c8_above",
    "e7_above",
    "d7_above",
    "f7_above",
    "c7_above",
    "g8_above",
    "b8_above",
]

CSV_FIELDNAMES = [
    "target_name",
    "target_type",
    "square",
    "success",
    "error_mm",
    "joint_name",
    "urdf_joint",
    "angle_rad",
    "angle_deg",
    "converted_tick",
    "software_min_tick",
    "software_max_tick",
    "violation_type",
    "violation_ticks",
    "violation_deg",
    "margin_to_min_tick",
    "margin_to_max_tick",
]


def build_parser():
    parser = argparse.ArgumentParser(
        description="Audit URDF-limit IK solutions against software tick limits."
    )
    parser.add_argument(
        "--ik-json",
        dest="ik_path",
        default=DEFAULT_IK_PATH,
        help="Path to analyse_square_ik JSON or CSV output generated with --limit-source urdf.",
    )
    parser.add_argument(
        "--joint-calibration",
        default=DEFAULT_JOINT_CALIBRATION_PATH,
        help="Joint calibration YAML path.",
    )
    parser.add_argument(
        "--joint-limits",
        default=DEFAULT_JOINT_LIMITS_PATH,
        help="Joint limits YAML path.",
    )
    parser.add_argument(
        "--output-prefix",
        default=DEFAULT_OUTPUT_PREFIX,
        help="Output prefix used for CSV, JSON, and optional summary text outputs.",
    )
    return parser


def load_ik_targets(path):
    extension = os.path.splitext(path)[1].lower()
    if extension == ".csv":
        return _load_ik_targets_csv(path)
    return _load_ik_targets_json(path)


def _load_ik_targets_json(path):
    with open(path, "r") as handle:
        document = json.load(handle)
    if isinstance(document, list):
        return document
    if isinstance(document, dict) and isinstance(document.get("targets"), list):
        return document["targets"]
    raise ValueError("IK JSON input must be a list or contain a 'targets' list.")


def _load_ik_targets_csv(path):
    with open(path, "r") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def build_joint_specs(calibration, joint_limits):
    specs = []
    for user_joint in calibration.get("joint_order") or []:
        calibration_entry = calibration["joints"][user_joint]
        urdf_joint = calibration_entry["urdf_joint"]
        limit_entry = joint_limits.get(user_joint)
        if limit_entry is None:
            limit_entry = joint_limits.get(urdf_joint)
        if not isinstance(limit_entry, dict):
            raise ValueError("Missing software limit entry for %s (%s)." % (user_joint, urdf_joint))

        software_min_tick = _coerce_int(limit_entry.get("provisional_min"))
        software_max_tick = _coerce_int(limit_entry.get("provisional_max"))
        if software_min_tick is None or software_max_tick is None:
            raise ValueError(
                "Software limit entry for %s (%s) must contain provisional_min and provisional_max."
                % (user_joint, urdf_joint)
            )

        specs.append(
            {
                "joint_name": user_joint,
                "urdf_joint": urdf_joint,
                "software_min_tick": software_min_tick,
                "software_max_tick": software_max_tick,
            }
        )
    return specs


def audit_targets(targets, calibration, joint_limits):
    joint_specs = build_joint_specs(calibration, joint_limits)
    audit_rows = []
    target_records = []

    for index, target in enumerate(targets):
        target_name = _text_value(target.get("target_name"))
        if not target_name:
            target_name = "target_%03d" % index
        target_type = _text_value(target.get("target_type"))
        square = _text_value(target.get("square"))
        success = _coerce_bool(target.get("success"))
        error_mm = _coerce_float(target.get("error_mm"))

        joint_rows = []
        for joint_spec in joint_specs:
            angle_rad = _extract_angle_rad(target, joint_spec)
            if angle_rad is None:
                continue
            joint_row = _build_joint_row(
                target_name=target_name,
                target_type=target_type,
                square=square,
                success=success,
                error_mm=error_mm,
                joint_spec=joint_spec,
                angle_rad=angle_rad,
                calibration=calibration,
            )
            joint_rows.append(joint_row)
            audit_rows.append(joint_row)

        if success and len(joint_rows) != len(joint_specs):
            raise ValueError(
                "Target %s is marked successful but is missing one or more joint angles."
                % target_name
            )

        if joint_rows:
            violations = [row for row in joint_rows if row["violation_type"] != "none"]
            target_records.append(
                {
                    "target_name": target_name,
                    "target_type": target_type,
                    "square": square,
                    "success": success,
                    "error_mm": error_mm,
                    "joint_count": len(joint_rows),
                    "violation_count": len(violations),
                    "violating_joints": [row["urdf_joint"] for row in violations],
                    "max_violation_ticks": max([row["violation_ticks"] for row in violations] or [0]),
                }
            )

    return audit_rows, target_records, joint_specs


def build_audit_report(ik_path, calibration_path, joint_limits_path):
    calibration = load_joint_calibration(calibration_path)
    joint_limits = load_joint_limits(joint_limits_path)
    targets = load_ik_targets(ik_path)
    audit_rows, target_records, joint_specs = audit_targets(targets, calibration, joint_limits)

    violation_rows = [row for row in audit_rows if row["violation_type"] != "none"]
    violating_target_records = [record for record in target_records if record["violation_count"] > 0]

    violations_per_joint = OrderedDict()
    for joint_spec in joint_specs:
        violations_per_joint[joint_spec["urdf_joint"]] = 0
    for row in violation_rows:
        violations_per_joint[row["urdf_joint"]] = violations_per_joint.get(row["urdf_joint"], 0) + 1

    worst_violations = sorted(
        violation_rows,
        key=lambda row: (-int(row["violation_ticks"]), row["target_name"], row["urdf_joint"]),
    )[:20]

    focus_target_violations = OrderedDict()
    for target_name in FOCUS_TARGET_NAMES:
        focus_target_violations[target_name] = [
            row for row in violation_rows if row["target_name"] == target_name
        ]

    report = OrderedDict()
    report["inputs"] = OrderedDict(
        [
            ("ik_path", ik_path),
            ("joint_calibration_path", calibration_path),
            ("joint_limits_path", joint_limits_path),
        ]
    )
    report["summary"] = OrderedDict(
        [
            ("ik_targets_audited", len(target_records)),
            ("targets_with_violations", len(violating_target_records)),
            ("total_joint_rows", len(audit_rows)),
            ("total_joint_violations", len(violation_rows)),
            ("violations_per_joint", violations_per_joint),
        ]
    )
    report["joint_specs"] = joint_specs
    report["violating_targets"] = violating_target_records
    report["worst_violations"] = worst_violations
    report["focus_target_violations"] = focus_target_violations
    report["audit_rows"] = audit_rows
    return report


def write_csv_report(path, rows):
    _ensure_parent_dir(path)
    with open(path, "w") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json_report(path, report):
    _ensure_parent_dir(path)
    with open(path, "w") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")


def write_summary_text(path, summary_text):
    _ensure_parent_dir(path)
    with open(path, "w") as handle:
        handle.write(summary_text)
        if not summary_text.endswith("\n"):
            handle.write("\n")


def build_summary_text(report):
    lines = []
    summary = report["summary"]
    lines.append("IK targets audited: %d" % summary["ik_targets_audited"])
    lines.append("Targets with software-limit violations: %d" % summary["targets_with_violations"])
    lines.append("Violations per joint:")
    for joint_name, count in summary["violations_per_joint"].items():
        lines.append("  %s: %d" % (joint_name, count))

    lines.append("Worst 20 violations by tick amount:")
    if report["worst_violations"]:
        for row in report["worst_violations"]:
            lines.append(
                "  %s %s: tick=%d limits=[%d, %d] %s by %d ticks (%.2f deg)"
                % (
                    row["target_name"],
                    row["urdf_joint"],
                    row["converted_tick"],
                    row["software_min_tick"],
                    row["software_max_tick"],
                    row["violation_type"],
                    row["violation_ticks"],
                    row["violation_deg"],
                )
            )
    else:
        lines.append("  none")

    lines.append("Focus target violations:")
    for target_name in FOCUS_TARGET_NAMES:
        rows = report["focus_target_violations"].get(target_name) or []
        if not rows:
            lines.append("  %s: none" % target_name)
            continue
        for row in rows:
            lines.append(
                "  %s %s: tick=%d limits=[%d, %d] %s by %d ticks (%.2f deg)"
                % (
                    target_name,
                    row["urdf_joint"],
                    row["converted_tick"],
                    row["software_min_tick"],
                    row["software_max_tick"],
                    row["violation_type"],
                    row["violation_ticks"],
                    row["violation_deg"],
                )
            )
    return "\n".join(lines)


def _build_joint_row(target_name, target_type, square, success, error_mm, joint_spec, angle_rad, calibration):
    joint_name = joint_spec["joint_name"]
    urdf_joint = joint_spec["urdf_joint"]
    converted_tick = int(angle_rad_to_tick(joint_name, angle_rad, calibration))
    software_min_tick = int(joint_spec["software_min_tick"])
    software_max_tick = int(joint_spec["software_max_tick"])
    margin_to_min_tick = int(converted_tick - software_min_tick)
    margin_to_max_tick = int(software_max_tick - converted_tick)

    if converted_tick < software_min_tick:
        violation_type = "below_min"
        violation_ticks = int(software_min_tick - converted_tick)
        limit_angle_rad = tick_to_angle_rad(joint_name, software_min_tick, calibration)
    elif converted_tick > software_max_tick:
        violation_type = "above_max"
        violation_ticks = int(converted_tick - software_max_tick)
        limit_angle_rad = tick_to_angle_rad(joint_name, software_max_tick, calibration)
    else:
        violation_type = "none"
        violation_ticks = 0
        limit_angle_rad = angle_rad

    violation_deg = abs(math.degrees(float(angle_rad) - float(limit_angle_rad)))

    return OrderedDict(
        [
            ("target_name", target_name),
            ("target_type", target_type),
            ("square", square),
            ("success", success),
            ("error_mm", error_mm),
            ("joint_name", joint_name),
            ("urdf_joint", urdf_joint),
            ("angle_rad", float(angle_rad)),
            ("angle_deg", float(math.degrees(angle_rad))),
            ("converted_tick", converted_tick),
            ("software_min_tick", software_min_tick),
            ("software_max_tick", software_max_tick),
            ("violation_type", violation_type),
            ("violation_ticks", violation_ticks),
            ("violation_deg", float(violation_deg)),
            ("margin_to_min_tick", margin_to_min_tick),
            ("margin_to_max_tick", margin_to_max_tick),
        ]
    )


def _extract_angle_rad(target, joint_spec):
    user_joint = joint_spec["joint_name"]
    urdf_joint = joint_spec["urdf_joint"]

    for key in ["%s_rad" % urdf_joint, "%s_rad" % user_joint]:
        value = _coerce_float(target.get(key))
        if value is not None:
            return value

    joint_positions_rad = target.get("joint_positions_rad")
    if isinstance(joint_positions_rad, dict):
        for key in [urdf_joint, user_joint]:
            value = _coerce_float(joint_positions_rad.get(key))
            if value is not None:
                return value

    for key in ["%s_deg" % urdf_joint, "%s_deg" % user_joint]:
        value = _coerce_float(target.get(key))
        if value is not None:
            return math.radians(value)

    joint_positions_deg = target.get("joint_positions_deg")
    if isinstance(joint_positions_deg, dict):
        for key in [urdf_joint, user_joint]:
            value = _coerce_float(joint_positions_deg.get(key))
            if value is not None:
                return math.radians(value)

    return None


def _coerce_bool(value):
    if isinstance(value, bool):
        return value
    text = _text_value(value).lower()
    return text in ("1", "true", "yes", "y")


def _coerce_float(value):
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    return float(text)


def _coerce_int(value):
    coerced = _coerce_float(value)
    if coerced is None:
        return None
    return int(round(coerced))


def _text_value(value):
    if value is None:
        return ""
    return str(value)


def _ensure_parent_dir(path):
    parent = os.path.dirname(path)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def main():
    args = build_parser().parse_args()
    report = build_audit_report(args.ik_path, args.joint_calibration, args.joint_limits)

    csv_path = args.output_prefix + ".csv"
    json_path = args.output_prefix + ".json"
    summary_path = args.output_prefix + "_summary.txt"

    write_csv_report(csv_path, report["audit_rows"])
    write_json_report(json_path, report)
    summary_text = build_summary_text(report)
    write_summary_text(summary_path, summary_text)

    print(summary_text)
    print("CSV report written to %s" % csv_path)
    print("JSON report written to %s" % json_path)
    print("Summary report written to %s" % summary_path)


if __name__ == "__main__":
    main()
