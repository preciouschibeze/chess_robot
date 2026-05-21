#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import csv
import json
import os
import sys
from collections import OrderedDict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map

DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_LIMIT_MARGIN_WARNING = 20
DEFAULT_JOINT_JUMP_WARNING = 180
DEFAULT_TOTAL_JUMP_WARNING = 450
DEFAULT_GENERATED_ANCHOR_DISTANCE_WARNING = 4
DEFAULT_GRADIENT_RATIO_WARNING = 1.50
DEFAULT_GRADIENT_RATIO_ERROR = 2.50
DEFAULT_BASELINE_PERCENTILE = 90

HIGH_WEIGHT = 100
MEDIUM_WEIGHT = 30
LOW_WEIGHT = 10
GENERATED_BASE_WEIGHT = 5
FIXED_NORMAL_WEIGHT = 8


def build_parser():
    parser = argparse.ArgumentParser(description="Audit square-target calibration data without touching hardware.")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH,
                        help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--output-json", default=None,
                        help="Optional JSON report output path.")
    parser.add_argument("--output-csv", default=None,
                        help="Optional CSV report output path.")
    parser.add_argument("--output-md", default=None,
                        help="Optional Markdown report output path.")
    parser.add_argument("--strict", action="store_true",
                        help="Exit non-zero when validation errors are present.")
    parser.add_argument("--fail-on-warnings", action="store_true",
                        help="Exit non-zero when warnings are present.")
    parser.add_argument("--limit-margin-warning", type=int, default=DEFAULT_LIMIT_MARGIN_WARNING,
                        help="Warn when nearest joint-limit margin is below this threshold.")
    parser.add_argument("--joint-jump-warning", type=int, default=DEFAULT_JOINT_JUMP_WARNING,
                        help="Warn when any per-joint neighbour delta exceeds this threshold.")
    parser.add_argument("--total-jump-warning", type=int, default=DEFAULT_TOTAL_JUMP_WARNING,
                        help="Warn when total L1 neighbour delta exceeds this threshold.")
    parser.set_defaults(gradient_aware=True)
    parser.add_argument("--gradient-aware", dest="gradient_aware", action="store_true",
                        help="Enable gradient-aware neighbour checks (default).")
    parser.add_argument("--no-gradient-aware", dest="gradient_aware", action="store_false",
                        help="Disable gradient-aware neighbour checks.")
    parser.add_argument("--gradient-ratio-warning", type=float, default=DEFAULT_GRADIENT_RATIO_WARNING,
                        help="Warn when neighbour jump exceeds this ratio vs local baseline.")
    parser.add_argument("--gradient-ratio-error", type=float, default=DEFAULT_GRADIENT_RATIO_ERROR,
                        help="Escalate when neighbour jump exceeds this ratio vs local baseline.")
    parser.add_argument("--baseline-percentile", type=int, default=DEFAULT_BASELINE_PERCENTILE,
                        help="Percentile used for manual-gradient baseline high values.")
    return parser


def _limit_bounds(limit_entry):
    if not isinstance(limit_entry, dict):
        return None
    lower = limit_entry.get("provisional_min")
    upper = limit_entry.get("provisional_max")
    if isinstance(lower, bool) or not isinstance(lower, int):
        return None
    if isinstance(upper, bool) or not isinstance(upper, int):
        return None
    return lower, upper


def _notes_contain_manual_validation(notes):
    if not isinstance(notes, list):
        return False
    for entry in notes:
        if isinstance(entry, str) and "manual validation" in entry.lower():
            return True
    return False


def _as_joint_map(joints, joint_order):
    if not isinstance(joints, dict):
        return None
    result = {}
    for joint_name in joint_order:
        value = joints.get(joint_name)
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        result[joint_name] = value
    return result


def _neighbours(row, col):
    candidates = [
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1),
    ]
    valid = []
    for next_row, next_col in candidates:
        if 0 <= next_row < robot_square_map.BOARD_SIZE and 0 <= next_col < robot_square_map.BOARD_SIZE:
            valid.append((next_row, next_col))
    return valid


def _nearest_manual_anchor(square_name, manual_anchors):
    if not manual_anchors:
        return None, None
    row, col = robot_square_map.square_to_grid(square_name)
    best = None
    best_dist = None
    for anchor_name in sorted(manual_anchors):
        anchor_row, anchor_col = robot_square_map.square_to_grid(anchor_name)
        distance = abs(row - anchor_row) + abs(col - anchor_col)
        if best_dist is None or distance < best_dist or (distance == best_dist and anchor_name < best):
            best = anchor_name
            best_dist = distance
    return best, best_dist


def _ensure_parent_dir(path):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)


def _add_issue(issues, summary_list, square_name, severity, code, message, weight):
    entry = {
        "square": square_name,
        "severity": severity,
        "code": code,
        "message": message,
        "weight": int(weight),
    }
    issues.append(entry)
    summary_list.append(entry)


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    if percentile <= 0:
        return float(ordered[0])
    if percentile >= 100:
        return float(ordered[-1])
    rank = (float(percentile) / 100.0) * float(len(ordered) - 1)
    lower_index = int(rank)
    upper_index = lower_index + 1
    if upper_index >= len(ordered):
        return float(ordered[lower_index])
    fraction = rank - float(lower_index)
    lower_value = float(ordered[lower_index])
    upper_value = float(ordered[upper_index])
    return lower_value + ((upper_value - lower_value) * fraction)


def _median(values):
    return _percentile(values, 50)


def _empty_orientation_stats(joint_order):
    per_joint = {}
    for joint_name in joint_order:
        per_joint[joint_name] = {
            "count": 0,
            "median": None,
            "high": None,
        }
    return {
        "count": 0,
        "per_joint": per_joint,
        "total": {
            "count": 0,
            "median": None,
            "high": None,
        },
    }


def build_manual_gradient_baselines(manual_joint_maps, joint_order, baseline_percentile):
    observations = {
        "vertical": {
            "per_joint": {},
            "total": [],
            "count": 0,
        },
        "horizontal": {
            "per_joint": {},
            "total": [],
            "count": 0,
        },
    }
    for orientation in ("vertical", "horizontal"):
        for joint_name in joint_order:
            observations[orientation]["per_joint"][joint_name] = []

    manual_items = []
    for square_name in sorted(manual_joint_maps.keys()):
        row, col = robot_square_map.square_to_grid(square_name)
        manual_items.append((square_name, row, col, manual_joint_maps[square_name]))

    item_count = len(manual_items)
    for i in range(item_count):
        square_a, row_a, col_a, joints_a = manual_items[i]
        for j in range(i + 1, item_count):
            square_b, row_b, col_b, joints_b = manual_items[j]
            orientation = None
            distance = None
            if col_a == col_b and row_a != row_b:
                orientation = "vertical"
                distance = abs(row_a - row_b)
            elif row_a == row_b and col_a != col_b:
                orientation = "horizontal"
                distance = abs(col_a - col_b)
            if orientation is None or distance is None or distance <= 0:
                continue

            total_delta = 0.0
            for joint_name in joint_order:
                delta = abs(joints_a[joint_name] - joints_b[joint_name])
                total_delta += float(delta)
                gradient = float(delta) / float(distance)
                observations[orientation]["per_joint"][joint_name].append(gradient)
            observations[orientation]["total"].append(total_delta / float(distance))
            observations[orientation]["count"] += 1

    baselines = {
        "baseline_percentile": int(baseline_percentile),
        "vertical": _empty_orientation_stats(joint_order),
        "horizontal": _empty_orientation_stats(joint_order),
    }
    for orientation in ("vertical", "horizontal"):
        baselines[orientation]["count"] = observations[orientation]["count"]
        totals = observations[orientation]["total"]
        baselines[orientation]["total"]["count"] = len(totals)
        baselines[orientation]["total"]["median"] = _median(totals)
        baselines[orientation]["total"]["high"] = _percentile(totals, baseline_percentile)
        for joint_name in joint_order:
            values = observations[orientation]["per_joint"][joint_name]
            baselines[orientation]["per_joint"][joint_name]["count"] = len(values)
            baselines[orientation]["per_joint"][joint_name]["median"] = _median(values)
            baselines[orientation]["per_joint"][joint_name]["high"] = _percentile(values, baseline_percentile)

    baselines["enabled"] = bool(
        baselines["vertical"]["count"] > 0 or baselines["horizontal"]["count"] > 0
    )
    return baselines


def _compute_gradient_ratio(joint_deltas, total_delta, joint_order, orientation_baseline):
    if not orientation_baseline:
        return None
    ratios = []
    baseline_used = {
        "total_high": None,
        "per_joint_high": {},
        "orientation_count": int(orientation_baseline.get("count") or 0),
    }

    total_high = orientation_baseline.get("total", {}).get("high")
    baseline_used["total_high"] = total_high
    if total_high is not None and total_high > 0:
        ratios.append(float(total_delta) / float(total_high))

    per_joint = orientation_baseline.get("per_joint", {})
    for joint_name in joint_order:
        high_value = None
        if isinstance(per_joint.get(joint_name), dict):
            high_value = per_joint[joint_name].get("high")
        baseline_used["per_joint_high"][joint_name] = high_value
        if high_value is None or high_value <= 0:
            continue
        delta = joint_deltas.get(joint_name)
        if delta is None:
            continue
        ratios.append(float(delta) / float(high_value))

    if not ratios:
        return None, baseline_used
    return max(ratios), baseline_used


def audit_square_targets_document(document, joint_limits,
                                 limit_margin_warning=DEFAULT_LIMIT_MARGIN_WARNING,
                                 joint_jump_warning=DEFAULT_JOINT_JUMP_WARNING,
                                 total_jump_warning=DEFAULT_TOTAL_JUMP_WARNING,
                                 gradient_aware=True,
                                 gradient_ratio_warning=DEFAULT_GRADIENT_RATIO_WARNING,
                                 gradient_ratio_error=DEFAULT_GRADIENT_RATIO_ERROR,
                                 baseline_percentile=DEFAULT_BASELINE_PERCENTILE):
    doc = robot_square_map.normalise_square_targets_document(document)
    joint_order = list(doc.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)

    per_square = OrderedDict()
    for square_name in robot_square_map.square_names():
        row, col = robot_square_map.square_to_grid(square_name)
        per_square[square_name] = {
            "square": square_name,
            "row": row,
            "col": col,
            "source": None,
            "has_pose": False,
            "valid": False,
            "error_count": 0,
            "warning_count": 0,
            "errors": [],
            "warnings": [],
            "nearest_manual_anchor": None,
            "nearest_manual_distance": None,
            "min_limit_margin": None,
            "max_neighbour_joint_delta": 0,
            "total_neighbour_delta": 0,
            "max_gradient_ratio": None,
            "gradient_warning_count": 0,
            "suspicion_score": 0,
        }

    all_errors = []
    all_warnings = []

    def add_square_error(square_name, code, message, weight=HIGH_WEIGHT):
        record = per_square.get(square_name)
        if record is None:
            _add_issue(all_errors, all_errors, square_name, "high", code, message, weight)
            return
        _add_issue(record["errors"], all_errors, square_name, "high", code, message, weight)
        record["error_count"] += 1
        record["suspicion_score"] += int(weight)

    def add_square_warning(square_name, code, message, weight=LOW_WEIGHT, severity="warning"):
        record = per_square.get(square_name)
        if record is None:
            _add_issue(all_warnings, all_warnings, square_name, severity, code, message, weight)
            return
        _add_issue(record["warnings"], all_warnings, square_name, severity, code, message, weight)
        record["warning_count"] += 1
        record["suspicion_score"] += int(weight)

    if doc.get("board_orientation") != robot_square_map.BOARD_ORIENTATION:
        _add_issue(all_errors, all_errors, None, "high", "board_orientation",
                   "board_orientation must be {}".format(robot_square_map.BOARD_ORIENTATION), HIGH_WEIGHT)
    if doc.get("pose_type") != robot_square_map.POSE_TYPE:
        _add_issue(all_errors, all_errors, None, "high", "pose_type",
                   "pose_type must be {}".format(robot_square_map.POSE_TYPE), HIGH_WEIGHT)

    source_counts = {
        "manual": 0,
        "generated": 0,
        "other": 0,
        "missing": 0,
    }

    manual_anchor_squares = set(robot_square_map.collect_manual_anchor_squares(doc, joint_order))
    missing_recommended = robot_square_map.missing_recommended_anchors(doc)

    valid_joint_maps = {}
    manual_joint_maps = {}

    for square_name in robot_square_map.square_names():
        record = per_square[square_name]
        square_info = doc.get("squares", {}).get(square_name)
        if not isinstance(square_info, dict):
            source_counts["missing"] += 1
            add_square_error(square_name, "missing_pose", "missing square entry or above_pose")
            continue

        above_pose = square_info.get("above_pose")
        if not isinstance(above_pose, dict):
            source_counts["missing"] += 1
            add_square_error(square_name, "missing_pose", "missing above_pose")
            continue

        record["has_pose"] = True
        source = above_pose.get("source")
        if source == "manual":
            source_counts["manual"] += 1
        elif source == "generated":
            source_counts["generated"] += 1
            record["suspicion_score"] += GENERATED_BASE_WEIGHT
        else:
            source_counts["other"] += 1
            add_square_warning(square_name, "unknown_source", "unknown source {}".format(source), LOW_WEIGHT)

        record["source"] = source

        joints = above_pose.get("joints")
        issues = robot_square_map.validate_pose_joints(joints, joint_limits, joint_order)
        for issue in issues:
            add_square_error(square_name, "joint_validation", issue, HIGH_WEIGHT)

        joints_map = _as_joint_map(joints, joint_order)
        if joints_map is not None:
            valid_joint_maps[square_name] = joints_map
            if source == "manual":
                manual_joint_maps[square_name] = joints_map
            nearest_limit_margin = None
            for joint_name in joint_order:
                value = joints_map[joint_name]
                bounds = _limit_bounds(joint_limits.get(joint_name))
                if bounds is None:
                    continue
                lower, upper = bounds
                distance_min = value - lower
                distance_max = upper - value
                margin = distance_min if distance_min < distance_max else distance_max
                if nearest_limit_margin is None or margin < nearest_limit_margin:
                    nearest_limit_margin = margin
                if margin < limit_margin_warning:
                    add_square_warning(
                        square_name,
                        "low_limit_margin",
                        "{} margin {} below warning threshold {}".format(joint_name, margin, limit_margin_warning),
                        MEDIUM_WEIGHT,
                    )
            record["min_limit_margin"] = nearest_limit_margin

        notes = above_pose.get("notes")
        if source == "generated":
            interpolation = above_pose.get("interpolation")
            if not isinstance(interpolation, dict):
                add_square_warning(square_name, "missing_interpolation", "generated pose missing interpolation metadata", LOW_WEIGHT)
            else:
                method = interpolation.get("method")
                anchors_used = interpolation.get("anchors_used")
                if not method:
                    add_square_warning(square_name, "missing_interpolation_method", "generated pose missing interpolation.method", LOW_WEIGHT)
                if not isinstance(anchors_used, list) or not anchors_used:
                    add_square_warning(square_name, "missing_anchors_used", "generated pose missing interpolation.anchors_used", LOW_WEIGHT)
                else:
                    for anchor_name in anchors_used:
                        try:
                            normalized_anchor = robot_square_map.normalise_square_name(anchor_name)
                        except ValueError:
                            add_square_warning(square_name, "invalid_anchor_name",
                                               "generated anchors_used contains invalid square {}".format(anchor_name), LOW_WEIGHT)
                            continue
                        if normalized_anchor not in manual_anchor_squares:
                            add_square_warning(square_name, "anchor_not_manual",
                                               "generated anchors_used references non-manual anchor {}".format(normalized_anchor), LOW_WEIGHT)
            if not _notes_contain_manual_validation(notes):
                add_square_warning(square_name, "missing_validation_note",
                                   "generated pose notes should include manual validation warning", LOW_WEIGHT)

            nearest_anchor, nearest_distance = _nearest_manual_anchor(square_name, manual_anchor_squares)
            record["nearest_manual_anchor"] = nearest_anchor
            record["nearest_manual_distance"] = nearest_distance
            if nearest_distance is not None and nearest_distance >= DEFAULT_GENERATED_ANCHOR_DISTANCE_WARNING:
                add_square_warning(
                    square_name,
                    "far_from_manual_anchor",
                    "generated pose is {} squares from nearest manual anchor {}".format(nearest_distance, nearest_anchor),
                    MEDIUM_WEIGHT,
                )
        elif source == "manual":
            if above_pose.get("confidence") != "taught":
                add_square_warning(square_name, "manual_confidence",
                                   "manual pose confidence should be taught", LOW_WEIGHT)
            if not above_pose.get("recorded_at"):
                add_square_warning(square_name, "missing_recorded_at",
                                   "manual pose missing recorded_at", LOW_WEIGHT)
            if not isinstance(notes, list):
                add_square_warning(square_name, "missing_notes",
                                   "manual pose notes should be a list", LOW_WEIGHT)

    baseline_percentile = max(0, min(100, int(baseline_percentile)))
    gradient_baselines = build_manual_gradient_baselines(manual_joint_maps, joint_order, baseline_percentile)
    gradient_enabled = bool(gradient_aware and gradient_baselines.get("enabled"))

    fixed_threshold_warning_edges = 0
    gradient_warning_edges = 0

    seen_pairs = set()
    neighbour_delta_records = []
    for square_name in robot_square_map.square_names():
        joints_a = valid_joint_maps.get(square_name)
        if joints_a is None:
            continue
        row, col = robot_square_map.square_to_grid(square_name)
        for neighbour_row, neighbour_col in _neighbours(row, col):
            neighbour_name = robot_square_map.grid_to_square(neighbour_row, neighbour_col)
            joints_b = valid_joint_maps.get(neighbour_name)
            if joints_b is None:
                continue
            pair = tuple(sorted([square_name, neighbour_name]))
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)

            row_a, col_a = robot_square_map.square_to_grid(pair[0])
            row_b, col_b = robot_square_map.square_to_grid(pair[1])
            if col_a == col_b:
                orientation = "vertical"
            else:
                orientation = "horizontal"

            joint_deltas = {}
            max_joint_delta = 0
            total_delta = 0
            for joint_name in joint_order:
                delta = abs(joints_a[joint_name] - joints_b[joint_name])
                joint_deltas[joint_name] = delta
                if delta > max_joint_delta:
                    max_joint_delta = delta
                total_delta += delta

            fixed_warning = (max_joint_delta > joint_jump_warning) or (total_delta > total_jump_warning)
            if fixed_warning:
                fixed_threshold_warning_edges += 1

            orientation_baseline = gradient_baselines.get(orientation, {})
            gradient_ratio = None
            baseline_used = {
                "enabled": gradient_enabled,
                "orientation": orientation,
                "baseline_percentile": baseline_percentile,
                "total_high": None,
                "per_joint_high": {},
                "orientation_count": int(orientation_baseline.get("count") or 0),
            }
            gradient_warning = False
            gradient_severity = "warning"
            ratio_result = None
            if gradient_enabled:
                ratio_result, computed_baseline_used = _compute_gradient_ratio(
                    joint_deltas,
                    total_delta,
                    joint_order,
                    orientation_baseline,
                )
                if computed_baseline_used is not None:
                    baseline_used.update(computed_baseline_used)
                gradient_ratio = ratio_result
                if gradient_ratio is not None and gradient_ratio > gradient_ratio_warning:
                    gradient_warning = True
                    if gradient_ratio > gradient_ratio_error:
                        gradient_severity = "high"
                        gradient_warning_edges += 1
                    else:
                        gradient_warning_edges += 1

            neighbour_delta_records.append({
                "square_a": pair[0],
                "square_b": pair[1],
                "orientation": orientation,
                "joint_deltas": joint_deltas,
                "max_joint_delta": max_joint_delta,
                "total_delta": total_delta,
                "fixed_threshold_warning": fixed_warning,
                "gradient_baseline_used": baseline_used,
                "gradient_ratio": gradient_ratio,
                "gradient_warning": gradient_warning,
                "warning": fixed_warning,
            })

            for member in pair:
                member_record = per_square[member]
                if max_joint_delta > member_record["max_neighbour_joint_delta"]:
                    member_record["max_neighbour_joint_delta"] = max_joint_delta
                if total_delta > member_record["total_neighbour_delta"]:
                    member_record["total_neighbour_delta"] = total_delta
                if gradient_ratio is not None:
                    current_ratio = member_record.get("max_gradient_ratio")
                    if current_ratio is None or gradient_ratio > current_ratio:
                        member_record["max_gradient_ratio"] = gradient_ratio
                if gradient_warning:
                    member_record["gradient_warning_count"] += 1

            if fixed_warning:
                fixed_weight = MEDIUM_WEIGHT
                if gradient_enabled and (not gradient_warning):
                    fixed_weight = FIXED_NORMAL_WEIGHT
                warning_message = "neighbour jump {}<->{} max_joint_delta={} total_delta={}".format(
                    pair[0], pair[1], max_joint_delta, total_delta
                )
                add_square_warning(pair[0], "neighbour_jump", warning_message, fixed_weight)
                add_square_warning(pair[1], "neighbour_jump", warning_message, fixed_weight)

            if gradient_warning:
                gradient_weight = MEDIUM_WEIGHT
                if gradient_ratio is not None and gradient_ratio > gradient_ratio_error:
                    gradient_weight = HIGH_WEIGHT
                gradient_message = "gradient-aware neighbour jump {}<->{} ratio={:.3f} orientation={}".format(
                    pair[0], pair[1], gradient_ratio, orientation
                )
                add_square_warning(
                    pair[0],
                    "neighbour_gradient_jump",
                    gradient_message,
                    gradient_weight,
                    severity=gradient_severity,
                )
                add_square_warning(
                    pair[1],
                    "neighbour_gradient_jump",
                    gradient_message,
                    gradient_weight,
                    severity=gradient_severity,
                )

    for square_name in robot_square_map.square_names():
        record = per_square[square_name]
        record["valid"] = record["has_pose"] and record["error_count"] == 0

    recommended_anchor_present_count = 0
    for square_name in robot_square_map.RECOMMENDED_ANCHORS:
        if square_name in manual_anchor_squares:
            recommended_anchor_present_count += 1

    total_with_pose = 64 - source_counts["missing"]
    summary = {
        "board_orientation": doc.get("board_orientation"),
        "pose_type": doc.get("pose_type"),
        "total_expected_squares": 64,
        "total_squares_with_above_pose": total_with_pose,
        "manual_count": source_counts["manual"],
        "generated_count": source_counts["generated"],
        "other_source_count": source_counts["other"],
        "missing_count": source_counts["missing"],
        "manual_anchor_count": len(manual_anchor_squares),
        "recommended_anchor_count": recommended_anchor_present_count,
        "recommended_anchor_target_count": len(robot_square_map.RECOMMENDED_ANCHORS),
        "missing_recommended_anchor_count": len(missing_recommended),
        "missing_recommended_anchors": list(missing_recommended),
        "has_all_recommended_anchors": recommended_anchor_present_count == len(robot_square_map.RECOMMENDED_ANCHORS),
        "has_minimum_manual_anchors": len(manual_anchor_squares) >= robot_square_map.MIN_MANUAL_ANCHORS_FOR_WRITE,
        "validation_error_count": len(all_errors),
        "warning_count": len(all_warnings),
        "manual_vertical_baseline_count": int(gradient_baselines["vertical"].get("count") or 0),
        "manual_horizontal_baseline_count": int(gradient_baselines["horizontal"].get("count") or 0),
        "baseline_percentile": baseline_percentile,
        "gradient_aware_enabled": gradient_enabled,
        "fixed_threshold_neighbour_warning_count": fixed_threshold_warning_edges,
        "gradient_aware_neighbour_warning_count": gradient_warning_edges,
    }

    rankings = []
    for square_name in robot_square_map.square_names():
        record = per_square[square_name]
        rankings.append({
            "square": square_name,
            "suspicion_score": record["suspicion_score"],
            "error_count": record["error_count"],
            "warning_count": record["warning_count"],
            "gradient_warning_count": record["gradient_warning_count"],
            "source": record["source"],
        })
    rankings.sort(
        key=lambda item: (
            -item.get("suspicion_score", 0),
            -item.get("gradient_warning_count", 0),
            -item.get("error_count", 0),
            item.get("square"),
        )
    )

    return {
        "summary": summary,
        "gradient_baselines": gradient_baselines,
        "per_square_records": [per_square[name] for name in robot_square_map.square_names()],
        "errors": all_errors,
        "warnings": all_warnings,
        "suspicious_rankings": rankings,
        "neighbour_delta_records": neighbour_delta_records,
    }


def write_json_report(path, report):
    _ensure_parent_dir(path)
    with open(path, "w") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)


def write_csv_report(path, report):
    _ensure_parent_dir(path)
    columns = [
        "square",
        "row",
        "col",
        "source",
        "has_pose",
        "valid",
        "error_count",
        "warning_count",
        "gradient_warning_count",
        "nearest_manual_anchor",
        "nearest_manual_distance",
        "min_limit_margin",
        "max_neighbour_joint_delta",
        "total_neighbour_delta",
        "max_gradient_ratio",
        "suspicion_score",
    ]
    with open(path, "w") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in report.get("per_square_records", []):
            writer.writerow({key: row.get(key) for key in columns})


def write_markdown_report(path, report):
    _ensure_parent_dir(path)
    summary = report.get("summary", {})
    rankings = report.get("suspicious_rankings", [])
    errors = report.get("errors", [])
    warnings = report.get("warnings", [])

    high_errors = [entry for entry in errors if entry.get("square")]
    fixed_neighbour_records = [
        record for record in report.get("neighbour_delta_records", [])
        if record.get("fixed_threshold_warning")
    ]
    gradient_neighbour_records = [
        record for record in report.get("neighbour_delta_records", [])
        if record.get("gradient_warning")
    ]
    margin_warnings = [entry for entry in warnings if entry.get("code") == "low_limit_margin"]
    top_generated = [entry for entry in rankings if entry.get("source") == "generated"][:10]

    lines = []
    lines.append("# Square Target Audit")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("| --- | --- |")
    lines.append("| Total squares with above_pose | {} |".format(summary.get("total_squares_with_above_pose")))
    lines.append("| Manual count | {} |".format(summary.get("manual_count")))
    lines.append("| Generated count | {} |".format(summary.get("generated_count")))
    lines.append("| Missing count | {} |".format(summary.get("missing_count")))
    lines.append("| Recommended anchor count | {} / {} |".format(
        summary.get("recommended_anchor_count"), summary.get("recommended_anchor_target_count")))
    lines.append("| Validation errors | {} |".format(summary.get("validation_error_count")))
    lines.append("| Warnings | {} |".format(summary.get("warning_count")))
    lines.append("| Baseline percentile | {} |".format(summary.get("baseline_percentile")))
    lines.append("| Manual vertical baseline count | {} |".format(summary.get("manual_vertical_baseline_count")))
    lines.append("| Manual horizontal baseline count | {} |".format(summary.get("manual_horizontal_baseline_count")))
    lines.append("| Fixed-threshold neighbour warnings | {} |".format(summary.get("fixed_threshold_neighbour_warning_count")))
    lines.append("| Gradient-aware neighbour warnings | {} |".format(summary.get("gradient_aware_neighbour_warning_count")))
    lines.append("")

    lines.append("## Missing Recommended Anchors")
    lines.append("")
    missing_recommended = summary.get("missing_recommended_anchors") or []
    if missing_recommended:
        for square_name in missing_recommended:
            lines.append("- {}".format(square_name))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## High Severity Errors")
    lines.append("")
    if high_errors:
        for entry in high_errors[:30]:
            lines.append("- {}: {}".format(entry.get("square"), entry.get("message")))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Top Suspicious Generated Poses")
    lines.append("")
    if top_generated:
        for entry in top_generated:
            lines.append("- {} score={} errors={} warnings={} gradient_warnings={}".format(
                entry.get("square"),
                entry.get("suspicion_score"),
                entry.get("error_count"),
                entry.get("warning_count"),
                entry.get("gradient_warning_count"),
            ))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Fixed-Threshold Neighbour Warnings")
    lines.append("")
    if fixed_neighbour_records:
        for record in fixed_neighbour_records:
            lines.append("- {}<->{} max_joint_delta={} total_delta={} orientation={}".format(
                record.get("square_a"),
                record.get("square_b"),
                record.get("max_joint_delta"),
                record.get("total_delta"),
                record.get("orientation"),
            ))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Gradient-Aware Neighbour Warnings")
    lines.append("")
    if gradient_neighbour_records:
        for record in gradient_neighbour_records:
            ratio = record.get("gradient_ratio")
            ratio_str = "{:.3f}".format(ratio) if ratio is not None else "n/a"
            lines.append("- {}<->{} ratio={} orientation={}".format(
                record.get("square_a"),
                record.get("square_b"),
                ratio_str,
                record.get("orientation"),
            ))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Low Joint-Limit Margin Warnings")
    lines.append("")
    if margin_warnings:
        for entry in margin_warnings[:30]:
            lines.append("- {}: {}".format(entry.get("square"), entry.get("message")))
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Suggested Manual Correction Order")
    lines.append("")
    top_ranked = rankings[:20]
    if top_ranked:
        for entry in top_ranked:
            lines.append("1. {} (score={}, source={}, gradient_warnings={})".format(
                entry.get("square"),
                entry.get("suspicion_score"),
                entry.get("source"),
                entry.get("gradient_warning_count"),
            ))
    else:
        lines.append("- none")
    lines.append("")

    with open(path, "w") as handle:
        handle.write("\n".join(lines))


def print_console_summary(report, output_paths):
    summary = report.get("summary", {})
    print("Total squares with above_pose: {}".format(summary.get("total_squares_with_above_pose")))
    print("Manual count: {}".format(summary.get("manual_count")))
    print("Generated count: {}".format(summary.get("generated_count")))
    print("Missing count: {}".format(summary.get("missing_count")))
    print("Recommended anchor count: {} / {}".format(
        summary.get("recommended_anchor_count"), summary.get("recommended_anchor_target_count")))
    print("Validation errors: {}".format(summary.get("validation_error_count")))
    print("Warnings: {}".format(summary.get("warning_count")))
    print("Manual vertical baseline count: {}".format(summary.get("manual_vertical_baseline_count")))
    print("Manual horizontal baseline count: {}".format(summary.get("manual_horizontal_baseline_count")))
    print("Baseline percentile used: {}".format(summary.get("baseline_percentile")))
    print("Fixed-threshold neighbour warnings: {}".format(summary.get("fixed_threshold_neighbour_warning_count")))
    print("Gradient-aware neighbour warnings: {}".format(summary.get("gradient_aware_neighbour_warning_count")))

    print("Top suspicious squares:")
    for entry in report.get("suspicious_rankings", [])[:10]:
        if entry.get("suspicion_score", 0) <= 0:
            continue
        print("  {} score={} errors={} warnings={} gradient_warnings={} source={}".format(
            entry.get("square"),
            entry.get("suspicion_score"),
            entry.get("error_count"),
            entry.get("warning_count"),
            entry.get("gradient_warning_count"),
            entry.get("source"),
        ))

    if output_paths:
        print("Report paths written:")
        for path in output_paths:
            print("  {}".format(path))


def main():
    args = build_parser().parse_args()

    targets = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits_path)

    report = audit_square_targets_document(
        targets,
        joint_limits,
        limit_margin_warning=args.limit_margin_warning,
        joint_jump_warning=args.joint_jump_warning,
        total_jump_warning=args.total_jump_warning,
        gradient_aware=bool(args.gradient_aware),
        gradient_ratio_warning=args.gradient_ratio_warning,
        gradient_ratio_error=args.gradient_ratio_error,
        baseline_percentile=args.baseline_percentile,
    )

    output_paths = []
    if args.output_json:
        write_json_report(args.output_json, report)
        output_paths.append(args.output_json)
    if args.output_csv:
        write_csv_report(args.output_csv, report)
        output_paths.append(args.output_csv)
    if args.output_md:
        write_markdown_report(args.output_md, report)
        output_paths.append(args.output_md)

    print_console_summary(report, output_paths)

    errors_count = report.get("summary", {}).get("validation_error_count", 0)
    warnings_count = report.get("summary", {}).get("warning_count", 0)

    if args.strict and errors_count > 0:
        return 1
    if args.fail_on_warnings and warnings_count > 0:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
