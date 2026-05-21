#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map

DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_AUDIT_JSON_PATH = os.path.join(ROOT, "data", "debug", "square_target_audit.json")


def build_parser():
    parser = argparse.ArgumentParser(description="Explain square-level audit findings without touching hardware.")
    parser.add_argument("--square", required=True, help="Square name, for example b8")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path")
    parser.add_argument("--joint-limits", dest="joint_limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Joint-limits YAML path")
    parser.add_argument("--audit-json", default=DEFAULT_AUDIT_JSON_PATH,
                        help="Audit JSON report path from tools/audit_square_targets.py")
    parser.add_argument("--output-md", default=None, help="Optional markdown output path")
    parser.add_argument("--output-json", default=None, help="Optional JSON output path")
    parser.add_argument("--top", type=int, default=10, help="Top suspicious ranking cutoff")
    return parser


def _load_json(path):
    with open(path, "r") as handle:
        return json.load(handle)


def _safe_source(square_info):
    if not isinstance(square_info, dict):
        return None
    above_pose = square_info.get("above_pose")
    if not isinstance(above_pose, dict):
        return None
    return above_pose.get("source")


def _joint_bounds(joint_limits, joint_name):
    entry = joint_limits.get(joint_name)
    if not isinstance(entry, dict):
        return None
    lower = entry.get("provisional_min")
    upper = entry.get("provisional_max")
    if isinstance(lower, bool) or not isinstance(lower, int):
        return None
    if isinstance(upper, bool) or not isinstance(upper, int):
        return None
    return lower, upper


def _joint_limit_info(joints, joint_limits, joint_order):
    result = []
    nearest = None
    for joint_name in joint_order:
        value = joints.get(joint_name)
        bounds = _joint_bounds(joint_limits, joint_name)
        info = {
            "joint": joint_name,
            "value": value,
            "min": None,
            "max": None,
            "margin_to_min": None,
            "margin_to_max": None,
            "nearest_margin": None,
            "nearest_limit": None,
        }
        if bounds is None or isinstance(value, bool) or not isinstance(value, int):
            result.append(info)
            continue
        lower, upper = bounds
        margin_to_min = value - lower
        margin_to_max = upper - value
        nearest_margin = margin_to_min if margin_to_min < margin_to_max else margin_to_max
        nearest_limit = "min" if margin_to_min <= margin_to_max else "max"
        info.update({
            "min": lower,
            "max": upper,
            "margin_to_min": margin_to_min,
            "margin_to_max": margin_to_max,
            "nearest_margin": nearest_margin,
            "nearest_limit": nearest_limit,
        })
        if nearest is None or nearest_margin < nearest["nearest_margin"]:
            nearest = {
                "joint": joint_name,
                "nearest_margin": nearest_margin,
                "nearest_limit": nearest_limit,
                "value": value,
                "min": lower,
                "max": upper,
            }
        result.append(info)
    return result, nearest


def _neighbour_squares(square_name):
    row, col = robot_square_map.square_to_grid(square_name)
    pairs = [
        ("up", row + 1, col),
        ("down", row - 1, col),
        ("left", row, col + 1),
        ("right", row, col - 1),
    ]
    result = []
    for direction, next_row, next_col in pairs:
        if 0 <= next_row < robot_square_map.BOARD_SIZE and 0 <= next_col < robot_square_map.BOARD_SIZE:
            result.append({
                "direction": direction,
                "square": robot_square_map.grid_to_square(next_row, next_col),
                "row": next_row,
                "col": next_col,
            })
    return result


def _edge_lookup(audit_report):
    lookup = {}
    for record in audit_report.get("neighbour_delta_records", []):
        square_a = record.get("square_a")
        square_b = record.get("square_b")
        if not square_a or not square_b:
            continue
        key = tuple(sorted([square_a, square_b]))
        lookup[key] = record
    return lookup


def _manhattan_distance(square_a, square_b):
    row_a, col_a = robot_square_map.square_to_grid(square_a)
    row_b, col_b = robot_square_map.square_to_grid(square_b)
    return abs(row_a - row_b) + abs(col_a - col_b)


def _source_of_square(doc, square_name):
    square_info = doc.get("squares", {}).get(square_name)
    return _safe_source(square_info)


def _interpolation_info(doc, square_name, joints, joint_order):
    square_info = doc.get("squares", {}).get(square_name, {})
    above_pose = square_info.get("above_pose", {}) if isinstance(square_info, dict) else {}
    interpolation = above_pose.get("interpolation") if isinstance(above_pose, dict) else None
    anchors = []
    if not isinstance(interpolation, dict):
        return {
            "method": None,
            "anchors_used": [],
            "anchors": anchors,
        }
    anchors_used = interpolation.get("anchors_used")
    if not isinstance(anchors_used, list):
        anchors_used = []
    for anchor_name in anchors_used:
        try:
            normalized_anchor = robot_square_map.normalise_square_name(anchor_name)
        except ValueError:
            continue
        anchor_source = _source_of_square(doc, normalized_anchor)
        anchor_pose = doc.get("squares", {}).get(normalized_anchor, {}).get("above_pose", {})
        anchor_joints = anchor_pose.get("joints") if isinstance(anchor_pose, dict) else None
        per_joint_delta = {}
        if isinstance(anchor_joints, dict) and isinstance(joints, dict):
            for joint_name in joint_order:
                value = joints.get(joint_name)
                anchor_value = anchor_joints.get(joint_name)
                if isinstance(value, int) and not isinstance(value, bool) and isinstance(anchor_value, int) and not isinstance(anchor_value, bool):
                    per_joint_delta[joint_name] = abs(value - anchor_value)
        anchors.append({
            "square": normalized_anchor,
            "source": anchor_source,
            "grid_distance": _manhattan_distance(square_name, normalized_anchor),
            "per_joint_delta": per_joint_delta,
        })
    return {
        "method": interpolation.get("method"),
        "anchors_used": [entry.get("square") for entry in anchors],
        "anchors": anchors,
    }


def _manual_square_context(doc, square_name):
    manual_anchors = []
    generated_neighbours = []
    for other_square in robot_square_map.square_names():
        if other_square == square_name:
            continue
        source = _source_of_square(doc, other_square)
        if source == "manual":
            manual_anchors.append({
                "square": other_square,
                "distance": _manhattan_distance(square_name, other_square),
            })
    manual_anchors.sort(key=lambda item: (item["distance"], item["square"]))

    for info in _neighbour_squares(square_name):
        source = _source_of_square(doc, info["square"])
        if source == "generated":
            generated_neighbours.append({
                "square": info["square"],
                "direction": info["direction"],
                "distance": 1,
            })
    generated_neighbours.sort(key=lambda item: item["square"])
    return {
        "nearest_generated_neighbours": generated_neighbours,
        "nearest_manual_anchors": manual_anchors[:5],
    }


def _diagnose(source, has_fixed_warning, has_gradient_warning, nearest_manual_distance):
    if source == "manual" and has_gradient_warning:
        return "manual anchor suspicious"
    if source == "generated" and has_fixed_warning and not has_gradient_warning:
        return "likely fixed-threshold false positive"
    if source == "generated" and nearest_manual_distance is not None and nearest_manual_distance >= 4 and has_gradient_warning:
        return "likely interpolation artefact"
    if source == "generated" and (has_fixed_warning or has_gradient_warning):
        return "generated pose suspicious but anchors look plausible"
    if source == "generated" and nearest_manual_distance is not None and nearest_manual_distance >= 4:
        return "likely interpolation artefact"
    return "no strong warning"


def _suggest_action(diagnosis):
    if diagnosis == "manual anchor suspicious":
        return "physically recheck/reteach this manual anchor before correcting generated neighbours"
    if diagnosis in ("generated pose suspicious but anchors look plausible", "likely interpolation artefact"):
        return "consider manual reteach after checking adjacent manual anchors"
    return "do not prioritise this square"


def build_square_explanation(square_name, targets_doc, joint_limits, audit_report, top=10):
    square = robot_square_map.normalise_square_name(square_name)
    joint_order = list(targets_doc.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)

    per_square_lookup = {}
    for record in audit_report.get("per_square_records", []):
        key = record.get("square")
        if key:
            per_square_lookup[key] = record

    rankings = audit_report.get("suspicious_rankings", [])
    ranking_position = None
    for index, entry in enumerate(rankings):
        if entry.get("square") == square:
            ranking_position = index + 1
            break

    square_info = targets_doc.get("squares", {}).get(square, {})
    above_pose = square_info.get("above_pose", {}) if isinstance(square_info, dict) else {}
    source = above_pose.get("source") if isinstance(above_pose, dict) else None
    confidence = above_pose.get("confidence") if isinstance(above_pose, dict) else None
    joints = above_pose.get("joints") if isinstance(above_pose, dict) else None
    if not isinstance(joints, dict):
        joints = {}

    row, col = robot_square_map.square_to_grid(square)
    recommended = set(targets_doc.get("recommended_anchors") or robot_square_map.RECOMMENDED_ANCHORS)
    audit_square_record = per_square_lookup.get(square, {})

    joint_limits_info, nearest_limit = _joint_limit_info(joints, joint_limits, joint_order)

    edge_lookup = _edge_lookup(audit_report)
    neighbour_payload = []
    has_fixed_warning = False
    has_gradient_warning = False

    for neighbour in _neighbour_squares(square):
        neighbour_name = neighbour["square"]
        neighbour_info = targets_doc.get("squares", {}).get(neighbour_name, {})
        neighbour_pose = neighbour_info.get("above_pose", {}) if isinstance(neighbour_info, dict) else {}
        neighbour_source = neighbour_pose.get("source") if isinstance(neighbour_pose, dict) else None
        neighbour_joints = neighbour_pose.get("joints") if isinstance(neighbour_pose, dict) else None
        if not isinstance(neighbour_joints, dict):
            neighbour_joints = {}

        per_joint_delta = {}
        max_joint_delta = None
        total_l1_delta = None
        if joints and neighbour_joints:
            total = 0
            max_value = 0
            for joint_name in joint_order:
                value = joints.get(joint_name)
                neighbour_value = neighbour_joints.get(joint_name)
                if isinstance(value, int) and not isinstance(value, bool) and isinstance(neighbour_value, int) and not isinstance(neighbour_value, bool):
                    delta = abs(value - neighbour_value)
                    per_joint_delta[joint_name] = delta
                    total += delta
                    if delta > max_value:
                        max_value = delta
            total_l1_delta = total
            max_joint_delta = max_value

        edge_key = tuple(sorted([square, neighbour_name]))
        edge_record = edge_lookup.get(edge_key, {})
        fixed_warning = bool(edge_record.get("fixed_threshold_warning"))
        gradient_warning = bool(edge_record.get("gradient_warning"))
        has_fixed_warning = has_fixed_warning or fixed_warning
        has_gradient_warning = has_gradient_warning or gradient_warning

        neighbour_payload.append({
            "direction": neighbour["direction"],
            "square": neighbour_name,
            "source": neighbour_source,
            "per_joint_delta": per_joint_delta,
            "total_l1_delta": total_l1_delta,
            "max_joint_delta": max_joint_delta,
            "orientation": edge_record.get("orientation"),
            "gradient_ratio": edge_record.get("gradient_ratio"),
            "fixed_threshold_warning": fixed_warning,
            "gradient_warning": gradient_warning,
        })

    interpolation_info = None
    manual_context = None
    nearest_manual_distance = audit_square_record.get("nearest_manual_distance")
    if source == "generated":
        interpolation_info = _interpolation_info(targets_doc, square, joints, joint_order)
    elif source == "manual":
        manual_context = _manual_square_context(targets_doc, square)

    diagnosis = _diagnose(source, has_fixed_warning, has_gradient_warning, nearest_manual_distance)
    action = _suggest_action(diagnosis)

    return {
        "square": square,
        "basic_info": {
            "square": square,
            "row": row,
            "col": col,
            "source": source,
            "confidence": confidence,
            "is_recommended_anchor": square in recommended,
            "has_validation_errors": bool(audit_square_record.get("error_count", 0) > 0),
            "has_warnings": bool(audit_square_record.get("warning_count", 0) > 0),
            "error_count": int(audit_square_record.get("error_count", 0) or 0),
            "warning_count": int(audit_square_record.get("warning_count", 0) or 0),
            "ranking_position": ranking_position,
            "is_in_top": bool(ranking_position is not None and ranking_position <= int(top)),
            "top_cutoff": int(top),
        },
        "pose": {
            "joints": {joint_name: joints.get(joint_name) for joint_name in joint_order},
            "joint_limits": joint_limits_info,
            "nearest_software_limit": nearest_limit,
        },
        "neighbours": neighbour_payload,
        "interpolation": interpolation_info,
        "manual_context": manual_context,
        "diagnosis": diagnosis,
        "suggested_action": action,
    }


def render_console(explanation):
    lines = []
    info = explanation["basic_info"]
    lines.append("Square: {}".format(info["square"]))
    lines.append("Grid: row={} col={}".format(info["row"], info["col"]))
    lines.append("Source: {}  Confidence: {}".format(info["source"], info["confidence"]))
    lines.append("Recommended anchor: {}".format("yes" if info["is_recommended_anchor"] else "no"))
    lines.append("Audit flags: errors={} warnings={}".format(info["error_count"], info["warning_count"]))
    lines.append("Ranking: {} (top {}: {})".format(
        info["ranking_position"],
        info["top_cutoff"],
        "yes" if info["is_in_top"] else "no",
    ))
    lines.append("")

    lines.append("Pose joints:")
    for joint_name, value in explanation["pose"]["joints"].items():
        lines.append("  {}={}".format(joint_name, value))
    lines.append("Limit margins:")
    for item in explanation["pose"]["joint_limits"]:
        lines.append("  {} min={} max={} margin_to_min={} margin_to_max={} nearest={}({})".format(
            item["joint"],
            item["min"],
            item["max"],
            item["margin_to_min"],
            item["margin_to_max"],
            item["nearest_limit"],
            item["nearest_margin"],
        ))
    lines.append("Nearest software limit: {}".format(explanation["pose"]["nearest_software_limit"]))
    lines.append("")

    lines.append("Neighbours:")
    for item in explanation["neighbours"]:
        lines.append("  {} {} source={} max_delta={} total_l1={} ratio={} fixed_warn={} gradient_warn={}".format(
            item["direction"],
            item["square"],
            item["source"],
            item["max_joint_delta"],
            item["total_l1_delta"],
            item["gradient_ratio"],
            item["fixed_threshold_warning"],
            item["gradient_warning"],
        ))

    if explanation.get("interpolation") is not None:
        lines.append("")
        lines.append("Interpolation:")
        lines.append("  method={}".format(explanation["interpolation"].get("method")))
        for anchor in explanation["interpolation"].get("anchors", []):
            lines.append("  anchor {} source={} distance={} deltas={}".format(
                anchor["square"],
                anchor["source"],
                anchor["grid_distance"],
                anchor["per_joint_delta"],
            ))

    if explanation.get("manual_context") is not None:
        lines.append("")
        lines.append("Manual context:")
        lines.append("  nearest generated neighbours={}".format(explanation["manual_context"].get("nearest_generated_neighbours")))
        lines.append("  nearest manual anchors={}".format(explanation["manual_context"].get("nearest_manual_anchors")))

    lines.append("")
    lines.append("Diagnosis: {}".format(explanation["diagnosis"]))
    lines.append("Suggested action: {}".format(explanation["suggested_action"]))
    return "\n".join(lines)


def write_json(path, explanation):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as handle:
        json.dump(explanation, handle, indent=2, sort_keys=True)


def write_markdown(path, explanation):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    info = explanation["basic_info"]
    lines = []
    lines.append("# Square Audit Explanation: {}".format(explanation["square"]))
    lines.append("")
    lines.append("## Basic Info")
    lines.append("")
    lines.append("- square: {}".format(info["square"]))
    lines.append("- row/col: {}/{}".format(info["row"], info["col"]))
    lines.append("- source: {}".format(info["source"]))
    lines.append("- confidence: {}".format(info["confidence"]))
    lines.append("- recommended anchor: {}".format(info["is_recommended_anchor"]))
    lines.append("- validation errors: {}".format(info["has_validation_errors"]))
    lines.append("- warnings: {}".format(info["has_warnings"]))
    lines.append("- ranking: {} (top {}: {})".format(info["ranking_position"], info["top_cutoff"], info["is_in_top"]))
    lines.append("")

    lines.append("## Pose")
    lines.append("")
    lines.append("### Joint Ticks")
    for joint_name, value in explanation["pose"]["joints"].items():
        lines.append("- {}: {}".format(joint_name, value))
    lines.append("")
    lines.append("### Limit Margins")
    for item in explanation["pose"]["joint_limits"]:
        lines.append("- {}: min={} max={} margin_to_min={} margin_to_max={} nearest={}({})".format(
            item["joint"],
            item["min"],
            item["max"],
            item["margin_to_min"],
            item["margin_to_max"],
            item["nearest_limit"],
            item["nearest_margin"],
        ))
    lines.append("- nearest software limit: {}".format(explanation["pose"]["nearest_software_limit"]))
    lines.append("")

    lines.append("## Neighbours")
    lines.append("")
    for item in explanation["neighbours"]:
        lines.append("- {} {} source={} total={} max={} ratio={} fixed_warn={} gradient_warn={} deltas={}".format(
            item["direction"],
            item["square"],
            item["source"],
            item["total_l1_delta"],
            item["max_joint_delta"],
            item["gradient_ratio"],
            item["fixed_threshold_warning"],
            item["gradient_warning"],
            item["per_joint_delta"],
        ))
    lines.append("")

    if explanation.get("interpolation") is not None:
        lines.append("## Interpolation")
        lines.append("")
        lines.append("- method: {}".format(explanation["interpolation"].get("method")))
        for anchor in explanation["interpolation"].get("anchors", []):
            lines.append("- anchor {} source={} distance={} deltas={}".format(
                anchor["square"],
                anchor["source"],
                anchor["grid_distance"],
                anchor["per_joint_delta"],
            ))
        lines.append("")

    if explanation.get("manual_context") is not None:
        lines.append("## Manual Context")
        lines.append("")
        lines.append("- nearest generated neighbours: {}".format(explanation["manual_context"].get("nearest_generated_neighbours")))
        lines.append("- nearest manual anchors: {}".format(explanation["manual_context"].get("nearest_manual_anchors")))
        lines.append("")

    lines.append("## Local Diagnosis")
    lines.append("")
    lines.append("- {}".format(explanation["diagnosis"]))
    lines.append("")
    lines.append("## Suggested Action")
    lines.append("")
    lines.append("- {}".format(explanation["suggested_action"]))
    lines.append("")

    with open(path, "w") as handle:
        handle.write("\n".join(lines))


def main():
    args = build_parser().parse_args()
    try:
        square = robot_square_map.normalise_square_name(args.square)
    except ValueError as exc:
        print("ERROR: {}".format(exc))
        return 1

    targets_doc = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits_path)
    audit_report = _load_json(args.audit_json)

    explanation = build_square_explanation(square, targets_doc, joint_limits, audit_report, top=args.top)
    print(render_console(explanation))

    if args.output_json:
        write_json(args.output_json, explanation)
    if args.output_md:
        write_markdown(args.output_md, explanation)
    return 0


if __name__ == "__main__":
    sys.exit(main())
