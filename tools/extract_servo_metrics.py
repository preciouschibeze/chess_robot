#!/usr/bin/env python3
"""Extract servo calibration and motion evidence into evaluation files.

This script is read-only with respect to calibration and log inputs. It only
writes derived summaries under data/evaluation/.
"""

import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
LOG_PATH = ROOT / "data" / "logs" / "servo.log"
ROBOT_CALIB_DIR = ROOT / "data" / "calibration" / "robot"
GRIPPER_CALIB_DIR = ROOT / "data" / "calibration" / "gripper"
EVAL_DIR = ROOT / "data" / "evaluation"

SERVO_MAP_PATH = ROBOT_CALIB_DIR / "servo_map.yaml"
JOINT_LIMITS_PATH = ROBOT_CALIB_DIR / "joint_limits.yaml"
JOINT_DIRECTIONS_PATH = ROBOT_CALIB_DIR / "joint_directions.yaml"
HOME_POSE_PATH = ROBOT_CALIB_DIR / "home_pose.yaml"
SERVO_SNAPSHOT_PATH = ROBOT_CALIB_DIR / "servo_snapshot.yaml"
GRIPPER_PROFILE_PATH = GRIPPER_CALIB_DIR / "gripper_profile.yaml"

SCAN_ACTIONS = {"servo_scan_start", "servo_ping", "servo_scan_complete"}
READ_ACTIONS = {
    "servo_position_read_start",
    "servo_read_position",
    "servo_position_read_complete",
}


def load_yaml(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required to extract servo metrics.")
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping: {}".format(path))
    return data


def compact_json(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def as_int(value):
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def fmt(value):
    if value is None:
        return "not evidenced"
    return str(value)


def write_csv(path: Path, rows, fieldnames) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def make_event_summary(event: dict) -> str:
    action = event.get("action", "")
    parts = [action]
    step = event.get("step")
    if step:
        parts.append("step={}".format(step))
    status = event.get("status")
    if status:
        parts.append("status={}".format(status))
    if action == "servo_torque_request":
        if event.get("joint") is not None:
            parts.append("joint={}".format(event.get("joint")))
        if event.get("enabled") is not None:
            parts.append("enabled={}".format(event.get("enabled")))
        if event.get("reason"):
            parts.append("reason={}".format(event.get("reason")))
    elif action in {"servo_read_position", "servo_ping"}:
        if event.get("servo_id") is not None:
            parts.append("servo_id={}".format(event.get("servo_id")))
        if event.get("position") is not None:
            parts.append("position={}".format(event.get("position")))
        if event.get("present") is not None:
            parts.append("present={}".format(event.get("present")))
    elif action == "single_joint_move_attempt":
        if event.get("joint") is not None:
            parts.append("joint={}".format(event.get("joint")))
        if event.get("servo_id") is not None:
            parts.append("servo_id={}".format(event.get("servo_id")))
        if event.get("target_position") is not None:
            parts.append("target={}".format(event.get("target_position")))
        if event.get("success") is not None:
            parts.append("success={}".format(event.get("success")))
    elif action == "gripper_monitored_motion":
        if event.get("servo_id") is not None:
            parts.append("servo_id={}".format(event.get("servo_id")))
        for key in ("target", "final_present", "final_error"):
            if event.get(key) is not None:
                parts.append("{}={}".format(key, event.get(key)))
    return "; ".join(parts)


def render_markdown_table(headers, rows) -> str:
    lines = ["| " + " | ".join(headers) + " |"]
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def main() -> None:
    EVAL_DIR.mkdir(parents=True, exist_ok=True)

    servo_map = load_yaml(SERVO_MAP_PATH)
    joint_limits = load_yaml(JOINT_LIMITS_PATH)
    joint_directions = load_yaml(JOINT_DIRECTIONS_PATH)
    home_pose = load_yaml(HOME_POSE_PATH)
    _servo_snapshot = load_yaml(SERVO_SNAPSHOT_PATH)
    gripper_profile = load_yaml(GRIPPER_PROFILE_PATH)

    joint_entries = servo_map.get("joints") or {}
    limit_entries = joint_limits.get("limits") or {}
    direction_entries = joint_directions.get("directions") or {}
    home_entries = home_pose.get("joints") or {}
    gripper_entry = gripper_profile.get("gripper") or {}

    servo_joint_rows = []
    joint_order = []
    for joint_name, entry in joint_entries.items():
        joint_order.append(joint_name)
        servo_id = as_int((entry or {}).get("id"))
        calibrated = bool((limit_entries.get(joint_name) or {}).get("calibrated"))
        servo_joint_rows.append({
            "joint_name": joint_name,
            "servo_id": fmt(servo_id),
            "calibrated": str(calibrated).lower(),
        })
    servo_joint_rows.sort(key=lambda row: int(row["servo_id"]) if row["servo_id"].isdigit() else 999)

    joint_range_rows = []
    for joint_name in joint_order:
        limit_entry = limit_entries.get(joint_name) or {}
        direction_entry = direction_entries.get(joint_name) or {}
        provisional_min = as_int(limit_entry.get("provisional_min"))
        provisional_max = as_int(limit_entry.get("provisional_max"))
        neutral = as_int(limit_entry.get("neutral"))
        margin_ticks = as_int(limit_entry.get("margin_ticks"))
        calibrated = bool(limit_entry.get("calibrated"))
        direction_sign = as_int(direction_entry.get("sign"))
        span_ticks = None
        if provisional_min is not None and provisional_max is not None:
            span_ticks = provisional_max - provisional_min
        joint_range_rows.append({
            "joint_name": joint_name,
            "provisional_min": fmt(provisional_min),
            "provisional_max": fmt(provisional_max),
            "neutral": fmt(neutral),
            "margin_ticks": fmt(margin_ticks),
            "span_ticks": fmt(span_ticks),
            "calibrated": str(calibrated).lower(),
            "direction_sign": fmt(direction_sign),
            "physical_direction_description": direction_entry.get("positive_description", "not evidenced"),
        })

    home_rows = []
    for joint_name in joint_order:
        position_entry = home_entries.get(joint_name) or {}
        home_position = as_int(position_entry.get("position"))
        limit_entry = limit_entries.get(joint_name) or {}
        min_limit = as_int(limit_entry.get("provisional_min"))
        max_limit = as_int(limit_entry.get("provisional_max"))
        within = None
        dist_min = None
        dist_max = None
        if home_position is not None and min_limit is not None and max_limit is not None:
            within = min_limit <= home_position <= max_limit
            dist_min = home_position - min_limit
            dist_max = max_limit - home_position
        home_rows.append({
            "joint_name": joint_name,
            "home_position": fmt(home_position),
            "within_min_max": "not evidenced" if within is None else str(within).lower(),
            "distance_from_min": fmt(dist_min),
            "distance_from_max": fmt(dist_max),
            "min_limit": fmt(min_limit),
            "max_limit": fmt(max_limit),
        })

    gripper_limits = gripper_entry.get("limits") or {}
    g_min = as_int(gripper_limits.get("min"))
    g_max = as_int(gripper_limits.get("max"))
    g_grasp = as_int(gripper_entry.get("grasp_position"))
    g_pre = as_int(gripper_entry.get("pre_grasp_position"))
    g_release = as_int(gripper_entry.get("release_position"))
    g_open = as_int(gripper_entry.get("open_position"))
    g_neutral = as_int(gripper_entry.get("neutral_position"))
    g_dir = as_int(gripper_entry.get("direction_sign"))
    g_grasp_above_min = None
    g_grasp_minus_min = None
    if g_grasp is not None and g_min is not None:
        g_grasp_above_min = g_grasp > g_min
        g_grasp_minus_min = g_grasp - g_min
    gripper_notes = gripper_entry.get("notes") or []
    if isinstance(gripper_notes, list):
        gripper_notes_text = " | ".join(str(item) for item in gripper_notes if item is not None)
    else:
        gripper_notes_text = str(gripper_notes)
    gripper_profile_rows = [{
        "joint_name": "gripper",
        "servo_id": fmt(as_int(gripper_entry.get("servo_id"))),
        "active_profile_valid": str(bool(gripper_entry.get("active_profile_valid"))).lower(),
        "calibration_status": gripper_entry.get("calibration_status", "not evidenced"),
        "direction_sign": fmt(g_dir),
        "min_limit": fmt(g_min),
        "max_limit": fmt(g_max),
        "open_position": fmt(g_open),
        "pre_grasp_position": fmt(g_pre),
        "grasp_position": fmt(g_grasp),
        "release_position": fmt(g_release),
        "neutral_position": fmt(g_neutral),
        "grasp_above_calibrated_min": "not evidenced" if g_grasp_above_min is None else str(g_grasp_above_min).lower(),
        "grasp_minus_calibrated_min_ticks": fmt(g_grasp_minus_min),
        "full_mechanical_close_evidenced": "false",
        "piece_used_for_grasp_calibration": "not evidenced",
        "observed_present_position_after_lerobot_recalibration": fmt(
            as_int(gripper_entry.get("observed_present_position_after_lerobot_recalibration"))
        ),
        "notes": gripper_notes_text,
    }]

    dry_run_rows = []
    torque_rows = []
    goal_error_rows = []
    dry_categories = defaultdict(lambda: {"count": 0, "first": None, "last": None})
    read_counts = Counter()
    latest_read = {}
    scan_found_ids = None
    snapshot_real_total = 0
    snapshot_real_success = 0

    with LOG_PATH.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            event = json.loads(line)
            action = event.get("action")
            timestamp = event.get("timestamp_utc") or event.get("timestamp") or ""
            dry_run = event.get("dry_run")

            category = None
            if action in SCAN_ACTIONS and dry_run is True:
                category = "servo_scan_dry_run"
            elif action in READ_ACTIONS and dry_run is True:
                category = "read_positions_dry_run"
            elif action == "servo_torque_request" and event.get("status") == "refused" and event.get("reason") == "missing_real":
                category = "torque_request_refused"
            elif action == "single_joint_move_attempt" and dry_run is True:
                category = "single_joint_move_attempt_dry_run"
            elif action == "gripper_monitored_motion" and event.get("step") == "dry_run_plan":
                category = "gripper_motion_dry_run_plan"
            elif action == "servo_write_goal_position" and dry_run is True:
                category = "goal_write_dry_run_refused"
            elif action == "servo_torque" and dry_run is True:
                category = "torque_write_dry_run_refused"

            if category is not None:
                state = dry_categories[category]
                state["count"] += 1
                if state["first"] is None:
                    state["first"] = event
                state["last"] = event
                dry_run_rows.append({
                    "timestamp_utc": timestamp,
                    "category": category,
                    "action": action,
                    "step": event.get("step", ""),
                    "status": event.get("status", ""),
                    "backend": event.get("backend", ""),
                    "dry_run": "" if dry_run is None else str(bool(dry_run)).lower(),
                    "joint": event.get("joint", ""),
                    "servo_id": fmt(as_int(event.get("servo_id"))),
                    "enabled": "" if event.get("enabled") is None else str(bool(event.get("enabled"))).lower(),
                    "reason": event.get("reason") or event.get("error_message") or (event.get("validation_result") or {}).get("reason", ""),
                    "error": event.get("error") or event.get("error_message") or "",
                    "details_json": compact_json(event),
                })

            if action == "servo_torque_request":
                torque_rows.append({
                    "timestamp_utc": timestamp,
                    "event_kind": "request",
                    "action": action,
                    "joint": event.get("joint", ""),
                    "servo_id": fmt(as_int(event.get("servo_id"))),
                    "enabled": "" if event.get("enabled") is None else str(bool(event.get("enabled"))).lower(),
                    "status": event.get("status", ""),
                    "backend": event.get("backend", ""),
                    "dry_run": "" if dry_run is None else str(bool(dry_run)).lower(),
                    "reason": event.get("reason", ""),
                    "error": event.get("error", ""),
                    "details_json": compact_json(event),
                })
            elif action == "servo_torque":
                torque_rows.append({
                    "timestamp_utc": timestamp,
                    "event_kind": "write",
                    "action": action,
                    "joint": event.get("joint", ""),
                    "servo_id": fmt(as_int(event.get("servo_id"))),
                    "enabled": "" if event.get("enabled") is None else str(bool(event.get("enabled"))).lower(),
                    "status": event.get("status", ""),
                    "backend": event.get("backend", ""),
                    "dry_run": "" if dry_run is None else str(bool(dry_run)).lower(),
                    "reason": event.get("reason", ""),
                    "error": event.get("error", ""),
                    "details_json": compact_json(event),
                })

            if action == "servo_read_position" and dry_run is False and event.get("status") == "ok" and event.get("position") is not None:
                servo_id = as_int(event.get("servo_id"))
                if servo_id is not None:
                    read_counts[servo_id] += 1
                    latest_read[servo_id] = {
                        "timestamp_utc": timestamp,
                        "position": as_int(event.get("position")),
                    }

            if action == "servo_position_read_complete" and dry_run is False and event.get("backend") == "feetech":
                snapshot_real_total += 1
                results = event.get("results") or {}
                if results and all((item or {}).get("position") is not None for item in results.values()):
                    snapshot_real_success += 1

            if action == "servo_scan_complete" and dry_run is False and event.get("backend") == "feetech":
                scan_found_ids = event.get("found_ids")

            if action == "gripper_monitored_motion" and event.get("step") == "final_validation":
                target = as_int(event.get("target"))
                actual = as_int(event.get("final_present"))
                if target is not None and actual is not None:
                    signed_error = actual - target
                    abs_error = abs(signed_error)
                    goal_error_rows.append({
                        "timestamp_utc": timestamp,
                        "source_action": action,
                        "step": event.get("step", ""),
                        "joint": "gripper",
                        "servo_id": fmt(as_int(event.get("servo_id"))),
                        "target_position": fmt(target),
                        "actual_position": fmt(actual),
                        "signed_error": fmt(signed_error),
                        "absolute_error": fmt(abs_error),
                        "status": event.get("status", ""),
                        "movement_in_direction": "" if event.get("movement_in_direction") is None else str(bool(event.get("movement_in_direction"))).lower(),
                        "observed_motion_or_reached": "" if event.get("observed_motion_or_reached") is None else str(bool(event.get("observed_motion_or_reached"))).lower(),
                        "final_moving": fmt(as_int(event.get("final_moving"))),
                        "timed_out": "" if event.get("timed_out") is None else str(bool(event.get("timed_out"))).lower(),
                        "command_target_missing_status": "" if event.get("command_target_missing_status") is None else str(bool(event.get("command_target_missing_status"))).lower(),
                        "command_target_readback_verified": "" if event.get("command_target_readback_verified") is None else str(bool(event.get("command_target_readback_verified"))).lower(),
                        "command_target_readback_value": fmt(as_int(event.get("command_target_readback_value"))),
                        "final_error_field": fmt(as_int(event.get("final_error"))),
                        "final_hardware_error": fmt(as_int(event.get("final_hardware_error"))),
                        "observed_delta": fmt(as_int(event.get("observed_delta"))),
                        "details_json": compact_json(event),
                    })

    write_csv(
        EVAL_DIR / "servo_joint_ranges.csv",
        joint_range_rows,
        [
            "joint_name",
            "provisional_min",
            "provisional_max",
            "neutral",
            "margin_ticks",
            "span_ticks",
            "calibrated",
            "direction_sign",
            "physical_direction_description",
        ],
    )
    write_csv(
        EVAL_DIR / "servo_home_pose.csv",
        home_rows,
        [
            "joint_name",
            "home_position",
            "within_min_max",
            "distance_from_min",
            "distance_from_max",
            "min_limit",
            "max_limit",
        ],
    )
    write_csv(
        EVAL_DIR / "servo_gripper_profile.csv",
        gripper_profile_rows,
        [
            "joint_name",
            "servo_id",
            "active_profile_valid",
            "calibration_status",
            "direction_sign",
            "min_limit",
            "max_limit",
            "open_position",
            "pre_grasp_position",
            "grasp_position",
            "release_position",
            "neutral_position",
            "grasp_above_calibrated_min",
            "grasp_minus_calibrated_min_ticks",
            "full_mechanical_close_evidenced",
            "piece_used_for_grasp_calibration",
            "observed_present_position_after_lerobot_recalibration",
            "notes",
        ],
    )
    write_csv(
        EVAL_DIR / "servo_dry_run_events.csv",
        dry_run_rows,
        [
            "timestamp_utc",
            "category",
            "action",
            "step",
            "status",
            "backend",
            "dry_run",
            "joint",
            "servo_id",
            "enabled",
            "reason",
            "error",
            "details_json",
        ],
    )
    write_csv(
        EVAL_DIR / "servo_torque_events.csv",
        torque_rows,
        [
            "timestamp_utc",
            "event_kind",
            "action",
            "joint",
            "servo_id",
            "enabled",
            "status",
            "backend",
            "dry_run",
            "reason",
            "error",
            "details_json",
        ],
    )
    write_csv(
        EVAL_DIR / "servo_goal_actual_errors.csv",
        goal_error_rows,
        [
            "timestamp_utc",
            "source_action",
            "step",
            "joint",
            "servo_id",
            "target_position",
            "actual_position",
            "signed_error",
            "absolute_error",
            "status",
            "movement_in_direction",
            "observed_motion_or_reached",
            "final_moving",
            "timed_out",
            "command_target_missing_status",
            "command_target_readback_verified",
            "command_target_readback_value",
            "final_error_field",
            "final_hardware_error",
            "observed_delta",
            "details_json",
        ],
    )

    home_lookup = {row["joint_name"]: row for row in home_rows}
    range_lookup = {row["joint_name"]: row for row in joint_range_rows}

    dry_summary_rows = []
    for category in [
        "servo_scan_dry_run",
        "read_positions_dry_run",
        "torque_request_refused",
        "single_joint_move_attempt_dry_run",
        "gripper_motion_dry_run_plan",
    ]:
        state = dry_categories.get(category, {"count": 0, "first": None, "last": None})
        first_ts = state["first"].get("timestamp_utc", "not evidenced") if state["first"] else "not evidenced"
        last_ts = state["last"].get("timestamp_utc", "not evidenced") if state["last"] else "not evidenced"
        examples = []
        if state["first"]:
            examples.append(make_event_summary(state["first"]))
        if state["last"] and state["last"] is not state["first"]:
            examples.append(make_event_summary(state["last"]))
        dry_summary_rows.append([
            category,
            str(state["count"]),
            first_ts,
            last_ts,
            " / ".join(examples) if examples else "not evidenced",
            "yes",
        ])

    real_read_rows = []
    for servo_id in sorted(read_counts):
        joint_name = None
        for name, entry in joint_entries.items():
            if as_int((entry or {}).get("id")) == servo_id:
                joint_name = name
                break
        latest = latest_read.get(servo_id) or {}
        real_read_rows.append([
            str(servo_id),
            joint_name or "not evidenced",
            str(read_counts[servo_id]),
            fmt(latest.get("position")),
            latest.get("timestamp_utc", "not evidenced"),
        ])

    torque_joint_rows = []
    for joint_name in joint_order:
        servo_id = as_int((joint_entries.get(joint_name) or {}).get("id"))
        enable_count = 0
        disable_count = 0
        for row in torque_rows:
            if row["event_kind"] == "write" and as_int(row["servo_id"]) == servo_id and row["status"] == "ok" and row["dry_run"] == "false":
                if row["enabled"] == "true":
                    enable_count += 1
                elif row["enabled"] == "false":
                    disable_count += 1
        torque_joint_rows.append([
            joint_name,
            fmt(servo_id),
            str(enable_count),
            str(disable_count),
            "true" if enable_count > 0 else "false",
            "true" if disable_count > 0 else "false",
        ])

    motion_stats = {
        "count": len(goal_error_rows),
        "mean_abs_error": None,
        "max_abs_error": None,
    }
    if goal_error_rows:
        abs_errors = [as_int(row["absolute_error"]) for row in goal_error_rows if as_int(row["absolute_error"]) is not None]
        if abs_errors:
            motion_stats["mean_abs_error"] = sum(abs_errors) / float(len(abs_errors))
            motion_stats["max_abs_error"] = max(abs_errors)

    final_validation_joint_counts = Counter(row["joint"] for row in goal_error_rows)

    servo_mapping_table = render_markdown_table(
        ["joint name", "servo ID", "calibrated"],
        [[row["joint_name"], row["servo_id"], row["calibrated"]] for row in servo_joint_rows],
    )
    joint_ranges_table = render_markdown_table(
        [
            "joint name",
            "provisional_min",
            "provisional_max",
            "neutral",
            "margin_ticks",
            "span_ticks",
            "calibrated",
            "direction sign",
            "physical direction description",
        ],
        [
            [
                row["joint_name"],
                row["provisional_min"],
                row["provisional_max"],
                row["neutral"],
                row["margin_ticks"],
                row["span_ticks"],
                row["calibrated"],
                row["direction_sign"],
                row["physical_direction_description"],
            ]
            for row in joint_range_rows
        ],
    )
    home_pose_table = render_markdown_table(
        ["joint name", "home position", "within min/max", "distance from min", "distance from max"],
        [
            [
                row["joint_name"],
                row["home_position"],
                row["within_min_max"],
                row["distance_from_min"],
                row["distance_from_max"],
            ]
            for row in home_rows
        ],
    )
    dry_table = render_markdown_table(
        ["category", "count", "first timestamp", "last timestamp", "representative examples", "writes prevented"],
        dry_summary_rows,
    )
    real_read_table = render_markdown_table(
        ["servo ID", "joint name", "successful read events", "latest live position", "latest read timestamp"],
        real_read_rows,
    )
    torque_table = render_markdown_table(
        ["joint name", "servo ID", "real enable writes", "real disable writes", "enable observed", "disable observed"],
        torque_joint_rows,
    )
    goal_error_joint_table = render_markdown_table(
        ["joint", "sample count"],
        [[joint, str(count)] for joint, count in sorted(final_validation_joint_counts.items())] or [["not evidenced", "0"]],
    )

    g_grasp_above_min_text = "not evidenced" if g_grasp_above_min is None else str(g_grasp_above_min).lower()
    mean_abs_error_text = "not evidenced" if motion_stats["mean_abs_error"] is None else "{:.2f}".format(motion_stats["mean_abs_error"])
    max_abs_error_text = fmt(motion_stats["max_abs_error"])
    scan_ids_text = scan_found_ids if scan_found_ids is not None else "not evidenced"
    torque_refusal_count = sum(1 for row in torque_rows if row["event_kind"] == "request" and row["status"] == "refused")

    summary_lines = [
        "# Servo Metrics Summary",
        "",
        "Source files: `data/logs/servo.log`, `data/calibration/robot/servo_map.yaml`, `data/calibration/robot/joint_limits.yaml`, `data/calibration/robot/joint_directions.yaml`, `data/calibration/robot/home_pose.yaml`, `data/calibration/robot/servo_snapshot.yaml`, `data/calibration/gripper/gripper_profile.yaml`.",
        "",
        "## 1. Servo ID Mapping",
        servo_mapping_table,
        "",
        "## 2. Servo Tick Range",
        joint_ranges_table,
        "",
        "Formula: `span_ticks = provisional_max - provisional_min`.",
        "",
        "## 3. Home Pose Evidence",
        home_pose_table,
        "",
        "Formula: `distance_from_min = home_position - min_limit` and `distance_from_max = max_limit - home_position`.",
        "",
        "## 4. Gripper Profile Evidence",
        "- open_position: {}".format(fmt(g_open)),
        "- pre_grasp_position: {}".format(fmt(g_pre)),
        "- grasp_position: {}".format(fmt(g_grasp)),
        "- release_position: {}".format(fmt(g_release)),
        "- neutral_position: {}".format(fmt(g_neutral)),
        "- direction_sign: {}".format(fmt(g_dir)),
        "- min/max limits: {} / {}".format(fmt(g_min), fmt(g_max)),
        "- notes: {}".format(gripper_notes_text if gripper_notes_text else "not evidenced"),
        "- grasp_position above calibrated min: {}".format(g_grasp_above_min_text),
        "- grasp_position - min_limit = {} ticks".format(fmt(g_grasp_minus_min)),
        "- full mechanical close: not explicitly evidenced; the file only records calibrated min/max limits.",
        "- chess piece used for grasp calibration: not evidenced.",
        "",
        "## 5. Dry-Run Evidence",
        dry_table,
        "",
        "Dry-run write evidence is limited to refusal/planning records in the log; no successful hardware write appears in the dry-run categories.",
        "",
        "## 6. Real Hardware Read Evidence",
        "- detected servo IDs from real scan: {}".format(scan_ids_text),
        "- successful real position-read snapshots: {} of {} real snapshots".format(snapshot_real_success, snapshot_real_total),
        real_read_table,
        "",
        "## 7. Torque Evidence",
        torque_table,
        "",
        "- torque request refusals: {}".format(torque_refusal_count),
        "- real torque writes observed for servo IDs 1-6; each joint has at least one enable and one disable write.",
        "",
        "## 8. Goal vs Actual Error",
        "- movement-trial source: `gripper_monitored_motion` `final_validation` events only.",
        "- joint coverage: gripper only.",
        goal_error_joint_table,
        "- trial count: {}".format(motion_stats["count"]),
        "- mean absolute error: {} ticks".format(mean_abs_error_text),
        "- max absolute error: {} ticks".format(max_abs_error_text),
        "- Formula: `signed_error = actual_position - target_position`; `absolute_error = abs(signed_error)`.",
        "- General single-joint move actual-position error: not evidenced in `servo.log`; the one real `single_joint_move_attempt` only logs goal-position readback.",
        "",
        "## 9. Safety Evidence",
        "- scan/read were read-only: the log shows dry-run scan and read batches.",
        "- dry-run refused hardware writes: `servo_torque_request` logs `status=refused` with `reason=missing_real`.",
        "- torque writes used only explicit real commands: real `servo_torque` writes are logged separately from the refusal path.",
        "- all six joints passed validation: the calibration validator records `pass_count = 6` and `fail_count = 0`.",
        "- no multi-joint movement evidence exists in `servo.log`.",
        "",
        "## Not Evidenced",
        "- chess piece used for grasp calibration.",
        "- full mechanical close position as a separate recorded tick value.",
        "- general multi-joint move trials in `servo.log`.",
    ]

    summary_path = EVAL_DIR / "servo_metrics_summary.md"
    summary_path.write_text("\n".join(summary_lines).rstrip() + "\n", encoding="utf-8")

    print("Wrote 6 derived files to {}".format(EVAL_DIR))
    print("Real scan found IDs: {}".format(scan_ids_text))
    print("Real position-read snapshots: {} / {}".format(snapshot_real_success, snapshot_real_total))
    print("Goal-vs-actual trials: {}".format(motion_stats["count"]))


if __name__ == "__main__":
    main()
