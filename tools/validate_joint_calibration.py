import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import (
    ServoBusError,
    ServoEventLogger,
    build_servo_bus,
    load_robot_config,
)

CANONICAL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

DEFAULT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_DIRECTIONS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_directions.yaml")
DEFAULT_LOG_PATH = os.path.join(ROOT, "data", "logs", "servo.log")


def _require_yaml():
    if yaml is None:
        raise ServoBusError("PyYAML is required for joint calibration validation.")


def _read_yaml(path):
    _require_yaml()
    if not os.path.exists(path):
        raise ServoBusError("Required calibration file not found: {}".format(path))
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("Calibration file must contain a YAML mapping: {}".format(path))
    return data


def _yaml_scalar(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    return '"{}"'.format(str(value).replace('"', '\\"'))


def _write_lines(path, lines):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _load_servo_map(path):
    data = _read_yaml(path)
    joints = data.get("joints") or {}
    aliases = data.get("aliases") or {}
    if not isinstance(joints, dict):
        raise ServoBusError("servo_map.yaml 'joints' must be a mapping.")
    if not isinstance(aliases, dict):
        raise ServoBusError("servo_map.yaml 'aliases' must be a mapping.")
    return {"joints": joints, "aliases": aliases}


def _load_limits(path):
    data = _read_yaml(path)
    limits = data.get("limits") or data.get("joints") or {}
    if not isinstance(limits, dict):
        raise ServoBusError("joint_limits.yaml must contain a 'limits' mapping.")
    return {"limits": limits}


def _load_directions(path):
    data = _read_yaml(path)
    directions = data.get("directions") or data.get("joints") or {}
    if not isinstance(directions, dict):
        raise ServoBusError("joint_directions.yaml must contain a 'directions' mapping.")
    return {"directions": directions}


def _write_limits_yaml(path, data):
    limits = data.get("limits") or {}
    lines = ["limits:"]
    for joint_name in CANONICAL_JOINTS:
        entry = limits.get(joint_name) or {}
        lines.append("  {}:".format(joint_name))
        for key in (
            "provisional_min",
            "provisional_max",
            "neutral",
            "margin_ticks",
            "calibrated",
            "notes",
        ):
            lines.append("    {}: {}".format(key, _yaml_scalar(entry.get(key))))
    _write_lines(path, lines)


def _resolve_joint_name(requested, servo_map):
    if requested is None:
        return None
    if requested in CANONICAL_JOINTS:
        return requested
    aliases = servo_map.get("aliases") or {}
    if requested in aliases:
        return aliases[requested]
    raise ServoBusError("Unknown joint {!r}. Refusing to guess.".format(requested))


def _optional_int(value):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _string_or_dash(value):
    if value is None:
        return "-"
    return str(value)


def _status_and_reason(issues, warnings, live_position, live_requested):
    if issues:
        return "FAIL", "; ".join(issues)
    if warnings:
        return "WARN", "; ".join(warnings)
    if live_requested and live_position is not None:
        return "PASS", "live position within range"
    return "PASS", "static checks passed"


def _validate_joint(
    joint_name,
    servo_map,
    limits_data,
    directions_data,
    live_requested,
    bus,
    live_backend_available,
    live_backend_error,
):
    issues = []
    warnings = []

    servo_entry = (servo_map.get("joints") or {}).get(joint_name)
    limit_entry = (limits_data.get("limits") or {}).get(joint_name) or {}
    direction_entry = (directions_data.get("directions") or {}).get(joint_name) or {}

    servo_id = None
    if servo_entry is None:
        issues.append("joint missing from servo_map.yaml")
    elif not isinstance(servo_entry, dict):
        issues.append("servo_map entry must be a mapping")
    else:
        raw_id = servo_entry.get("id", servo_entry.get("servo_id"))
        if raw_id is None:
            issues.append("mapped servo ID missing")
        else:
            try:
                servo_id = safety.validate_servo_id(raw_id)
            except (ServoBusError, ValueError) as exc:
                issues.append(str(exc))

    min_value = None
    max_value = None
    neutral = None
    if not isinstance(limit_entry, dict):
        issues.append("joint_limits.yaml entry must be a mapping")
    else:
        if "provisional_min" not in limit_entry or limit_entry.get("provisional_min") is None:
            issues.append("provisional_min missing")
        else:
            min_value = _optional_int(limit_entry.get("provisional_min"))
            if min_value is None:
                issues.append("provisional_min must be an integer")

        if "provisional_max" not in limit_entry or limit_entry.get("provisional_max") is None:
            issues.append("provisional_max missing")
        else:
            max_value = _optional_int(limit_entry.get("provisional_max"))
            if max_value is None:
                issues.append("provisional_max must be an integer")

        if "neutral" not in limit_entry or limit_entry.get("neutral") is None:
            issues.append("neutral missing")
        else:
            neutral = _optional_int(limit_entry.get("neutral"))
            if neutral is None:
                issues.append("neutral must be an integer")

        if "margin_ticks" not in limit_entry or limit_entry.get("margin_ticks") is None:
            issues.append("margin_ticks missing")
        else:
            margin = _optional_int(limit_entry.get("margin_ticks"))
            if margin is None:
                issues.append("margin_ticks must be an integer")
            elif margin < 0:
                issues.append("margin_ticks must be non-negative")

    if min_value is not None and max_value is not None:
        span = max_value - min_value
        if min_value >= max_value:
            issues.append("provisional_min must be less than provisional_max")
        if span < 50:
            issues.append("span must be at least 50 ticks")
    else:
        span = None

    if min_value is not None and max_value is not None and neutral is not None:
        if neutral < min_value or neutral > max_value:
            issues.append("neutral must lie between provisional_min and provisional_max")

    sign = None
    if not isinstance(direction_entry, dict):
        issues.append("joint_directions.yaml entry must be a mapping")
    else:
        if "sign" not in direction_entry or direction_entry.get("sign") is None:
            issues.append("direction sign missing")
        else:
            try:
                sign = int(direction_entry.get("sign"))
            except (TypeError, ValueError):
                issues.append("direction sign must be 1 or -1")
            else:
                if sign not in (1, -1):
                    issues.append("direction sign must be 1 or -1")

        positive_description = direction_entry.get("positive_description")
        if positive_description is None or str(positive_description).strip() == "":
            issues.append("positive_description must be non-empty")

    live_position = None
    if live_requested:
        if not live_backend_available:
            warnings.append("live read unavailable: {}".format(live_backend_error))
        elif bus is not None and servo_id is not None:
            try:
                live_position = bus.read_position(servo_id)
            except Exception as exc:
                warnings.append("live read unavailable: {}".format(exc))
            else:
                if live_position is None:
                    warnings.append("live read unavailable")
                elif min_value is not None and max_value is not None:
                    if live_position < min_value or live_position > max_value:
                        issues.append(
                            "live position {} outside [{}, {}]".format(live_position, min_value, max_value)
                        )

    status, reason = _status_and_reason(issues, warnings, live_position, live_requested)
    return {
        "joint": joint_name,
        "id": _string_or_dash(servo_id),
        "min": _string_or_dash(min_value),
        "max": _string_or_dash(max_value),
        "neutral": _string_or_dash(neutral),
        "span": _string_or_dash(span),
        "direction": _string_or_dash(sign),
        "live_position": _string_or_dash(live_position),
        "status": status,
        "reason": reason,
        "calibrated": status == "PASS",
    }


def _render_table(rows):
    headers = [
        "joint",
        "id",
        "min",
        "max",
        "neutral",
        "span",
        "direction",
        "live_position",
        "status",
        "reason",
    ]
    widths = dict((header, len(header)) for header in headers)
    for row in rows:
        for header in headers:
            widths[header] = max(widths[header], len(str(row[header])))

    header_line = (
        "{joint:<{joint_w}}  {id:>{id_w}}  {min:>{min_w}}  {max:>{max_w}}  "
        "{neutral:>{neutral_w}}  {span:>{span_w}}  {direction:>{direction_w}}  "
        "{live_position:>{live_w}}  {status:<{status_w}}  {reason}"
    ).format(
        joint="joint",
        id="id",
        min="min",
        max="max",
        neutral="neutral",
        span="span",
        direction="direction",
        live_position="live_position",
        status="status",
        reason="reason",
        joint_w=widths["joint"],
        id_w=widths["id"],
        min_w=widths["min"],
        max_w=widths["max"],
        neutral_w=widths["neutral"],
        span_w=widths["span"],
        direction_w=widths["direction"],
        live_w=widths["live_position"],
        status_w=widths["status"],
    )
    separator_line = (
        "{joint:<{joint_w}}  {id:>{id_w}}  {min:>{min_w}}  {max:>{max_w}}  "
        "{neutral:>{neutral_w}}  {span:>{span_w}}  {direction:>{direction_w}}  "
        "{live_position:>{live_w}}  {status:<{status_w}}  {reason}"
    ).format(
        joint="-" * widths["joint"],
        id="-" * widths["id"],
        min="-" * widths["min"],
        max="-" * widths["max"],
        neutral="-" * widths["neutral"],
        span="-" * widths["span"],
        direction="-" * widths["direction"],
        live_position="-" * widths["live_position"],
        status="-" * widths["status"],
        reason="-" * widths["reason"],
        joint_w=widths["joint"],
        id_w=widths["id"],
        min_w=widths["min"],
        max_w=widths["max"],
        neutral_w=widths["neutral"],
        span_w=widths["span"],
        direction_w=widths["direction"],
        live_w=widths["live_position"],
        status_w=widths["status"],
    )
    print(header_line)
    print(separator_line)
    for row in rows:
        print(
            "{joint:<{joint_w}}  {id:>{id_w}}  {min:>{min_w}}  {max:>{max_w}}  "
            "{neutral:>{neutral_w}}  {span:>{span_w}}  {direction:>{direction_w}}  "
            "{live_position:>{live_w}}  {status:<{status_w}}  {reason}"
            .format(
                joint=row["joint"],
                id=row["id"],
                min=row["min"],
                max=row["max"],
                neutral=row["neutral"],
                span=row["span"],
                direction=row["direction"],
                live_position=row["live_position"],
                status=row["status"],
                reason=row["reason"],
                joint_w=widths["joint"],
                id_w=widths["id"],
                min_w=widths["min"],
                max_w=widths["max"],
                neutral_w=widths["neutral"],
                span_w=widths["span"],
                direction_w=widths["direction"],
                live_w=widths["live_position"],
                status_w=widths["status"],
            )
        )


def _selected_joints(args, servo_map):
    if args.joint is not None:
        return [_resolve_joint_name(args.joint, servo_map)]
    return list(CANONICAL_JOINTS)


def _write_status_if_requested(args, rows, limits_data):
    if not args.write_status:
        return
    limits = limits_data.get("limits") or {}
    for row in rows:
        joint_name = row["joint"]
        entry = dict(limits.get(joint_name) or {})
        entry["calibrated"] = bool(row["status"] == "PASS")
        limits[joint_name] = entry
    limits_data["limits"] = limits
    _write_limits_yaml(args.limits_path, limits_data)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Validate recorded joint limits and direction signs without commanding movement."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None)
    parser.add_argument("--joint", default=None, help="Validate a single joint name or servo_map alias.")
    parser.add_argument("--write-status", action="store_true", help="Update calibrated flags in joint_limits.yaml.")
    parser.add_argument("--servo-map", dest="servo_map_path", default=DEFAULT_SERVO_MAP_PATH)
    parser.add_argument("--joint-limits", dest="limits_path", default=DEFAULT_LIMITS_PATH)
    parser.add_argument("--joint-directions", dest="directions_path", default=DEFAULT_DIRECTIONS_PATH)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    bus = None
    try:
        servo_map = _load_servo_map(args.servo_map_path)
        limits_data = _load_limits(args.limits_path)
        directions_data = _load_directions(args.directions_path)

        live_requested = args.backend is not None
        live_backend_available = False
        live_backend_error = ""
        validation_logger = ServoEventLogger(DEFAULT_LOG_PATH)

        if live_requested:
            try:
                config = load_robot_config(args.config)
                bus = build_servo_bus(
                    config,
                    args.config,
                    dry_run=(args.backend == "mock"),
                    backend_name=args.backend,
                )
                live_backend_available = True
            except Exception as exc:
                live_backend_error = str(exc)
                validation_logger.log(
                    "joint_calibration_backend",
                    backend=args.backend,
                    status="unavailable",
                    error=live_backend_error,
                )

        rows = []
        pass_count = 0
        warn_count = 0
        fail_count = 0

        for joint_name in _selected_joints(args, servo_map):
            row = _validate_joint(
                joint_name,
                servo_map,
                limits_data,
                directions_data,
                live_requested,
                bus,
                live_backend_available,
                live_backend_error,
            )
            rows.append(row)
            if row["status"] == "PASS":
                pass_count += 1
            elif row["status"] == "WARN":
                warn_count += 1
            else:
                fail_count += 1

            validation_logger.log(
                "joint_calibration_validation",
                joint=joint_name,
                servo_id=None if row["id"] == "-" else int(row["id"]),
                status=row["status"],
                reason=row["reason"],
                min=None if row["min"] == "-" else int(row["min"]),
                max=None if row["max"] == "-" else int(row["max"]),
                neutral=None if row["neutral"] == "-" else int(row["neutral"]),
                span=None if row["span"] == "-" else int(row["span"]),
                direction=None if row["direction"] == "-" else int(row["direction"]),
                live_position=None if row["live_position"] == "-" else int(row["live_position"]),
                write_status=bool(args.write_status),
                calibrated=bool(row["calibrated"]),
            )

        _render_table(rows)
        print(
            "Summary: {} PASS, {} WARN, {} FAIL{}".format(
                pass_count,
                warn_count,
                fail_count,
                " (backend unavailable)" if live_requested and not live_backend_available else "",
            )
        )

        if args.write_status:
            _write_status_if_requested(args, rows, limits_data)
            print("Updated calibrated flags in: {}".format(args.limits_path))

        validation_logger.log(
            "joint_calibration_validation_complete",
            backend=args.backend or "none",
            write_status=bool(args.write_status),
            pass_count=pass_count,
            warn_count=warn_count,
            fail_count=fail_count,
            backend_available=live_backend_available,
            joint_count=len(rows),
        )
    except (ServoBusError, safety.SafetyError) as exc:
        parser.exit(2, "error: {}\n".format(exc))
    finally:
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
