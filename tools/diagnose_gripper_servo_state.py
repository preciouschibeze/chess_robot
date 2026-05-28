#!/usr/bin/env python3
"""Read and safely diagnose gripper servo ID 6 state.

Default mode is read-only. The only supported write is optional ID 6
Torque_Enable=0 via --disable-torque.
"""

from __future__ import print_function

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.servo_bus import (  # noqa: E402
    BackendUnavailable,
    ServoBusError,
    build_servo_bus,
    load_robot_config,
)

GRIPPER_JOINT = "gripper"
GRIPPER_SERVO_ID = 6

SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
GRIPPER_PROFILE_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "gripper_profile.yaml")

REG_PRESENT_POSITION = (56, 2)
REG_GOAL_POSITION = (42, 2)
REG_OPERATING_MODE = (33, 1)
REG_TORQUE_ENABLE = (40, 1)
REG_MIN_ANGLE_LIMIT = (9, 2)
REG_MAX_ANGLE_LIMIT = (11, 2)
REG_MOVING = (66, 1)
REG_HARDWARE_ERROR_STATUS = (65, 1)


class DiagnosticError(RuntimeError):
    pass


def build_parser():
    parser = argparse.ArgumentParser(
        description="Diagnose gripper servo ID 6 state. Default is read-only."
    )
    parser.add_argument(
        "--config",
        default=os.path.join(ROOT, "configs", "robot.yaml"),
        help="Path to robot YAML config. Default: configs/robot.yaml",
    )
    parser.add_argument(
        "--backend",
        choices=("mock", "feetech"),
        default="mock",
        help="Backend to use. Default: mock. Use feetech for live read-only diagnosis.",
    )
    parser.add_argument(
        "--disable-torque",
        action="store_true",
        help="Write Torque_Enable=0 for ID 6 only, then verify readback.",
    )
    parser.add_argument(
        "--verify-duration",
        type=float,
        default=3.0,
        help="Seconds to keep checking torque after --disable-torque. Default: 3.0.",
    )
    return parser


def _read_yaml(path):
    if yaml is None:
        raise DiagnosticError("PyYAML is required to read calibration files.")
    if not os.path.exists(path):
        raise DiagnosticError("Required calibration file is missing: {}".format(path))
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise DiagnosticError("YAML file must contain a mapping: {}".format(path))
    return data


def _to_int(value, label):
    if isinstance(value, bool):
        raise DiagnosticError("{} must be an integer.".format(label))
    try:
        return int(value)
    except (TypeError, ValueError):
        raise DiagnosticError("{} must be an integer.".format(label))


def load_gripper_calibration(servo_map_path=SERVO_MAP_PATH,
                             joint_limits_path=JOINT_LIMITS_PATH,
                             gripper_profile_path=GRIPPER_PROFILE_PATH):
    servo_map = _read_yaml(servo_map_path)
    servo_entry = ((servo_map.get("joints") or {}).get(GRIPPER_JOINT) or {})
    servo_id = _to_int(servo_entry.get("id"), "servo_map gripper id")
    if servo_id != GRIPPER_SERVO_ID:
        raise DiagnosticError(
            "Refusing: servo_map gripper id is {}, expected {}.".format(
                servo_id, GRIPPER_SERVO_ID
            )
        )

    joint_limits = _read_yaml(joint_limits_path)
    joint_entry = ((joint_limits.get("limits") or {}).get(GRIPPER_JOINT) or {})
    joint_min = _to_int(joint_entry.get("provisional_min"), "joint_limits gripper provisional_min")
    joint_max = _to_int(joint_entry.get("provisional_max"), "joint_limits gripper provisional_max")

    profile_data = _read_yaml(gripper_profile_path)
    profile = profile_data.get("gripper") or {}
    profile_servo_id = _to_int(profile.get("servo_id"), "gripper_profile servo_id")
    if profile_servo_id != GRIPPER_SERVO_ID:
        raise DiagnosticError(
            "Refusing: gripper_profile servo_id is {}, expected {}.".format(
                profile_servo_id, GRIPPER_SERVO_ID
            )
        )
    profile_limits = profile.get("limits") or {}
    active_min = _to_int(profile_limits.get("min", joint_min), "gripper active min")
    active_max = _to_int(profile_limits.get("max", joint_max), "gripper active max")
    return {
        "servo_id": servo_id,
        "joint_min": joint_min,
        "joint_max": joint_max,
        "active_min": active_min,
        "active_max": active_max,
        "joint_limits_entry": joint_entry,
        "profile": profile,
    }


def format_range(minimum, maximum):
    return "{}..{}".format(minimum, maximum)


def serial_port_from_config(config):
    servo_config = config.get("servo_bus") or {}
    feetech_config = servo_config.get("feetech") or {}
    return feetech_config.get("port")


def _run_command(args):
    try:
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = proc.communicate()
    except OSError as exc:
        return 127, "", str(exc)
    return proc.returncode, out.decode("utf-8", "replace"), err.decode("utf-8", "replace")


def parse_python_robot_processes(ps_output, current_pid):
    matches = []
    current_pid = int(current_pid)
    keywords = ("chess_robot", "servo", "gripper", "teleop", "calibrate", "test_")
    for line in (ps_output or "").splitlines():
        stripped = line.strip()
        if not stripped or stripped.upper().startswith("PID "):
            continue
        parts = stripped.split(None, 1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        command = parts[1]
        executable = os.path.basename(command.split(None, 1)[0]).lower()
        lowered = command.lower()
        if pid == current_pid:
            continue
        if "python" not in executable:
            continue
        if any(keyword in lowered for keyword in keywords):
            matches.append({"pid": pid, "command": command})
    return matches


def inspect_competing_processes(serial_port, current_pid=None, command_runner=None):
    current_pid = int(current_pid if current_pid is not None else os.getpid())
    command_runner = command_runner or _run_command
    paths = []
    if serial_port:
        paths.append(serial_port)
    for path in glob.glob("/dev/serial/by-id/*"):
        if path not in paths:
            paths.append(path)

    checks = []
    for path in paths:
        if shutil.which("fuser"):
            code, out, err = command_runner(["fuser", "-v", path])
            checks.append({"tool": "fuser", "path": path, "returncode": code, "stdout": out, "stderr": err})
        if shutil.which("lsof"):
            code, out, err = command_runner(["lsof", path])
            checks.append({"tool": "lsof", "path": path, "returncode": code, "stdout": out, "stderr": err})

    ps_code, ps_out, ps_err = command_runner(["ps", "-eo", "pid,args"])
    python_matches = parse_python_robot_processes(ps_out, current_pid)
    return {
        "current_pid": current_pid,
        "serial_paths": paths,
        "device_checks": checks,
        "ps_returncode": ps_code,
        "ps_stderr": ps_err,
        "python_robot_processes": python_matches,
    }


def _read_optional_register(bus, label, spec):
    address, length = spec
    try:
        return bus.read_register(GRIPPER_SERVO_ID, address, length), None
    except Exception as exc:
        return None, str(exc)


def read_servo_state(bus):
    specs = [
        ("present_position", REG_PRESENT_POSITION),
        ("goal_position", REG_GOAL_POSITION),
        ("torque_state", REG_TORQUE_ENABLE),
        ("operating_mode", REG_OPERATING_MODE),
        ("moving_flag", REG_MOVING),
        ("hardware_error", REG_HARDWARE_ERROR_STATUS),
        ("eeprom_min_angle_limit", REG_MIN_ANGLE_LIMIT),
        ("eeprom_max_angle_limit", REG_MAX_ANGLE_LIMIT),
    ]
    state = {}
    errors = {}
    for label, spec in specs:
        value, error = _read_optional_register(bus, label, spec)
        state[label] = value
        if error:
            errors[label] = error
    return state, errors


def _print_competing_processes(info):
    print("Current process PID: {}".format(info.get("current_pid")))
    print("Serial paths checked: {}".format(", ".join(info.get("serial_paths") or []) or "none"))
    checks = info.get("device_checks") or []
    if not checks:
        print("Device user checks: fuser/lsof unavailable or no serial paths found")
    for check in checks:
        text = (check.get("stdout") or check.get("stderr") or "").strip()
        if not text:
            text = "no output"
        print("{} {} rc={}: {}".format(
            check.get("tool"), check.get("path"), check.get("returncode"), text
        ))
    matches = info.get("python_robot_processes") or []
    if matches:
        print("WARNING: possible competing Python robot/servo processes:")
        for match in matches:
            print("  pid={} command={}".format(match.get("pid"), match.get("command")))
    else:
        print("Possible competing Python robot/servo processes: none detected")


def _print_state(state, errors, calibration, serial_port, backend_name, process_info):
    print("Backend: {}".format(backend_name))
    print("Serial port path: {}".format(serial_port or "n/a"))
    print("Servo ID: {} ({})".format(GRIPPER_SERVO_ID, GRIPPER_JOINT))
    for key in (
            "present_position", "goal_position", "torque_state", "operating_mode",
            "moving_flag", "hardware_error", "eeprom_min_angle_limit",
            "eeprom_max_angle_limit"):
        suffix = ""
        if key in errors:
            suffix = " error={}".format(errors[key])
        print("{}: {}{}".format(key, state.get(key), suffix))
    print("Active software limits: {}".format(
        format_range(calibration["active_min"], calibration["active_max"])
    ))
    print("Joint-limit source range: {}".format(
        format_range(calibration["joint_min"], calibration["joint_max"])
    ))
    profile = calibration.get("profile") or {}
    print("Gripper profile values:")
    for key in (
            "active_profile_valid", "direction_sign", "grasp_position",
            "pre_grasp_position", "neutral_position", "release_position",
            "open_position"):
        print("  {}: {}".format(key, profile.get(key)))
    _print_competing_processes(process_info)


def torque_verify_delays(verify_duration):
    duration = max(0.0, float(verify_duration))
    delays = [0.0]
    for candidate in (0.5, 2.0, duration):
        if candidate <= duration and candidate not in delays:
            delays.append(candidate)
    return delays


def disable_torque_and_verify(bus, verify_duration):
    print("write_step: disable_torque intended_register: Torque_Enable intended_value: 0 servo_id: {}".format(
        GRIPPER_SERVO_ID
    ))
    bus.torque_enable(GRIPPER_SERVO_ID, False)
    start = time.time()
    readings = []
    for delay in torque_verify_delays(verify_duration):
        sleep_for = start + delay - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        value = bus.read_register(GRIPPER_SERVO_ID, REG_TORQUE_ENABLE[0], REG_TORQUE_ENABLE[1])
        readings.append({"elapsed_seconds": delay, "torque_state": value})
        print("Torque readback at {:.3f}s: {}".format(delay, value))
    for reading in readings:
        if int(reading["torque_state"]) != 0:
            raise DiagnosticError(
                "Torque became enabled during verification: {}".format(readings)
            )
    return readings


def main():
    args = build_parser().parse_args()
    if float(args.verify_duration) < 0:
        raise SystemExit("ERROR: --verify-duration must be non-negative.")

    config = load_robot_config(args.config)
    calibration = load_gripper_calibration()
    serial_port = serial_port_from_config(config)
    process_info = inspect_competing_processes(serial_port)

    dry_run = args.backend != "feetech"
    if args.disable_torque and args.backend != "feetech":
        raise SystemExit("ERROR: --disable-torque requires --backend feetech.")

    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=dry_run,
        backend_name=args.backend,
        mock_ids=[GRIPPER_SERVO_ID],
    )
    try:
        state, errors = read_servo_state(bus)
        _print_state(state, errors, calibration, serial_port, args.backend, process_info)
        if args.disable_torque:
            disable_torque_and_verify(bus, args.verify_duration)
    finally:
        bus.close()


if __name__ == "__main__":
    try:
        main()
    except (BackendUnavailable, ServoBusError, DiagnosticError, OSError, ValueError) as exc:
        print("ERROR: {}".format(exc))
        raise SystemExit(1)
