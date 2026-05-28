from __future__ import absolute_import

import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.diagnose_gripper_servo_state as diagnose
import tools.test_gripper_monitored_motion as monitored


class FakeLogger(object):
    def __init__(self):
        self.events = []

    def log(self, action, **fields):
        event = {"action": action}
        event.update(fields)
        self.events.append(event)


class FakeBus(object):
    def __init__(self, torque_reads=None):
        self.logger = FakeLogger()
        self.torque_calls = []
        self.register_reads = []
        self.torque_reads = list(torque_reads if torque_reads is not None else [0])

    def torque_enable(self, servo_id, enabled):
        self.torque_calls.append((int(servo_id), bool(enabled)))

    def read_register(self, servo_id, address, length):
        self.register_reads.append((int(servo_id), int(address), int(length)))
        if int(address) == monitored.REG_TORQUE_ENABLE[0]:
            if len(self.torque_reads) > 1:
                return self.torque_reads.pop(0)
            return self.torque_reads[0]
        return 0


def _write_yaml(tmpdir, name, data):
    path = str(tmpdir.join(name))
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)
    return path


def test_diagnostic_loads_gripper_profile_and_limits(tmpdir):
    servo_map = {"joints": {"gripper": {"id": 6}}}
    joint_limits = {"limits": {"gripper": {"provisional_min": 1463, "provisional_max": 1738}}}
    profile = {"gripper": {"servo_id": 6, "limits": {"min": 1463, "max": 1738}, "open_position": 1704}}
    calibration = diagnose.load_gripper_calibration(
        _write_yaml(tmpdir, "servo_map.yaml", servo_map),
        _write_yaml(tmpdir, "joint_limits.yaml", joint_limits),
        _write_yaml(tmpdir, "gripper_profile.yaml", profile),
    )
    assert calibration["servo_id"] == 6
    assert calibration["active_min"] == 1463
    assert calibration["active_max"] == 1738
    assert calibration["profile"]["open_position"] == 1704


def test_parse_python_robot_processes_excludes_current_pid():
    output = """  PID COMMAND
  100 python3 tools/test_gripper_monitored_motion.py --real
  101 python3 unrelated.py
  102 bash
"""
    matches = diagnose.parse_python_robot_processes(output, current_pid=101)
    assert matches == [{"pid": 100, "command": "python3 tools/test_gripper_monitored_motion.py --real"}]


def test_inspect_competing_processes_uses_mocked_ps(monkeypatch):
    monkeypatch.setattr(diagnose.shutil, "which", lambda name: None)

    def runner(args):
        if args[:2] == ["ps", "-eo"]:
            return 0, "  PID COMMAND\n  200 python3 tools/teleop_joints_keyboard.py --execute\n", ""
        return 1, "", ""

    info = diagnose.inspect_competing_processes(
        "/dev/ttyUSB0", current_pid=123, command_runner=runner
    )
    assert info["serial_paths"][0] == "/dev/ttyUSB0"
    assert info["python_robot_processes"][0]["pid"] == 200


def test_torque_disable_verify_delays_include_intermediate_reads():
    assert diagnose.torque_verify_delays(3.0) == [0.0, 0.5, 2.0, 3.0]


def test_monitored_refuses_when_initial_torque_enabled_and_required():
    args = type("Args", (object,), {
        "require_initial_torque_disabled": True,
        "disable_before_start": False,
    })()
    bus = FakeBus(torque_reads=[1])
    registers = {"torque": 1}
    with pytest.raises(monitored.GripperMotionError):
        monitored._handle_initial_torque_state(args, bus, registers)
    assert bus.torque_calls == []


def test_monitored_disable_before_start_writes_and_verifies():
    args = type("Args", (object,), {
        "require_initial_torque_disabled": False,
        "disable_before_start": True,
    })()
    bus = FakeBus(torque_reads=[0])
    registers = {"torque": 1}
    monitored._handle_initial_torque_state(args, bus, registers)
    assert bus.torque_calls == [(6, False)]
    assert registers["torque"] == 0
    assert bus.register_reads[-1] == (6, 40, 1)


def test_monitored_torque_write_fails_when_readback_stays_enabled():
    bus = FakeBus(torque_reads=[1])
    with pytest.raises(monitored.GripperMotionError):
        monitored._write_torque_with_readback(
            bus, "disable_torque", False, allow_verified_continue=True
        )
    assert bus.torque_calls == [(6, False)]
