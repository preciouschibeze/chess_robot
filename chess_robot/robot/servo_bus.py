"""Read-only servo bus foundation for discovery and position snapshots.

Only this module may communicate directly with the servo bus. The default path
is a dry-run mock backend. The Feetech backend is isolated here and intentionally
limited to ping/read operations for the calibration foundation.
"""

import datetime
import importlib
import json
import os
import select
import termios
import time
from typing import Any, Dict, Iterable, List, Optional

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.robot import safety


DEFAULT_CONFIG_PATH = os.path.join("configs", "robot.yaml")
DEFAULT_LOG_PATH = os.path.join("data", "logs", "servo.log")


class ServoBusError(RuntimeError):
    """Base error for servo bus operations."""


class BackendUnavailable(ServoBusError):
    """Raised when a configured backend cannot be used."""


class ServoBackend(object):
    """Minimal read-only servo backend interface."""

    name = "base"

    def ping(self, servo_id: int) -> bool:
        raise NotImplementedError

    def read_position(self, servo_id: int) -> Optional[int]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class MockServoBackend(ServoBackend):
    """Deterministic dry-run backend for tests and CLI development."""

    name = "mock"

    def __init__(self, servo_ids: Optional[Iterable[int]] = None,
                 positions: Optional[Dict[Any, Any]] = None) -> None:
        self._servo_ids = set(safety.validate_servo_ids(servo_ids or []))
        self._positions = {}
        for key, value in (positions or {}).items():
            servo_id = safety.validate_servo_id(key)
            self._servo_ids.add(servo_id)
            self._positions[servo_id] = int(value)

    def ping(self, servo_id: int) -> bool:
        return safety.validate_servo_id(servo_id) in self._servo_ids

    def read_position(self, servo_id: int) -> Optional[int]:
        servo_id = safety.validate_servo_id(servo_id)
        if servo_id not in self._servo_ids:
            return None
        return self._positions.get(servo_id)


class FeetechServoBackend(ServoBackend):
    """SO-101/Feetech read-only backend.

    The preferred path uses scservo_sdk when it is installed. The Nano can also
    use the built-in raw serial transport, which implements only Feetech read
    packets for Model_Number and Present_Position. No write packets exist here.
    """

    name = "feetech"

    MODEL_NUMBER_ADDRESS = 3
    MODEL_NUMBER_LENGTH = 2
    READ_INSTRUCTION = 0x02

    def __init__(self, port: Optional[str], baudrate: Optional[int],
                 timeout_seconds: float = 0.1,
                 sdk: str = "scservo_sdk",
                 protocol_version: int = 0,
                 position_address: int = 56,
                 position_length: int = 2,
                 transport: str = "raw_serial") -> None:
        if not port:
            raise BackendUnavailable("servo_bus.feetech.port is not configured.")
        if baudrate is None:
            raise BackendUnavailable("servo_bus.feetech.baudrate is not configured.")

        try:
            self._baudrate = int(baudrate)
            self._protocol_version = int(protocol_version)
            self._position_address = int(position_address)
            self._position_length = int(position_length)
            self._timeout_seconds = float(timeout_seconds)
        except (TypeError, ValueError):
            raise BackendUnavailable("Feetech baudrate/protocol/address values must be numeric.")

        self._port = port
        self._transport = transport
        self._fd = None
        self._port_handler = None
        self._packet_handler = None
        self._comm_success = 0
        self._no_error = 0

        if transport == "scservo_sdk":
            self._open_scservo_sdk(sdk)
        elif transport == "raw_serial":
            self._open_raw_serial()
        else:
            raise BackendUnavailable("Unknown Feetech transport {!r}.".format(transport))

    def _open_scservo_sdk(self, sdk: str) -> None:
        try:
            self._sdk = importlib.import_module(sdk)
        except ImportError as exc:
            raise BackendUnavailable(
                "Feetech SDK {!r} is not installed: {}".format(sdk, exc)
            )

        port_handler_class = getattr(self._sdk, "PortHandler", None)
        packet_handler_factory = getattr(self._sdk, "PacketHandler", None)
        if port_handler_class is None or packet_handler_factory is None:
            raise BackendUnavailable(
                "Feetech SDK {!r} does not expose PortHandler/PacketHandler.".format(sdk)
            )

        self._port_handler = port_handler_class(self._port)
        self._packet_handler = packet_handler_factory(self._protocol_version)
        self._comm_success = getattr(self._sdk, "COMM_SUCCESS", 0)
        if not self._port_handler.openPort():
            raise BackendUnavailable("Could not open servo bus port {!r}.".format(self._port))
        if not self._port_handler.setBaudRate(self._baudrate):
            self.close()
            raise BackendUnavailable("Could not set servo bus baudrate {}.".format(self._baudrate))
        if hasattr(self._port_handler, "setPacketTimeoutMillis"):
            self._port_handler.setPacketTimeoutMillis(int(self._timeout_seconds * 1000))

    def _open_raw_serial(self) -> None:
        baud_const_name = "B{}".format(self._baudrate)
        baud_const = getattr(termios, baud_const_name, None)
        if baud_const is None:
            raise BackendUnavailable("termios does not support baudrate {}.".format(self._baudrate))

        try:
            self._fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise BackendUnavailable("Could not open servo bus port {!r}: {}".format(self._port, exc))

        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = baud_const | termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    def ping(self, servo_id: int) -> bool:
        servo_id = safety.validate_servo_id(servo_id)
        if self._transport == "scservo_sdk":
            result = self._packet_handler.ping(self._port_handler, servo_id)
            model_number, comm, error = _normalize_packet_result(result, expected_values=3)
            return model_number is not None and comm == self._comm_success and error == self._no_error
        return self._read_raw_register(
            servo_id, self.MODEL_NUMBER_ADDRESS, self.MODEL_NUMBER_LENGTH
        ) is not None

    def read_position(self, servo_id: int) -> Optional[int]:
        servo_id = safety.validate_servo_id(servo_id)
        if self._position_length != 2:
            raise BackendUnavailable("Only 2-byte Present_Position reads are currently supported.")
        if self._transport == "scservo_sdk":
            read_fn = getattr(self._packet_handler, "read2ByteTxRx", None)
            if read_fn is None:
                raise BackendUnavailable("Configured Feetech SDK has no read2ByteTxRx method.")
            result = read_fn(self._port_handler, servo_id, self._position_address)
            value, comm, error = _normalize_packet_result(result, expected_values=3)
            if comm != self._comm_success or error != self._no_error:
                return None
            return int(value)
        return self._read_raw_register(servo_id, self._position_address, self._position_length)

    def _read_raw_register(self, servo_id: int, address: int, length: int) -> Optional[int]:
        packet = self._build_read_packet(servo_id, address, length)
        termios.tcflush(self._fd, termios.TCIOFLUSH)
        self._write_all(packet)
        response = self._read_response(expected_length=6 + length)
        if response is None:
            return None
        params = response[5:5 + length]
        value = 0
        for shift, byte in enumerate(params):
            value |= byte << (8 * shift)
        return value

    def _build_read_packet(self, servo_id: int, address: int, length: int) -> bytes:
        body = [servo_id, 4, self.READ_INSTRUCTION, address, length]
        checksum = (~sum(body)) & 0xFF
        return bytes([0xFF, 0xFF] + body + [checksum])

    def _read_response(self, expected_length: int) -> Optional[bytes]:
        deadline = time.time() + self._timeout_seconds
        data = bytearray()
        while time.time() < deadline and len(data) < expected_length:
            remaining = max(0.0, deadline - time.time())
            ready, _, _ = select.select([self._fd], [], [], remaining)
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, expected_length - len(data))
            except BlockingIOError:
                continue
            if chunk:
                data.extend(chunk)

        header = bytes([0xFF, 0xFF])
        start = bytes(data).find(header)
        if start < 0:
            return None
        packet = bytes(data[start:])
        if len(packet) < expected_length:
            return None
        packet = packet[:expected_length]
        if self._checksum(packet[2:-1]) != packet[-1]:
            return None
        if packet[4] != 0:
            return None
        return packet

    def _write_all(self, packet: bytes) -> None:
        view = memoryview(packet)
        while view:
            written = os.write(self._fd, view)
            view = view[written:]

    def _checksum(self, values: bytes) -> int:
        return (~sum(values)) & 0xFF

    def close(self) -> None:
        close_port = getattr(getattr(self, "_port_handler", None), "closePort", None)
        if close_port is not None:
            close_port()
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


def _normalize_packet_result(result: Any, expected_values: int) -> Any:
    if isinstance(result, (list, tuple)):
        if len(result) >= expected_values:
            return tuple(result[:expected_values])
        if len(result) == 2 and expected_values == 3:
            return result[0], result[1], 0
    raise BackendUnavailable("Unexpected Feetech SDK response: {!r}".format(result))


class ServoEventLogger(object):
    """Append JSONL servo events to data/logs/servo.log."""

    def __init__(self, path: str) -> None:
        self.path = path
        log_dir = os.path.dirname(os.path.abspath(path))
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

    def log(self, action: str, **fields: Any) -> None:
        event = {
            "timestamp_utc": datetime.datetime.utcnow().isoformat() + "Z",
            "action": action,
        }
        event.update(fields)
        with open(self.path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, sort_keys=True) + "\n")


class ServoBus(object):
    """Safe read-only facade used by calibration tools."""

    def __init__(self, backend: ServoBackend, logger: ServoEventLogger,
                 dry_run: bool) -> None:
        self.backend = backend
        self.logger = logger
        self.dry_run = bool(dry_run)

    def ping(self, servo_id: int) -> bool:
        servo_id = safety.validate_servo_id(servo_id)
        try:
            present = self.backend.ping(servo_id)
        except Exception as exc:
            self.logger.log(
                "servo_ping",
                servo_id=servo_id,
                backend=self.backend.name,
                dry_run=self.dry_run,
                status="error",
                error=str(exc),
            )
            raise
        self.logger.log(
            "servo_ping",
            servo_id=servo_id,
            backend=self.backend.name,
            dry_run=self.dry_run,
            status="ok",
            present=bool(present),
        )
        return bool(present)

    def read_position(self, servo_id: int) -> Optional[int]:
        servo_id = safety.validate_servo_id(servo_id)
        try:
            position = self.backend.read_position(servo_id)
        except Exception as exc:
            self.logger.log(
                "servo_read_position",
                servo_id=servo_id,
                backend=self.backend.name,
                dry_run=self.dry_run,
                status="error",
                error=str(exc),
            )
            raise
        self.logger.log(
            "servo_read_position",
            servo_id=servo_id,
            backend=self.backend.name,
            dry_run=self.dry_run,
            status="ok",
            position=position,
        )
        return position

    def close(self) -> None:
        self.backend.close()


def load_robot_config(path: str = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    if yaml is None:
        raise ServoBusError("PyYAML is required to read robot configuration.")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("Robot config must be a YAML mapping.")
    return data


def configured_joint_servo_ids(config: Dict[str, Any]) -> List[int]:
    joints = config.get("joints") or {}
    ids = []
    if isinstance(joints, dict):
        for joint in joints.values():
            if isinstance(joint, dict) and joint.get("servo_id") is not None:
                ids.append(joint.get("servo_id"))
    return safety.validate_servo_ids(ids)


def configured_mock_ids(config: Dict[str, Any]) -> List[int]:
    servo_config = config.get("servo_bus") or {}
    mock_config = servo_config.get("mock") or {}
    return safety.validate_servo_ids(mock_config.get("servo_ids") or [])


def resolve_log_path(config: Dict[str, Any], config_path: str) -> str:
    servo_config = config.get("servo_bus") or {}
    log_path = servo_config.get("log_path") or DEFAULT_LOG_PATH
    if os.path.isabs(log_path):
        return log_path
    config_dir = os.path.dirname(os.path.abspath(config_path))
    if os.path.basename(config_dir) == "configs":
        project_root = os.path.dirname(config_dir)
    else:
        project_root = os.getcwd()
    return os.path.join(project_root, log_path)


def build_servo_bus(config: Dict[str, Any], config_path: str,
                    dry_run: Optional[bool] = None,
                    backend_name: Optional[str] = None,
                    mock_ids: Optional[Iterable[int]] = None) -> ServoBus:
    servo_config = config.get("servo_bus") or {}
    if dry_run is None:
        dry_run = bool(servo_config.get("dry_run_default", True))
    backend_name = backend_name or servo_config.get("backend") or "mock"

    log_path = resolve_log_path(config, config_path)
    logger = ServoEventLogger(log_path)

    try:
        if dry_run or backend_name == "mock":
            mock_config = servo_config.get("mock") or {}
            ids = list(configured_mock_ids(config))
            if mock_ids is not None:
                ids = safety.validate_servo_ids(mock_ids)
            backend = MockServoBackend(ids, mock_config.get("positions") or {})
        elif backend_name == "feetech":
            feetech_config = servo_config.get("feetech") or {}
            backend = FeetechServoBackend(
                port=feetech_config.get("port"),
                baudrate=feetech_config.get("baudrate"),
                timeout_seconds=feetech_config.get("timeout_seconds", 0.1),
                sdk=feetech_config.get("sdk", "scservo_sdk"),
                protocol_version=feetech_config.get("protocol_version", 0),
                position_address=feetech_config.get("position_address", 56),
                position_length=feetech_config.get("position_length", 2),
                transport=feetech_config.get("transport", "raw_serial"),
            )
        else:
            raise BackendUnavailable("Unknown servo backend {!r}.".format(backend_name))
    except Exception as exc:
        logger.log(
            "servo_backend_open",
            backend=backend_name,
            dry_run=bool(dry_run),
            status="error",
            error=str(exc),
        )
        raise

    logger.log(
        "servo_backend_open",
        backend=backend.name,
        dry_run=bool(dry_run),
        status="ok",
    )
    return ServoBus(backend=backend, logger=logger, dry_run=dry_run)
