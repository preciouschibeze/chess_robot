"""Safety checks for robot hardware access.

This module contains validation and confirmation gates. It does not talk to
hardware; direct servo bus access belongs only in chess_robot.robot.servo_bus.
"""

from typing import Iterable, List


SERVO_ID_MIN = 0
SERVO_ID_MAX = 253


class SafetyError(ValueError):
    """Raised when a requested hardware operation is not safe to attempt."""


def validate_servo_id(servo_id: int) -> int:
    """Return a normalized servo ID or raise SafetyError."""
    try:
        value = int(servo_id)
    except (TypeError, ValueError):
        raise SafetyError("Servo ID must be an integer: {!r}".format(servo_id))

    if value < SERVO_ID_MIN or value > SERVO_ID_MAX:
        raise SafetyError(
            "Servo ID {} is outside the supported range {}-{}.".format(
                value, SERVO_ID_MIN, SERVO_ID_MAX
            )
        )
    return value


def validate_servo_ids(servo_ids: Iterable[int]) -> List[int]:
    """Validate IDs, remove duplicates, and preserve input order."""
    result = []
    seen = set()
    for servo_id in servo_ids:
        value = validate_servo_id(servo_id)
        if value not in seen:
            result.append(value)
            seen.add(value)
    return result


def validate_id_range(start_id: int, end_id: int) -> List[int]:
    """Return an inclusive validated servo ID range."""
    start = validate_servo_id(start_id)
    end = validate_servo_id(end_id)
    if start > end:
        raise SafetyError("start_id must be <= end_id.")
    return list(range(start, end + 1))


def require_read_only_hardware_confirmation(dry_run: bool, confirmed: bool) -> None:
    """Require explicit confirmation before read-only non-dry-run bus access."""
    if not dry_run and not confirmed:
        raise SafetyError(
            "Read-only real servo bus access requires --yes. Dry-run remains the default."
        )


def require_hardware_confirmation(dry_run: bool, confirmed: bool) -> None:
    """Disable generic hardware confirmation so movement cannot reuse --yes."""
    raise SafetyError(
        "Generic hardware confirmation is disabled. Use "
        "require_read_only_hardware_confirmation for scan/read tools; "
        "movement requires a separate movement-specific safety gate."
    )


def reject_movement_until_safety_gates_exist() -> None:
    """Reject movement until the full movement safety gate is implemented."""
    raise SafetyError(
        "Movement commands are not available yet. Future movement must use a "
        "separate movement-specific confirmation, not --yes, and must validate "
        "joint name, servo ID, current position, configured limits, small target "
        "delta, and logging."
    )
