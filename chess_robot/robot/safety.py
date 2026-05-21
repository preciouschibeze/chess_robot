"""Safety checks for robot hardware access.

This module contains validation and confirmation gates. It does not talk to
hardware; direct servo bus access belongs only in chess_robot.robot.servo_bus.
"""

from typing import Any, Dict, Iterable, List, Optional


SERVO_ID_MIN = 0
SERVO_ID_MAX = 253
DEFAULT_SINGLE_JOINT_MAX_DELTA = 10
REAL_MOVEMENT_ALLOWED_JOINTS = ("gripper",)


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


def normalize_joint_name(joint: Any) -> str:
    """Normalize a joint label."""
    if joint is None:
        raise SafetyError("Joint name is required.")
    value = str(joint).strip()
    if not value:
        raise SafetyError("Joint name is required.")
    return value


def validate_single_commanded_joint(joints: Iterable[Any]) -> List[str]:
    """Validate that exactly one joint is being commanded."""
    normalized = []
    seen = set()
    for joint in joints or []:
        value = normalize_joint_name(joint)
        if value not in seen:
            normalized.append(value)
            seen.add(value)
    if len(normalized) != 1:
        raise SafetyError(
            "Exactly one joint must be commanded; got {}.".format(len(normalized))
        )
    return normalized


def coerce_optional_int(value: Any) -> Optional[int]:
    """Return an integer value or None."""
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def resolve_joint_limits(limits_by_joint: Dict[str, Any], joint: str) -> Optional[Dict[str, int]]:
    """Resolve min/max limits from a joint limit mapping."""
    entry = (limits_by_joint or {}).get(joint)
    if not isinstance(entry, dict):
        return None

    minimum = None
    maximum = None
    for key in ("min", "provisional_min"):
        minimum = coerce_optional_int(entry.get(key))
        if minimum is not None:
            break
    for key in ("max", "provisional_max"):
        maximum = coerce_optional_int(entry.get(key))
        if maximum is not None:
            break
    if minimum is None or maximum is None:
        return None
    return {"min": minimum, "max": maximum}


def build_single_joint_validation_result(
        ok: bool,
        reason: str,
        joint: Optional[str],
        servo_id: Optional[int],
        current_position: Optional[int],
        target_position: Optional[int],
        delta: Optional[int],
        limits: Optional[Dict[str, int]],
        max_delta: int) -> Dict[str, Any]:
    """Build a structured validation result."""
    return {
        "ok": bool(ok),
        "reason": str(reason),
        "joint": joint,
        "servo_id": servo_id,
        "current_position": current_position,
        "target_position": target_position,
        "delta": delta,
        "limits": limits,
        "max_delta": int(max_delta),
    }


def validate_single_joint_move(
        joint: Any,
        configured_joints: Dict[str, Any],
        current_position: Any,
        target_position: Any,
        limits_by_joint: Dict[str, Any],
        commanded_joints: Iterable[Any],
        max_delta: Optional[int] = None) -> Dict[str, Any]:
    """Validate a single-joint move request and return a structured result."""
    max_delta_value = coerce_optional_int(max_delta)
    if max_delta_value is None or max_delta_value <= 0:
        max_delta_value = DEFAULT_SINGLE_JOINT_MAX_DELTA

    normalized_joint = None
    servo_id = None
    current_value = coerce_optional_int(current_position)
    target_value = coerce_optional_int(target_position)
    delta = None
    limits = None

    try:
        commanded = validate_single_commanded_joint(commanded_joints)
        normalized_joint = normalize_joint_name(joint)
        if commanded[0] != normalized_joint:
            raise SafetyError(
                "Only one joint may be commanded, and it must match {!r}.".format(normalized_joint)
            )

        joint_config = (configured_joints or {}).get(normalized_joint)
        if not isinstance(joint_config, dict):
            raise SafetyError("Unknown joint {!r}.".format(normalized_joint))

        if joint_config.get("servo_id") is None:
            raise SafetyError("Joint {!r} has no configured servo ID.".format(normalized_joint))
        servo_id = validate_servo_id(joint_config.get("servo_id"))

        if current_value is None:
            raise SafetyError(
                "Current position is unreadable for joint {!r} (servo {}).".format(
                    normalized_joint, servo_id
                )
            )

        if target_value is None:
            raise SafetyError("Target position must be an integer.")

        limits = resolve_joint_limits(limits_by_joint, normalized_joint)
        if limits is None:
            raise SafetyError("No joint limits are configured for {!r}.".format(normalized_joint))

        if target_value < limits["min"] or target_value > limits["max"]:
            raise SafetyError(
                "Target {} is outside limits {}..{} for {!r}.".format(
                    target_value, limits["min"], limits["max"], normalized_joint
                )
            )

        delta = abs(target_value - current_value)
        if delta > max_delta_value:
            raise SafetyError(
                "Requested move delta {} exceeds max_delta {} for {!r}.".format(
                    delta, max_delta_value, normalized_joint
                )
            )
    except SafetyError as exc:
        return build_single_joint_validation_result(
            ok=False,
            reason=str(exc),
            joint=normalized_joint,
            servo_id=servo_id,
            current_position=current_value,
            target_position=target_value,
            delta=delta,
            limits=limits,
            max_delta=max_delta_value,
        )

    return build_single_joint_validation_result(
        ok=True,
        reason="validated",
        joint=normalized_joint,
        servo_id=servo_id,
        current_position=current_value,
        target_position=target_value,
        delta=delta,
        limits=limits,
        max_delta=max_delta_value,
    )


def require_real_movement_joint_allowed(joint: Any) -> str:
    """Reject real movement for all joints except the current allowlist."""
    normalized_joint = normalize_joint_name(joint)
    if normalized_joint not in REAL_MOVEMENT_ALLOWED_JOINTS:
        raise SafetyError(
            "Real movement is currently allowed only for gripper; {!r} is dry-run only.".format(
                normalized_joint
            )
        )
    return normalized_joint


def build_move_confirmation(joint: Any, target_position: Any) -> str:
    """Return the exact typed confirmation string required for real movement."""
    normalized_joint = normalize_joint_name(joint)
    target_value = coerce_optional_int(target_position)
    if target_value is None:
        raise SafetyError("Target position must be an integer for confirmation.")
    return "MOVE {} {}".format(normalized_joint, target_value)


def confirmation_matches(expected: str, provided: Any) -> bool:
    """Return True only when the typed confirmation matches exactly."""
    if provided is None:
        return False
    return str(provided) == str(expected)


def reject_movement_until_safety_gates_exist() -> None:
    """Preserved compatibility helper for older callers."""
    raise SafetyError(
        "Movement commands are not available through this compatibility path. "
        "Use validate_single_joint_move and typed MOVE confirmation gates instead."
    )


ABOVE_SQUARE_CONFIRMATION_TEXT = "MOVE ABOVE SQUARES"


def validate_multi_joint_commanded_joints(configured_joints: Dict[str, Any],
                                          commanded_joints: Iterable[Any],
                                          include_gripper: bool = False) -> List[str]:
    """Validate a guarded multi-joint command set for above-square playback."""
    if not isinstance(configured_joints, dict):
        raise SafetyError("Configured joints mapping is required.")
    normalized = []
    seen = set()
    for joint in commanded_joints:
        joint_name = normalize_joint_name(joint)
        if joint_name not in configured_joints:
            raise SafetyError("Unknown configured joint {!r}.".format(joint_name))
        if joint_name == "gripper" and not include_gripper:
            raise SafetyError("Gripper playback requires explicit --include-gripper.")
        if joint_name not in seen:
            normalized.append(joint_name)
            seen.add(joint_name)
    if not normalized:
        raise SafetyError("At least one commanded joint is required.")
    return normalized


def build_above_square_confirmation() -> str:
    """Return the exact typed confirmation string for above-square playback."""
    return ABOVE_SQUARE_CONFIRMATION_TEXT


def require_real_above_square_confirmation(provided: Any) -> str:
    """Require the exact typed confirmation for real above-square playback."""
    expected = build_above_square_confirmation()
    if not confirmation_matches(expected, provided):
        raise SafetyError(
            "Typed confirmation did not match exactly. Expected {!r}.".format(expected)
        )
    return expected
