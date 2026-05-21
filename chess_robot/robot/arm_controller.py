"""Single-joint micro-motion planning and execution."""

import os
import time
from typing import Any, Dict, Optional

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import (
    DEFAULT_CONFIG_PATH,
    ServoBus,
    ServoBusError,
    build_servo_bus,
    load_robot_config,
)


DEFAULT_JOINT_LIMITS_PATH = os.path.join("data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_GRIPPER_PROFILE_PATH = os.path.join("data", "calibration", "gripper", "gripper_profile.yaml")
DEFAULT_READBACK_DELAY_SECONDS = 0.2
DEFAULT_PRESENT_POSITION_TOLERANCE = 2
DEFAULT_PRESENT_MONITOR_SECONDS = 1.0
DEFAULT_PRESENT_MONITOR_INTERVAL_SECONDS = 0.1


class ArmControllerError(RuntimeError):
    """Raised when planning or executing a micro-motion fails."""


def _resolve_project_path(config_path: str, relative_path: str) -> str:
    config_dir = os.path.dirname(os.path.abspath(config_path))
    if os.path.basename(config_dir) == "configs":
        project_root = os.path.dirname(config_dir)
    else:
        project_root = os.getcwd()
    return os.path.join(project_root, relative_path)


def _load_yaml_mapping(path: str) -> Dict[str, Any]:
    if yaml is None:
        raise ArmControllerError("PyYAML is required to read calibration data.")
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ArmControllerError("YAML file must contain a mapping: {}".format(path))
    return data


def _resolve_joint_limits(config_path: str) -> Dict[str, Any]:
    joint_limits_path = _resolve_project_path(config_path, DEFAULT_JOINT_LIMITS_PATH)
    gripper_profile_path = _resolve_project_path(config_path, DEFAULT_GRIPPER_PROFILE_PATH)

    limits_data = _load_yaml_mapping(joint_limits_path)
    limits = limits_data.get("limits") or {}
    if not isinstance(limits, dict):
        limits = {}

    gripper_profile = _load_yaml_mapping(gripper_profile_path).get("gripper") or {}
    if isinstance(gripper_profile, dict):
        gripper_limits = gripper_profile.get("limits") or {}
        if isinstance(gripper_limits, dict):
            merged_gripper_limits = dict(limits.get("gripper") or {})
            if gripper_limits.get("min") is not None:
                merged_gripper_limits["min"] = gripper_limits.get("min")
            if gripper_limits.get("max") is not None:
                merged_gripper_limits["max"] = gripper_limits.get("max")
            if merged_gripper_limits:
                limits["gripper"] = merged_gripper_limits
    return limits


def _resolve_default_max_delta(config: Dict[str, Any], requested: Optional[int]) -> int:
    if requested is not None:
        value = int(requested)
        if value <= 0:
            raise ArmControllerError("--max-delta must be a positive integer.")
        return value

    candidates = [
        ((config.get("robot") or {}).get("single_joint_max_delta")),
        ((config.get("motion") or {}).get("single_joint_max_delta")),
        ((config.get("servo_bus") or {}).get("single_joint_max_delta")),
    ]
    for candidate in candidates:
        try:
            value = int(candidate)
        except (TypeError, ValueError):
            continue
        if value > 0:
            return min(value, safety.DEFAULT_SINGLE_JOINT_MAX_DELTA)
    return safety.DEFAULT_SINGLE_JOINT_MAX_DELTA


def _resolve_target_position(current_position: Optional[int],
                             delta: Optional[int],
                             target: Optional[int]) -> Optional[int]:
    if delta is None and target is None:
        raise ArmControllerError("Provide exactly one of delta or target.")
    if delta is not None and target is not None:
        raise ArmControllerError("Provide only one of delta or target, not both.")
    if delta is not None:
        if current_position is None:
            return None
        return int(current_position) + int(delta)
    return int(target)


class ArmController(object):
    """Safe single-joint planner/executor for guarded micro-motion."""

    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH,
                 config: Optional[Dict[str, Any]] = None,
                 bus: Optional[ServoBus] = None) -> None:
        self.config_path = config_path
        self.config = config or load_robot_config(config_path)
        self.bus = bus
        self.joints = self.config.get("joints") or {}
        self.joint_limits = _resolve_joint_limits(config_path)

    def close(self) -> None:
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def _ensure_bus(self, real: bool) -> ServoBus:
        if self.bus is not None:
            return self.bus

        servo_config = self.config.get("servo_bus") or {}
        configured_backend = servo_config.get("backend")
        has_feetech = bool(servo_config.get("feetech"))

        backend_name = configured_backend
        dry_run = (not real)
        if real:
            if configured_backend == "mock" and has_feetech:
                backend_name = "feetech"
            dry_run = False
        else:
            if has_feetech:
                backend_name = "feetech"
                dry_run = False
            else:
                backend_name = "mock"
                dry_run = True

        self.bus = build_servo_bus(
            config=self.config,
            config_path=self.config_path,
            dry_run=dry_run,
            backend_name=backend_name,
        )
        return self.bus

    def plan_single_joint_move(self,
                               joint: str,
                               delta: Optional[int] = None,
                               target: Optional[int] = None,
                               max_delta: Optional[int] = None,
                               real: bool = False) -> Dict[str, Any]:
        """Plan and validate a single-joint move."""
        normalized_joint = safety.normalize_joint_name(joint)
        bus = self._ensure_bus(real=real)

        current_position = None
        servo_id = None
        joint_config = self.joints.get(normalized_joint)
        if isinstance(joint_config, dict) and joint_config.get("servo_id") is not None:
            servo_id = safety.validate_servo_id(joint_config.get("servo_id"))
            current_position = bus.read_position(servo_id)

        target_position = _resolve_target_position(current_position, delta, target)
        effective_max_delta = _resolve_default_max_delta(self.config, max_delta)
        validation = safety.validate_single_joint_move(
            joint=normalized_joint,
            configured_joints=self.joints,
            current_position=current_position,
            target_position=target_position,
            limits_by_joint=self.joint_limits,
            commanded_joints=[normalized_joint],
            max_delta=effective_max_delta,
        )

        expected_confirmation = None
        confirmation_target = validation.get("target_position")
        if confirmation_target is None:
            confirmation_target = target_position
        if validation.get("ok") and confirmation_target is not None:
            expected_confirmation = safety.build_move_confirmation(
                validation.get("joint") or normalized_joint,
                confirmation_target,
            )
        return {
            "dry_run": not real,
            "joint": validation.get("joint") or normalized_joint,
            "servo_id": validation.get("servo_id"),
            "current_position": validation.get("current_position"),
            "target_position": validation.get("target_position"),
            "delta": validation.get("delta"),
            "limits": validation.get("limits"),
            "max_delta": validation.get("max_delta"),
            "validation": validation,
            "expected_confirmation": expected_confirmation,
        }

    def _log_move_attempt(self,
                          plan: Dict[str, Any],
                          confirmation_required: bool,
                          confirmation_matched: bool,
                          write_result: Optional[Dict[str, Any]],
                          readback_result: Optional[Dict[str, Any]],
                          success: bool,
                          error_message: Optional[str]) -> None:
        bus = self._ensure_bus(real=(not bool(plan.get("dry_run", True))))
        validation = plan.get("validation") or {}
        bus.logger.log(
            "single_joint_move_attempt",
            dry_run=bool(plan.get("dry_run", True)),
            joint=plan.get("joint"),
            servo_id=plan.get("servo_id"),
            current_position=plan.get("current_position"),
            target_position=plan.get("target_position"),
            delta=plan.get("delta"),
            max_delta=plan.get("max_delta"),
            limits=plan.get("limits"),
            validation_result=validation,
            confirmation_required=bool(confirmation_required),
            confirmation_matched=bool(confirmation_matched),
            write_result=write_result,
            readback_result=readback_result,
            success=bool(success),
            error_message=error_message,
        )

    def _monitor_present_position(self,
                                  servo_id: int,
                                  target_position: int,
                                  limits: Dict[str, Any],
                                  timeout_seconds: float,
                                  interval_seconds: float) -> Dict[str, Any]:
        bus = self._ensure_bus(real=True)
        deadline = time.time() + float(timeout_seconds)
        samples = []
        result = {
            "present_position": None,
            "present_position_samples": samples,
            "present_position_tolerance": DEFAULT_PRESENT_POSITION_TOLERANCE,
            "verified": False,
            "failure_reason": "present_position_unavailable",
        }

        while True:
            present_position = bus.read_position(servo_id)
            samples.append(present_position)
            result["present_position"] = present_position
            if present_position is None:
                result["failure_reason"] = "present_position_unavailable"
            else:
                minimum = limits.get("min")
                maximum = limits.get("max")
                if minimum is not None and present_position < int(minimum):
                    result["failure_reason"] = "present_position_below_limit"
                    return result
                if maximum is not None and present_position > int(maximum):
                    result["failure_reason"] = "present_position_above_limit"
                    return result
                if abs(int(present_position) - target_position) <= DEFAULT_PRESENT_POSITION_TOLERANCE:
                    result["verified"] = True
                    result["failure_reason"] = None
                    return result
                result["failure_reason"] = "present_position_outside_target_tolerance"

            if time.time() >= deadline:
                return result
            time.sleep(float(interval_seconds))

    def execute_single_joint_move(self,
                                  plan: Dict[str, Any],
                                  confirmation_text: Optional[str] = None,
                                  readback_delay_seconds: float = DEFAULT_READBACK_DELAY_SECONDS) -> Dict[str, Any]:
        """Execute a validated single-joint move or log a dry-run attempt."""
        validation = plan.get("validation") or {}
        dry_run = bool(plan.get("dry_run", True))
        confirmation_required = not dry_run
        confirmation_matched = False
        write_result = None
        readback_result = None

        if not validation.get("ok"):
            error_message = validation.get("reason") or "validation_failed"
            self._log_move_attempt(
                plan=plan,
                confirmation_required=confirmation_required,
                confirmation_matched=False,
                write_result=write_result,
                readback_result=readback_result,
                success=False,
                error_message=error_message,
            )
            raise ArmControllerError(error_message)

        if dry_run:
            self._log_move_attempt(
                plan=plan,
                confirmation_required=False,
                confirmation_matched=False,
                write_result=None,
                readback_result=None,
                success=True,
                error_message=None,
            )
            return {
                "success": True,
                "dry_run": True,
                "message": "Dry-run validated. No servo write performed.",
                "plan": plan,
            }

        try:
            safety.require_real_movement_joint_allowed(plan.get("joint"))
        except safety.SafetyError as exc:
            self._log_move_attempt(
                plan=plan,
                confirmation_required=False,
                confirmation_matched=False,
                write_result=None,
                readback_result=None,
                success=False,
                error_message=str(exc),
            )
            raise ArmControllerError(str(exc))

        expected_confirmation = plan.get("expected_confirmation")
        confirmation_matched = safety.confirmation_matches(
            expected_confirmation, confirmation_text
        )
        if not confirmation_matched:
            error_message = "Typed confirmation did not match exactly."
            self._log_move_attempt(
                plan=plan,
                confirmation_required=True,
                confirmation_matched=False,
                write_result=None,
                readback_result=None,
                success=False,
                error_message=error_message,
            )
            raise ArmControllerError(error_message)

        bus = self._ensure_bus(real=True)
        target_position = int(plan.get("target_position"))
        servo_id = int(plan.get("servo_id"))
        try:
            bus.write_goal_position(servo_id, target_position)
            write_result = {
                "status": "ok",
                "servo_id": servo_id,
                "target_position": target_position,
            }

            time.sleep(float(readback_delay_seconds))
            readback_result = {
                "goal_position": None,
                "present_position": None,
                "present_position_samples": [],
                "present_position_tolerance": DEFAULT_PRESENT_POSITION_TOLERANCE,
                "verified": False,
                "failure_reason": None,
            }

            try:
                readback_result["goal_position"] = bus.read_goal_position(servo_id)
            except (ServoBusError, Exception):
                readback_result["goal_position"] = None

            present_result = self._monitor_present_position(
                servo_id=servo_id,
                target_position=target_position,
                limits=plan.get("limits") or {},
                timeout_seconds=DEFAULT_PRESENT_MONITOR_SECONDS,
                interval_seconds=DEFAULT_PRESENT_MONITOR_INTERVAL_SECONDS,
            )
            readback_result.update(present_result)

            success = bool(readback_result.get("verified"))
            if not success:
                raise ArmControllerError(
                    "Present-position readback did not verify the requested target: {}".format(
                        readback_result.get("failure_reason") or "unknown"
                    )
                )
        except Exception as exc:
            if write_result is None:
                write_result = {
                    "status": "error",
                    "servo_id": servo_id,
                    "target_position": target_position,
                }
            self._log_move_attempt(
                plan=plan,
                confirmation_required=True,
                confirmation_matched=True,
                write_result=write_result,
                readback_result=readback_result,
                success=False,
                error_message=str(exc),
            )
            if isinstance(exc, ArmControllerError):
                raise
            raise ArmControllerError(str(exc))

        self._log_move_attempt(
            plan=plan,
            confirmation_required=True,
            confirmation_matched=True,
            write_result=write_result,
            readback_result=readback_result,
            success=True,
            error_message=None,
        )
        return {
            "success": True,
            "dry_run": False,
            "message": "Servo write verified.",
            "plan": plan,
            "write_result": write_result,
            "readback_result": readback_result,
        }


def plan_single_joint_move(joint: str,
                           delta: Optional[int] = None,
                           target: Optional[int] = None,
                           max_delta: Optional[int] = None,
                           real: bool = False,
                           config_path: str = DEFAULT_CONFIG_PATH,
                           config: Optional[Dict[str, Any]] = None,
                           bus: Optional[ServoBus] = None) -> Dict[str, Any]:
    """Convenience wrapper for single-joint planning."""
    controller = ArmController(config_path=config_path, config=config, bus=bus)
    return controller.plan_single_joint_move(
        joint=joint,
        delta=delta,
        target=target,
        max_delta=max_delta,
        real=real,
    )


def execute_single_joint_move(plan: Dict[str, Any],
                              confirmation_text: Optional[str] = None,
                              config_path: str = DEFAULT_CONFIG_PATH,
                              config: Optional[Dict[str, Any]] = None,
                              bus: Optional[ServoBus] = None) -> Dict[str, Any]:
    """Convenience wrapper for single-joint execution."""
    controller = ArmController(config_path=config_path, config=config, bus=bus)
    return controller.execute_single_joint_move(
        plan=plan,
        confirmation_text=confirmation_text,
    )
