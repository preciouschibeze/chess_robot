"""Dry-run motion primitive resolution from symbolic move plans."""

from __future__ import absolute_import

import os

import yaml


class PrimitiveStep(object):
    """Single primitive step in a dry-run execution preview."""

    def __init__(self, name, params=None, requires_pose=False, pose=None, notes=None):
        self.name = name
        self.params = params or {}
        self.requires_pose = bool(requires_pose)
        self.pose = pose
        self.notes = list(notes or [])

    def to_dict(self):
        return {
            "name": self.name,
            "params": dict(self.params),
            "requires_pose": self.requires_pose,
            "pose": self.pose,
            "notes": list(self.notes),
        }


class PrimitiveResolutionResult(object):
    """Result of resolving a symbolic move plan into dry-run primitives."""

    def __init__(
        self,
        supported,
        ready_for_execution,
        missing_calibration,
        steps,
        source_plan_path=None,
        notes=None,
    ):
        self.supported = bool(supported)
        self.ready_for_execution = bool(ready_for_execution)
        self.missing_calibration = list(missing_calibration or [])
        self.steps = list(steps or [])
        self.source_plan_path = source_plan_path
        self.notes = list(notes or [])

    def to_dict(self):
        return {
            "supported": self.supported,
            "ready_for_execution": self.ready_for_execution,
            "missing_calibration": list(self.missing_calibration),
            "steps": [step.to_dict() for step in self.steps],
            "source_plan_path": self.source_plan_path,
            "notes": list(self.notes),
        }


def load_yaml_file(path):
    """Load a YAML file to dict, returning {} when absent or invalid type."""
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r") as handle:
        data = yaml.safe_load(handle)
    if isinstance(data, dict):
        return data
    return {}


def _pose_from_square(square_targets, square, pose_name):
    squares = square_targets.get("squares", {}) if isinstance(square_targets, dict) else {}
    square_info = squares.get(square, {}) if isinstance(squares, dict) else {}
    if not isinstance(square_info, dict):
        square_info = {}
    return square_info.get(pose_name)


def _pose_from_zone(square_targets, zone_name, pose_name):
    zones = square_targets.get("zones", {}) if isinstance(square_targets, dict) else {}
    zone_info = zones.get(zone_name, {}) if isinstance(zones, dict) else {}
    if not isinstance(zone_info, dict):
        zone_info = {}
    return zone_info.get(pose_name)


def _add_missing(missing, key):
    if key not in missing:
        missing.append(key)


def resolve_move_plan(
    plan_dict,
    square_targets_path="data/calibration/robot/square_targets.yaml",
    home_pose_path="data/calibration/robot/home_pose.yaml",
    gripper_profile_path="data/calibration/gripper/gripper_profile.yaml",
):
    """Resolve symbolic move-plan actions into dry-run primitive steps."""
    square_targets = load_yaml_file(square_targets_path)
    home_pose_data = load_yaml_file(home_pose_path)
    gripper_profile = load_yaml_file(gripper_profile_path)

    actions = plan_dict.get("actions", []) if isinstance(plan_dict, dict) else []
    source_plan_path = None
    if isinstance(plan_dict, dict):
        source_plan_path = plan_dict.get("source_plan_path")

    supported = True
    notes = []
    missing = []
    steps = []

    home_pose = home_pose_data.get("home_pose") if isinstance(home_pose_data, dict) else None
    gripper = gripper_profile.get("gripper", {}) if isinstance(gripper_profile, dict) else {}
    if not isinstance(gripper, dict):
        gripper = {}

    for action in actions:
        if not isinstance(action, dict):
            supported = False
            notes.append("Invalid action entry: expected dict.")
            continue

        action_name = action.get("name")
        params = action.get("params", {})
        if not isinstance(params, dict):
            params = {}

        if action_name == "move_home":
            pose = home_pose
            if pose is None:
                _add_missing(missing, "home_pose")
            steps.append(
                PrimitiveStep(
                    name="move_joints",
                    params={"target": "home_pose"},
                    requires_pose=True,
                    pose=pose,
                )
            )
            continue

        if action_name == "move_above_square":
            square = params.get("square")
            pose = _pose_from_square(square_targets, square, "above_pose")
            if pose is None:
                _add_missing(missing, "squares.%s.above_pose" % square)
            steps.append(
                PrimitiveStep(
                    name="move_joints",
                    params={"target": "%s.above_pose" % square, "square": square},
                    requires_pose=True,
                    pose=pose,
                )
            )
            continue

        if action_name == "descend_to_pick":
            square = params.get("square")
            pose = _pose_from_square(square_targets, square, "pick_pose")
            if pose is None:
                _add_missing(missing, "squares.%s.pick_pose" % square)
            steps.append(
                PrimitiveStep(
                    name="move_joints",
                    params={"target": "%s.pick_pose" % square, "square": square},
                    requires_pose=True,
                    pose=pose,
                )
            )
            continue

        if action_name == "descend_to_place":
            square = params.get("square")
            pose = _pose_from_square(square_targets, square, "place_pose")
            target_name = "place_pose"
            step_notes = []
            if pose is None:
                fallback_pose = _pose_from_square(square_targets, square, "pick_pose")
                if fallback_pose is not None:
                    pose = fallback_pose
                    target_name = "pick_pose"
                    step_notes.append("place_pose missing; fell back to pick_pose")
                else:
                    _add_missing(missing, "squares.%s.place_pose" % square)
                    _add_missing(missing, "squares.%s.pick_pose" % square)
            steps.append(
                PrimitiveStep(
                    name="move_joints",
                    params={"target": "%s.%s" % (square, target_name), "square": square},
                    requires_pose=True,
                    pose=pose,
                    notes=step_notes,
                )
            )
            continue

        if action_name == "lift_from_square":
            square = params.get("square")
            pose = _pose_from_square(square_targets, square, "above_pose")
            if pose is None:
                _add_missing(missing, "squares.%s.above_pose" % square)
            steps.append(
                PrimitiveStep(
                    name="move_joints",
                    params={"target": "%s.above_pose" % square, "square": square},
                    requires_pose=True,
                    pose=pose,
                )
            )
            continue

        if action_name == "close_gripper":
            position = gripper.get("grasp_position")
            if position is None:
                _add_missing(missing, "gripper.grasp_position")
            steps.append(
                PrimitiveStep(
                    name="set_gripper",
                    params={"position_name": "grasp_position", "position": position},
                    requires_pose=False,
                    pose=None,
                )
            )
            continue

        if action_name == "open_gripper":
            if "release_position" in gripper and gripper.get("release_position") is not None:
                position_name = "release_position"
                position = gripper.get("release_position")
            else:
                position_name = "open_position"
                position = gripper.get("open_position")
                if position is None:
                    _add_missing(missing, "gripper.release_position")
                    _add_missing(missing, "gripper.open_position")
            steps.append(
                PrimitiveStep(
                    name="set_gripper",
                    params={"position_name": position_name, "position": position},
                    requires_pose=False,
                    pose=None,
                )
            )
            continue

        if action_name == "move_to_capture_zone":
            above_pose = _pose_from_zone(square_targets, "capture_zone", "above_pose")
            place_pose = _pose_from_zone(square_targets, "capture_zone", "place_pose")
            if above_pose is None:
                _add_missing(missing, "zones.capture_zone.above_pose")
            if place_pose is None:
                _add_missing(missing, "zones.capture_zone.place_pose")
            if above_pose is None or place_pose is None:
                supported = False
                notes.append("move_to_capture_zone unsupported without capture_zone calibration")
                steps.append(
                    PrimitiveStep(
                        name="move_joints",
                        params={"target": "capture_zone.above_pose", "zone": "capture_zone"},
                        requires_pose=True,
                        pose=above_pose,
                    )
                )
            else:
                steps.append(
                    PrimitiveStep(
                        name="move_joints",
                        params={"target": "capture_zone.above_pose", "zone": "capture_zone"},
                        requires_pose=True,
                        pose=above_pose,
                    )
                )
                steps.append(
                    PrimitiveStep(
                        name="move_joints",
                        params={"target": "capture_zone.place_pose", "zone": "capture_zone"},
                        requires_pose=True,
                        pose=place_pose,
                    )
                )
            continue

        supported = False
        notes.append("Unknown action: %s" % action_name)

    ready_for_execution = bool(supported and not missing)
    return PrimitiveResolutionResult(
        supported=supported,
        ready_for_execution=ready_for_execution,
        missing_calibration=missing,
        steps=steps,
        source_plan_path=source_plan_path,
        notes=notes,
    )
