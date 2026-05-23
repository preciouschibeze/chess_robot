"""Lightweight URDF parsing helpers for the SO101 serial arm."""

from __future__ import absolute_import

import xml.etree.ElementTree as ET

DEFAULT_END_LINK = "gripper_frame_link"
GRIPPER_JOINT_NAME = "gripper"
EXPECTED_ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


class JointLimit(object):
    """Numeric URDF joint limits when present."""

    def __init__(self, lower=None, upper=None, effort=None, velocity=None):
        self.lower = lower
        self.upper = upper
        self.effort = effort
        self.velocity = velocity


class JointModel(object):
    """URDF joint description with parsed numeric fields."""

    def __init__(
        self,
        name,
        joint_type,
        parent,
        child,
        origin_xyz,
        origin_rpy,
        axis,
        limit=None,
    ):
        self.name = name
        self.joint_type = joint_type
        self.parent = parent
        self.child = child
        self.origin_xyz = tuple(origin_xyz)
        self.origin_rpy = tuple(origin_rpy)
        self.axis = tuple(axis)
        self.limit = limit

    @property
    def is_fixed(self):
        return self.joint_type == "fixed"

    @property
    def is_movable(self):
        return not self.is_fixed


class UrdfModel(object):
    """Minimal URDF model for serial-chain inspection and FK."""

    def __init__(self, robot_name, links, joints):
        self.robot_name = robot_name
        self.links = list(links)
        self.joints = list(joints)
        self.joints_by_name = dict((joint.name, joint) for joint in self.joints)
        self.child_to_joint = dict((joint.child, joint) for joint in self.joints)
        self.root_link = self._find_root_link()

    @classmethod
    def from_file(cls, urdf_path):
        root = ET.parse(urdf_path).getroot()
        robot_name = root.attrib.get("name", "unknown_robot")
        links = []
        joints = []

        for link_node in root.findall("link"):
            link_name = link_node.attrib.get("name")
            if link_name:
                links.append(link_name)

        for joint_node in root.findall("joint"):
            joints.append(_parse_joint_node(joint_node))

        return cls(robot_name=robot_name, links=links, joints=joints)

    def _find_root_link(self):
        child_links = set(self.child_to_joint.keys())
        candidate_links = [link_name for link_name in self.links if link_name not in child_links]
        if not candidate_links:
            raise ValueError("No root link found in URDF model.")
        if "base_link" in candidate_links:
            return "base_link"
        return candidate_links[0]

    def get_joint(self, joint_name):
        return self.joints_by_name[joint_name]

    def get_movable_joints(self):
        return [joint for joint in self.joints if joint.is_movable]

    def get_chain(self, end_link=DEFAULT_END_LINK, root_link=None):
        if end_link not in self.links:
            raise KeyError("Unknown end link: %s" % end_link)

        root_link = root_link or self.root_link
        chain = []
        current_link = end_link
        visited_links = set()

        while current_link != root_link:
            if current_link in visited_links:
                raise ValueError("Cycle detected while resolving chain to %s" % end_link)
            visited_links.add(current_link)

            joint = self.child_to_joint.get(current_link)
            if joint is None:
                raise ValueError(
                    "Could not resolve parent joint for link %s while tracing chain to %s"
                    % (current_link, end_link)
                )
            chain.append(joint)
            current_link = joint.parent

        chain.reverse()
        return chain

    def get_arm_chain(self, end_link=DEFAULT_END_LINK):
        chain = self.get_chain(end_link=end_link)
        arm_chain = []
        for joint in chain:
            if not joint.is_movable:
                continue
            if joint.name == GRIPPER_JOINT_NAME:
                continue
            arm_chain.append(joint)
        return arm_chain


def load_urdf_model(urdf_path):
    """Load a URDF file into the lightweight model representation."""

    return UrdfModel.from_file(urdf_path)


def _parse_joint_node(joint_node):
    name = joint_node.attrib.get("name")
    joint_type = joint_node.attrib.get("type", "fixed")

    parent_node = joint_node.find("parent")
    child_node = joint_node.find("child")
    if parent_node is None or child_node is None:
        raise ValueError("Joint %s is missing parent/child tags." % name)

    origin_node = joint_node.find("origin")
    axis_node = joint_node.find("axis")
    limit_node = joint_node.find("limit")

    origin_xyz = _parse_vector_attribute(origin_node, "xyz", (0.0, 0.0, 0.0))
    origin_rpy = _parse_vector_attribute(origin_node, "rpy", (0.0, 0.0, 0.0))
    axis = _parse_vector_attribute(axis_node, "xyz", (1.0, 0.0, 0.0))
    limit = _parse_limit(limit_node)

    return JointModel(
        name=name,
        joint_type=joint_type,
        parent=parent_node.attrib.get("link"),
        child=child_node.attrib.get("link"),
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        axis=axis,
        limit=limit,
    )


def _parse_vector_attribute(node, key, default):
    if node is None:
        return tuple(default)
    value = node.attrib.get(key)
    if value is None:
        return tuple(default)
    parts = value.split()
    if len(parts) != 3:
        raise ValueError("Expected 3 values for %s, got %r" % (key, value))
    return tuple(float(part) for part in parts)


def _parse_limit(limit_node):
    if limit_node is None:
        return None
    return JointLimit(
        lower=_parse_optional_float(limit_node.attrib.get("lower")),
        upper=_parse_optional_float(limit_node.attrib.get("upper")),
        effort=_parse_optional_float(limit_node.attrib.get("effort")),
        velocity=_parse_optional_float(limit_node.attrib.get("velocity")),
    )


def _parse_optional_float(value):
    if value is None:
        return None
    return float(value)
