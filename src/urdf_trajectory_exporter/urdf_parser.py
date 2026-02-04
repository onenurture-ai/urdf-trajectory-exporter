"""Parse actuated joint names from a URDF file."""

import xml.etree.ElementTree as ET

from .models import ACTUATED_JOINT_TYPES


def get_actuated_joint_names(urdf_path: str) -> list[str]:
    """Return actuated joint names from URDF in document order.

    Actuated joints are those with type revolute, continuous, or prismatic
    (i.e. they have one degree of freedom). Fixed and other types are excluded.

    Raises:
        FileNotFoundError: If urdf_path does not exist.
        ValueError: If the file is not valid XML or contains no actuated joints.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # URDF root is <robot>; joints are direct children with tag "joint"
    names: list[str] = []
    for joint in root.findall("joint"):
        if joint is None:
            continue
        name = joint.get("name")
        kind = (joint.get("type") or "").strip().lower()
        if name is None:
            continue
        if kind in ACTUATED_JOINT_TYPES:
            names.append(name)
    if not names:
        raise ValueError(
            f"No actuated joints (revolute/continuous/prismatic) found in {urdf_path}"
        )
    return names
