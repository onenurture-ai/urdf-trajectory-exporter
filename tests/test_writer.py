"""Tests for URDFTrajectoryWriter and URDF parsing."""

import json
import tempfile
from pathlib import Path

import pytest

from urdf_trajectory_exporter import JointState, URDFTrajectoryWriter
from urdf_trajectory_exporter.urdf_parser import get_actuated_joint_names


@pytest.fixture
def sample_urdf(tmp_path: Path) -> Path:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""
<?xml version="1.0"?>
<robot name="test_robot">
  <joint name="joint_a" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="1" velocity="1"/>
  </joint>
  <joint name="fixed_joint" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
  <joint name="joint_b" type="continuous">
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
""")
    return urdf


def test_get_actuated_joint_names(sample_urdf: Path) -> None:
    names = get_actuated_joint_names(str(sample_urdf))
    assert names == ["joint_a", "joint_b"]
    assert "fixed_joint" not in names


def test_writer_csv(sample_urdf: Path, tmp_path: Path) -> None:
    writer = URDFTrajectoryWriter(sample_urdf)
    assert writer.joint_names == ["joint_a", "joint_b"]
    assert writer.num_joints == 2

    trajectory = [
        JointState(positions={"joint_a": 0.0, "joint_b": 0.1}),
        JointState(positions={"joint_a": 0.5, "joint_b": 0.2}),
    ]
    out = tmp_path / "out.csv"
    writer.write(trajectory, out, format="csv")

    lines = out.read_text().strip().splitlines()
    assert lines[0] == "joint_a,joint_b"
    assert lines[1] == "0.0,0.1"
    assert lines[2] == "0.5,0.2"


def test_writer_json(sample_urdf: Path, tmp_path: Path) -> None:
    writer = URDFTrajectoryWriter(sample_urdf)
    trajectory = [
        JointState(positions={"joint_a": 0.0, "joint_b": 0.1}),
    ]
    out = tmp_path / "out.json"
    writer.write(trajectory, out, format="json")

    data = json.loads(out.read_text())
    assert data["joint_names"] == ["joint_a", "joint_b"]
    assert data["timesteps"] == [{"joint_a": 0.0, "joint_b": 0.1}]


def test_writer_validates_column_count(sample_urdf: Path, tmp_path: Path) -> None:
    writer = URDFTrajectoryWriter(sample_urdf)
    # Too few keys
    trajectory = [JointState(positions={"joint_a": 0.0})]
    with pytest.raises(ValueError, match="missing"):
        writer.write(trajectory, tmp_path / "out.csv", format="csv")
    # Extra key
    trajectory = [
        JointState(positions={"joint_a": 0.0, "joint_b": 0.0, "extra": 1.0}),
    ]
    with pytest.raises(ValueError, match="extra"):
        writer.write(trajectory, tmp_path / "out.csv", format="csv")


def test_writer_validates_row_length(sample_urdf: Path, tmp_path: Path) -> None:
    writer = URDFTrajectoryWriter(sample_urdf)
    # List with wrong length (no joint_names so we use joint_names from URDF in get_ordered_positions;
    # actually we need joint_names on state for list - and then we'd have 3 elements vs 2 URDF joints)
    trajectory = [
        JointState(positions=[0.0, 0.1, 0.2], joint_names=["joint_a", "joint_b", "extra"]),
    ]
    with pytest.raises(ValueError, match="do not match URDF"):
        writer.write(trajectory, tmp_path / "out.csv", format="csv")


def test_writer_list_positions_same_order_as_urdf(sample_urdf: Path, tmp_path: Path) -> None:
    writer = URDFTrajectoryWriter(sample_urdf)
    # Positions in URDF order: joint_a, joint_b
    trajectory = [
        JointState(positions=[0.0, 0.1], joint_names=["joint_a", "joint_b"]),
    ]
    out = tmp_path / "out.csv"
    writer.write(trajectory, out, format="csv")
    lines = out.read_text().strip().splitlines()
    assert lines[0] == "joint_a,joint_b"
    assert lines[1] == "0.0,0.1"


def test_urdf_not_found() -> None:
    with pytest.raises(FileNotFoundError):
        URDFTrajectoryWriter("/nonexistent/robot.urdf")


def test_urdf_no_actuated_joints(tmp_path: Path) -> None:
    urdf = tmp_path / "fixed_only.urdf"
    urdf.write_text("""
<?xml version="1.0"?>
<robot name="fixed">
  <joint name="only_fixed" type="fixed">
    <parent link="base"/>
    <child link="link1"/>
  </joint>
</robot>
""")
    with pytest.raises(ValueError, match="No actuated joints"):
        URDFTrajectoryWriter(urdf)
