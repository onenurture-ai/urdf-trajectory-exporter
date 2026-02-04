"""Write trajectory CSV/JSON with URDF-aligned joint headers and validation."""

import csv
import json
from pathlib import Path
from typing import Iterable, Literal

from .models import RobotJointState, get_ordered_positions
from .urdf_parser import get_actuated_joint_names


class URDFTrajectoryWriter:
    """Writes a sequence of RobotJointState timesteps to CSV or JSON.

    Headers match the actuated joint names from the robot's URDF (revolute,
    continuous, prismatic). Column count is validated against the URDF.
    """

    def __init__(self, urdf_path: str | Path) -> None:
        """Load joint order from the robot's URDF.

        Args:
            urdf_path: Path to the robot's URDF file.

        Raises:
            FileNotFoundError: If urdf_path does not exist.
            ValueError: If URDF has no actuated joints.
        """
        self._urdf_path = Path(urdf_path)
        self._joint_names = get_actuated_joint_names(str(self._urdf_path))
        self._num_joints = len(self._joint_names)

    @property
    def joint_names(self) -> list[str]:
        """Actuated joint names in URDF order (read-only)."""
        return list(self._joint_names)

    @property
    def num_joints(self) -> int:
        """Number of actuated joints in the URDF."""
        return self._num_joints

    def _validate_state(self, state: RobotJointState, index: int) -> list[float]:
        """Return ordered positions for one state; raise if column count or keys mismatch."""
        row = get_ordered_positions(state, self._joint_names)
        if len(row) != self._num_joints:
            raise ValueError(
                f"Timestep {index}: expected {self._num_joints} joint values (from URDF), "
                f"got {len(row)}"
            )
        return row

    def write(
        self,
        trajectory: Iterable[RobotJointState],
        output_path: str | Path,
        format: Literal["csv", "json"] = "csv",
    ) -> None:
        """Write trajectory to a CSV or JSON file.

        Header row (CSV) or key (JSON) is the URDF actuated joint names.
        Each subsequent row is one timestep of joint angles.

        Args:
            trajectory: Sequence of joint states (one per timestep).
            output_path: Path for the output file.
            format: "csv" or "json".

        Raises:
            ValueError: If any timestep has a different number of joints than the URDF,
                or missing/extra joint names when using dict positions.
        """
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        rows: list[list[float]] = []
        for i, state in enumerate(trajectory):
            rows.append(self._validate_state(state, i))

        if format == "csv":
            self._write_csv(path, rows)
        elif format == "json":
            self._write_json(path, rows)
        else:
            raise ValueError(f"Unsupported format: {format!r}. Use 'csv' or 'json'.")

    def _write_csv(self, path: Path, rows: list[list[float]]) -> None:
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self._joint_names)
            w.writerows(rows)

    def _write_json(self, path: Path, rows: list[list[float]]) -> None:
        data = {
            "joint_names": self._joint_names,
            "timesteps": [dict(zip(self._joint_names, row)) for row in rows],
        }
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
