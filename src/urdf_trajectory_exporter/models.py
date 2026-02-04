"""Data models for retargeting output compatible with URDFTrajectoryWriter."""

from dataclasses import dataclass
from typing import Protocol, Union

# Actuated joint types in URDF (revolute, continuous, prismatic have DOF)
ACTUATED_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})


class RobotJointState(Protocol):
    """Protocol for a single timestep of joint state from the retargeting step.

    Implementations must provide either:
    - positions: mapping of joint name -> angle (radians or meters for prismatic), or
    - positions as a sequence of floats in the same order as joint_names.

    Optionally provides joint_names when using ordered positions; otherwise
    joint order is taken from the URDF.
    """

    @property
    def positions(self) -> Union[dict[str, float], list[float]]:
        """Joint positions: dict[name -> value] or list of values in joint order."""
        ...

    @property
    def joint_names(self) -> list[str] | None:
        """If positions is a list, this defines the order. None if positions is a dict."""
        ...


@dataclass(frozen=True)
class JointState:
    """Concrete RobotJointState: positions dict and optional joint_names for list order."""

    positions: Union[dict[str, float], list[float]]
    joint_names: list[str] | None = None


def get_positions_dict(state: RobotJointState) -> dict[str, float]:
    """Normalize a RobotJointState to a dict joint_name -> position."""
    p = state.positions
    if isinstance(p, dict):
        return dict(p)
    names = getattr(state, "joint_names", None)
    if names is None:
        raise ValueError("RobotJointState with list positions must provide joint_names")
    if len(names) != len(p):
        raise ValueError(
            f"joint_names length ({len(names)}) does not match positions length ({len(p)})"
        )
    return dict(zip(names, p))


def get_ordered_positions(state: RobotJointState, joint_names: list[str]) -> list[float]:
    """Return positions as a list in the order of joint_names.

    Raises ValueError if the set of keys in state does not exactly match joint_names.
    """
    d = get_positions_dict(state)
    expected = set(joint_names)
    actual = set(d)
    if actual != expected:
        missing = expected - actual
        extra = actual - expected
        parts = []
        if missing:
            parts.append(f"missing: {sorted(missing)}")
        if extra:
            parts.append(f"extra: {sorted(extra)}")
        raise ValueError(f"Joint names do not match URDF: {', '.join(parts)}")
    return [d[name] for name in joint_names]
