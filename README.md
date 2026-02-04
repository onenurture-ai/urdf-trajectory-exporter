# urdf-trajectory-exporter

Export retargeted joint trajectories to CSV or JSON with headers that match the robot’s URDF actuated joints. Validates that each timestep has the same joints as the URDF.

## Install

```bash
pip install -e .
```

## Usage

```python
from pathlib import Path
from urdf_trajectory_exporter import URDFTrajectoryWriter, JointState

# From your retargeting step: sequence of joint states (one per timestep)
trajectory = [
    JointState(positions={"shoulder_pan": 0.0, "shoulder_lift": 0.1, "elbow": 0.0}),
    JointState(positions={"shoulder_pan": 0.1, "shoulder_lift": 0.2, "elbow": 0.05}),
    # ...
]

writer = URDFTrajectoryWriter("/path/to/robot.urdf")
writer.write(trajectory, "output.csv", format="csv")   # or format="json"
```

- **URDF**: Only **revolute**, **continuous**, and **prismatic** joints are used (actuated DOF). Fixed joints are ignored. Joint order in the file is the column order.
- **RobotJointState**: Any object with a `positions` attribute that is either:
  - a `dict[str, float]` (joint name → angle in radians or meters), or
  - a `list[float]` with a `joint_names` attribute giving the order.
- **Validation**: The writer checks that every timestep has exactly the same set of joint names as the URDF (no missing, no extra). Column count is enforced.

### Output formats

- **CSV**: Header row = URDF actuated joint names; each following row = one timestep of joint values.
- **JSON**: `{"joint_names": [...], "timesteps": [{"joint_a": 0.0, ...}, ...]}`.

## Development

```bash
pip install -e ".[dev]"
pytest tests -v
```
