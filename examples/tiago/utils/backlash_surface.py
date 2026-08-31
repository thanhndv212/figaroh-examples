"""Bounded empirical backlash compensation surface for the TIAGo example."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _polynomial_design(
    position: np.ndarray, torque: np.ndarray, degree: int
) -> np.ndarray:
    columns = []
    for total_degree in range(degree + 1):
        for torque_degree in range(total_degree + 1):
            position_degree = total_degree - torque_degree
            columns.append(position**position_degree * torque**torque_degree)
    return np.column_stack(columns)


@dataclass(frozen=True)
class BacklashJointTrajectory:
    """Finite relative/absolute encoder signals for one TIAGo joint."""

    joint_name: str
    time: np.ndarray
    relative_position: np.ndarray
    absolute_position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray

    def __post_init__(self) -> None:
        fields = {
            "time": self.time,
            "relative_position": self.relative_position,
            "absolute_position": self.absolute_position,
            "velocity": self.velocity,
            "effort": self.effort,
        }
        values = {
            name: np.asarray(value, dtype=float).reshape(-1)
            for name, value in fields.items()
        }
        if not self.joint_name:
            raise ValueError("joint_name must be non-empty")
        if values["time"].size < 2:
            raise ValueError("at least two finite encoder samples are required")
        if any(value.shape != values["time"].shape for value in values.values()):
            raise ValueError("encoder signals must have equal lengths")
        if not all(np.all(np.isfinite(value)) for value in values.values()):
            raise ValueError("encoder signals must be finite")
        if np.any(np.diff(values["time"]) <= 0.0):
            raise ValueError("encoder timestamps must be strictly increasing")
        for name, value in values.items():
            object.__setattr__(self, name, value)

    @property
    def sample_count(self) -> int:
        """Return the number of validated encoder samples."""
        return self.time.size

    @property
    def encoder_difference(self) -> np.ndarray:
        """Return relative minus absolute encoder position."""
        return self.relative_position - self.absolute_position


def _load_introspection_signal_names(directory: Path) -> list[str]:
    names_path = directory / "introspection_datanames.csv"
    values_path = directory / "introspection_datavalues.csv"
    if not names_path.is_file() or not values_path.is_file():
        raise FileNotFoundError(f"missing introspection CSV files in {directory}")

    with names_path.open(newline="") as names_file:
        headers = next(csv.reader(names_file), None)
    if headers is None or "names" not in headers:
        raise ValueError(f"invalid introspection name table: {names_path}")
    return [
        name.strip().removeprefix("- ")
        for name in headers[headers.index("names") + 1 :]
    ]


def load_backlash_joint_trajectory(
    directory: str | Path, joint_name: str
) -> BacklashJointTrajectory:
    """Load one joint's signals from historical TIAGo introspection exports."""
    directory = Path(directory)
    signal_names = _load_introspection_signal_names(directory)
    values_path = directory / "introspection_datavalues.csv"
    # Historical PAL introspection rows include one leading value with no name
    # in the datanames export. Keep the offset explicit to prevent silent
    # relative/absolute encoder misalignment.
    signal_indices = {
        name: index + 1
        for name, index in _required_signal_indices(signal_names, joint_name).items()
    }

    time: list[float] = []
    samples: dict[str, list[float]] = {name: [] for name in signal_indices}
    with values_path.open(newline="") as values_file:
        reader = csv.DictReader(values_file)
        for row in reader:
            values = np.fromstring(row["values"].strip().strip("[]"), sep=",")
            if values.size != len(signal_names):
                continue
            timestamp = float(row["secs"]) + float(row["nsecs"]) * 1e-9
            selected = {name: values[index] for name, index in signal_indices.items()}
            if not np.isfinite(timestamp) or not all(
                np.isfinite(value) for value in selected.values()
            ):
                continue
            if time and timestamp <= time[-1]:
                continue
            time.append(timestamp)
            for name, value in selected.items():
                samples[name].append(float(value))

    return BacklashJointTrajectory(
        joint_name=joint_name,
        time=np.asarray(time),
        relative_position=np.asarray(samples[f"{joint_name}_position"]),
        absolute_position=np.asarray(
            samples[f"{joint_name}_absolute_encoder_position"]
        ),
        velocity=np.asarray(samples[f"{joint_name}_velocity"]),
        effort=np.asarray(samples[f"{joint_name}_effort"]),
    )


def _required_signal_indices(
    signal_names: list[str], joint_name: str
) -> dict[str, int]:
    required_names = [
        f"{joint_name}_position",
        f"{joint_name}_absolute_encoder_position",
        f"{joint_name}_velocity",
        f"{joint_name}_effort",
    ]
    missing = [name for name in required_names if name not in signal_names]
    if missing:
        raise ValueError(f"missing joint signals for {joint_name}: {missing}")
    return {name: signal_names.index(name) for name in required_names}


# Joints whose logged position is used to reconstruct a full configuration
# for gravity-torque evaluation. Any model DOF not listed here (gripper,
# wheels) is held at the URDF neutral pose; TIAGo's arm/wrist gravity
# torque does not depend on those.
DEFAULT_CONFIGURATION_JOINT_NAMES = (
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
    "head_1_joint",
    "head_2_joint",
)


def load_backlash_joint_trajectory_with_gravity(
    directory: str | Path,
    joint_name: str,
    model,
    *,
    configuration_joint_names: tuple[str, ...] = DEFAULT_CONFIGURATION_JOINT_NAMES,
) -> tuple[BacklashJointTrajectory, np.ndarray]:
    """Load one joint's encoder signals plus its per-sample gravity torque.

    Reconstructs a full robot configuration at each sample from
    ``configuration_joint_names``' logged position signals (any other model
    DOF held at the URDF neutral pose) and evaluates
    ``pin.computeGeneralizedGravity`` for ``joint_name``. This reproduces
    the historical figaroh-plus ``SurfaceFitting.tau_g`` feature.

    Logged motor effort (:attr:`BacklashJointTrajectory.effort`, used by
    :func:`load_backlash_joint_trajectory`) is a poor stand-in for it on
    TIAGo's wrist joints (arm_5_joint..arm_7_joint): their effort signal is
    heavily quantized in this log (tens of unique values across thousands
    of samples), whereas true gravity torque is a smooth function of the
    full-arm configuration and varies substantially even where its average
    magnitude is small.
    """
    import pinocchio as pin

    directory = Path(directory)
    signal_names = _load_introspection_signal_names(directory)
    values_path = directory / "introspection_datavalues.csv"
    signal_indices = {
        name: index + 1
        for name, index in _required_signal_indices(signal_names, joint_name).items()
    }

    if not model.existJointName(joint_name):
        raise ValueError(f"joint not found in robot model: {joint_name}")
    configuration_columns = {}
    configuration_idx_q = {}
    for name in configuration_joint_names:
        position_signal = f"{name}_position"
        if position_signal not in signal_names:
            raise ValueError(f"missing configuration joint signal: {position_signal}")
        if not model.existJointName(name):
            raise ValueError(f"configuration joint not found in robot model: {name}")
        configuration_columns[name] = signal_names.index(position_signal) + 1
        configuration_idx_q[name] = model.joints[model.getJointId(name)].idx_q
    idx_v = model.joints[model.getJointId(joint_name)].idx_v

    neutral_configuration = pin.neutral(model)
    dynamics_data = model.createData()

    time: list[float] = []
    samples: dict[str, list[float]] = {name: [] for name in signal_indices}
    gravity_torque: list[float] = []
    with values_path.open(newline="") as values_file:
        reader = csv.DictReader(values_file)
        for row in reader:
            values = np.fromstring(row["values"].strip().strip("[]"), sep=",")
            if values.size != len(signal_names):
                continue
            timestamp = float(row["secs"]) + float(row["nsecs"]) * 1e-9
            selected = {name: values[index] for name, index in signal_indices.items()}
            configuration_values = {
                name: values[index] for name, index in configuration_columns.items()
            }
            if (
                not np.isfinite(timestamp)
                or not all(np.isfinite(value) for value in selected.values())
                or not all(
                    np.isfinite(value) for value in configuration_values.values()
                )
            ):
                continue
            if time and timestamp <= time[-1]:
                continue
            time.append(timestamp)
            for name, value in selected.items():
                samples[name].append(float(value))
            configuration = neutral_configuration.copy()
            for name, value in configuration_values.items():
                configuration[configuration_idx_q[name]] = value
            gravity = pin.computeGeneralizedGravity(
                model, dynamics_data, configuration
            )
            gravity_torque.append(float(gravity[idx_v]))

    trajectory = BacklashJointTrajectory(
        joint_name=joint_name,
        time=np.asarray(time),
        relative_position=np.asarray(samples[f"{joint_name}_position"]),
        absolute_position=np.asarray(
            samples[f"{joint_name}_absolute_encoder_position"]
        ),
        velocity=np.asarray(samples[f"{joint_name}_velocity"]),
        effort=np.asarray(samples[f"{joint_name}_effort"]),
    )
    return trajectory, np.asarray(gravity_torque)


@dataclass
class EmpiricalBacklashSurface:
    """Direction-gated polynomial encoder correction with fitted-domain checks."""

    degree: int = 2
    transition_steepness: float = 100.0
    coefficients: np.ndarray | None = field(default=None, init=False)
    position_range: tuple[float, float] | None = field(default=None, init=False)
    torque_range: tuple[float, float] | None = field(default=None, init=False)
    rank: int | None = field(default=None, init=False)
    condition_number: float | None = field(default=None, init=False)
    dropped_term_count: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        if self.degree < 0:
            raise ValueError("degree must be non-negative")
        if self.transition_steepness <= 0.0:
            raise ValueError("transition_steepness must be positive")

    def fit(
        self,
        position: np.ndarray,
        gravity_torque: np.ndarray,
        velocity: np.ndarray,
        encoder_difference: np.ndarray,
    ) -> "EmpiricalBacklashSurface":
        """Fit directional polynomial surfaces to relative-to-absolute encoder error."""
        position, gravity_torque, velocity, encoder_difference = self._validate_inputs(
            position, gravity_torque, velocity, encoder_difference
        )
        design = _polynomial_design(position, gravity_torque, self.degree)
        gate = self._direction_gate(velocity)
        combined = np.hstack((gate[:, None] * design, (1.0 - gate)[:, None] * design))

        # A term with a positive power of gravity_torque is the exact zero
        # vector whenever gravity_torque is (numerically) constant for
        # every sample -- e.g. a joint whose rotation axis does not couple
        # to gravity at all, such as TIAGo's arm_1_joint (near-vertical
        # shoulder-yaw axis: rotating it never changes any downstream
        # mass's height, so its gravity torque is exactly zero by
        # geometry, not by measurement). A coefficient on an all-zero
        # column is undetermined -- multiplying it by zero always gives
        # zero -- so it is dropped before solving rather than treated as a
        # generic rank deficiency that forces the whole degree down. This
        # keeps the genuinely well-posed position-only terms (which do not
        # depend on gravity_torque) fittable at the full requested degree.
        column_norms = np.linalg.norm(combined, axis=0)
        scale = column_norms.max()
        active = column_norms > scale * 1e-10 if scale > 0.0 else column_norms > 0.0
        reduced = combined[:, active]

        reduced_coefficients, _, rank, singular_values = np.linalg.lstsq(
            reduced, encoder_difference, rcond=None
        )
        if rank < reduced.shape[1]:
            raise ValueError("backlash surface design matrix is rank deficient")

        coefficients = np.zeros(combined.shape[1])
        coefficients[active] = reduced_coefficients

        self.coefficients = coefficients
        self.dropped_term_count = int(np.sum(~active))
        self.position_range = (float(position.min()), float(position.max()))
        self.torque_range = (
            float(gravity_torque.min()),
            float(gravity_torque.max()),
        )
        self.rank = int(rank)
        self.condition_number = (
            float(singular_values[0] / singular_values[-1])
            if singular_values[-1] > 0
            else float("inf")
        )
        return self

    def predict(
        self,
        position: np.ndarray,
        gravity_torque: np.ndarray,
        velocity: np.ndarray,
        *,
        allow_extrapolation: bool = False,
    ) -> np.ndarray:
        """Predict encoder correction, rejecting values outside the fitted domain."""
        if (
            self.coefficients is None
            or self.position_range is None
            or self.torque_range is None
        ):
            raise RuntimeError("fit the backlash surface before prediction")
        position, gravity_torque, velocity, _ = self._validate_inputs(
            position,
            gravity_torque,
            velocity,
            np.zeros_like(position, dtype=float),
        )
        if not allow_extrapolation and self._outside_domain(position, gravity_torque):
            raise ValueError("prediction is outside fitted domain")
        design = _polynomial_design(position, gravity_torque, self.degree)
        count = design.shape[1]
        gate = self._direction_gate(velocity)
        return gate * (design @ self.coefficients[:count]) + (1.0 - gate) * (
            design @ self.coefficients[count:]
        )

    def _validate_inputs(
        self,
        position: np.ndarray,
        gravity_torque: np.ndarray,
        velocity: np.ndarray,
        target: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        values = [
            np.asarray(value, dtype=float).reshape(-1)
            for value in (position, gravity_torque, velocity, target)
        ]
        if not values[0].size or any(
            value.shape != values[0].shape for value in values[1:]
        ):
            raise ValueError(
                "position, gravity_torque, velocity, and target must be "
                "equal-length vectors"
            )
        if not all(np.all(np.isfinite(value)) for value in values):
            raise ValueError("backlash surface inputs must be finite")
        return tuple(values)  # type: ignore[return-value]

    def _direction_gate(self, velocity: np.ndarray) -> np.ndarray:
        argument = np.clip(self.transition_steepness * velocity, -700.0, 700.0)
        return 1.0 / (1.0 + np.exp(-argument))

    def _outside_domain(self, position: np.ndarray, torque: np.ndarray) -> bool:
        return bool(
            np.any(
                (position < self.position_range[0])
                | (position > self.position_range[1])
            )
            or np.any((torque < self.torque_range[0]) | (torque > self.torque_range[1]))
        )
