"""Validated data containers for the experimental TIAGo suspension example."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SuspensionTrajectory:
    """Synchronized generalized-base motion and wrench measurements in SI units."""

    time: np.ndarray
    base_displacement: np.ndarray
    base_velocity: np.ndarray
    wrench: np.ndarray
    frame: str

    def __post_init__(self) -> None:
        time = np.asarray(self.time, dtype=float)
        displacement = np.asarray(self.base_displacement, dtype=float)
        velocity = np.asarray(self.base_velocity, dtype=float)
        wrench = np.asarray(self.wrench, dtype=float)

        if time.ndim != 1 or time.size < 2:
            raise ValueError(
                "time must be a one-dimensional array with at least two samples"
            )
        if not np.all(np.isfinite(time)) or np.any(np.diff(time) <= 0.0):
            raise ValueError("time must be finite and strictly increasing")
        for name, values in {
            "base_displacement": displacement,
            "base_velocity": velocity,
            "wrench": wrench,
        }.items():
            if values.shape != (time.size, 6):
                raise ValueError(
                    f"{name} must have shape ({time.size}, 6), got {values.shape}"
                )
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must contain only finite values")
        if not self.frame:
            raise ValueError("frame must be a non-empty frame identifier")

        object.__setattr__(self, "time", time)
        object.__setattr__(self, "base_displacement", displacement)
        object.__setattr__(self, "base_velocity", velocity)
        object.__setattr__(self, "wrench", wrench)

    @property
    def sample_count(self) -> int:
        """Return the synchronized sample count."""
        return self.time.size

    @property
    def wrench_vector(self) -> np.ndarray:
        """Return a DOF-major wrench vector: Fx, Fy, Fz, Mx, My, Mz."""
        return self.wrench.reshape(-1, order="F")


def load_vicon_forceplate_trajectory(path: str | Path) -> SuspensionTrajectory:
    """Load the historical TIAGo Vicon and force-plate CSV export.

    The source file has no header and uses the historical column order: timestamp,
    ten marker triplets, force, moment, and center of pressure. The fitted model
    uses base marker 1 as the translational origin and the base 1/2/3 marker triad
    for relative roll-pitch-yaw displacement.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Vicon force-plate log not found: {path}")

    rows: list[list[str]] = []
    with path.open(newline="") as log_file:
        for row in csv.reader(log_file):
            if not row:
                continue
            if len(row) != 40:
                raise ValueError(f"expected 40 Vicon columns in {path}, got {len(row)}")
            rows.append(row)
    if len(rows) < 3:
        raise ValueError(f"Vicon force-plate log requires at least three rows: {path}")

    time = np.array(
        [datetime.fromisoformat(row[0]).timestamp() for row in rows], dtype=float
    )
    values = np.asarray([[float(value) for value in row[1:]] for row in rows])
    if np.any(np.diff(time) <= 0.0):
        raise ValueError(f"Vicon timestamps must be strictly increasing: {path}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Vicon log contains non-finite measurements: {path}")

    base_1 = values[:, 0:3]
    base_2 = values[:, 3:6]
    base_3 = values[:, 6:9]
    rotation = _base_marker_rotation(base_1, base_2, base_3)
    relative_rotation = np.einsum("ji,njk->nik", rotation[0], rotation)
    orientation = np.unwrap(_rotation_to_rpy(relative_rotation), axis=0)
    displacement = np.column_stack((base_1 - base_1[0], orientation))
    velocity = np.gradient(displacement, time, axis=0, edge_order=2)
    wrench = values[:, 30:36]
    wrench = wrench - np.mean(wrench[: min(100, len(wrench))], axis=0)

    return SuspensionTrajectory(
        time=time,
        base_displacement=displacement,
        base_velocity=velocity,
        wrench=wrench,
        frame="vicon_world",
    )


def _base_marker_rotation(
    base_1: np.ndarray, base_2: np.ndarray, base_3: np.ndarray
) -> np.ndarray:
    x_axis = _normalize_rows(base_2 - base_1, "base 1 to base 2")
    z_axis = _normalize_rows(np.cross(x_axis, base_3 - base_1), "base marker triad")
    y_axis = _normalize_rows(np.cross(z_axis, x_axis), "base marker triad")
    return np.stack((x_axis, y_axis, z_axis), axis=2)


def _normalize_rows(vectors: np.ndarray, name: str) -> np.ndarray:
    norms = np.linalg.norm(vectors, axis=1)
    if np.any(norms <= np.finfo(float).eps):
        raise ValueError(f"degenerate {name} marker geometry")
    return vectors / norms[:, None]


def _rotation_to_rpy(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[:, 2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[:, 2, 1], rotation[:, 2, 2])
    yaw = np.arctan2(rotation[:, 1, 0], rotation[:, 0, 0])
    return np.column_stack((roll, pitch, yaw))
