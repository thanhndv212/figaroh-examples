"""Generalized-base linear spring-damper model for the TIAGo example."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SuspensionFitResult:
    """Result of fitting the 12-parameter generalized-base suspension model."""

    parameters: np.ndarray
    rank: int
    condition_number: float
    rmse: float
    predicted_wrench: np.ndarray


def _validate_motion(
    displacement: np.ndarray, velocity: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    displacement = np.asarray(displacement, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    if displacement.ndim != 2 or displacement.shape[1] != 6:
        raise ValueError("displacement must have shape (n_samples, 6)")
    if velocity.shape != displacement.shape:
        raise ValueError("velocity must have the same shape as displacement")
    if displacement.shape[0] < 2:
        raise ValueError("at least two motion samples are required")
    if not np.all(np.isfinite(displacement)) or not np.all(
        np.isfinite(velocity)
    ):
        raise ValueError("motion values must be finite")
    return displacement, velocity


def build_generalized_base_regressor(
    displacement: np.ndarray, velocity: np.ndarray
) -> np.ndarray:
    """Build the DOF-major regressor for diagonal translational/rotational springs.

    Parameter order is ``[kx, cx, ky, cy, kz, cz, krx, crx, kry, cry, krz, crz]``.
    """
    displacement, velocity = _validate_motion(displacement, velocity)
    sample_count = displacement.shape[0]
    regressor = np.zeros((sample_count * 6, 12), dtype=float)

    for dof in range(6):
        row_slice = slice(dof * sample_count, (dof + 1) * sample_count)
        column = dof * 2
        regressor[row_slice, column] = displacement[:, dof]
        regressor[row_slice, column + 1] = velocity[:, dof]
    return regressor


def fit_generalized_base_suspension(
    displacement: np.ndarray, velocity: np.ndarray, wrench: np.ndarray
) -> SuspensionFitResult:
    """Fit the generalized-base spring-damper model with linear least squares."""
    regressor = build_generalized_base_regressor(displacement, velocity)
    wrench = np.asarray(wrench, dtype=float).reshape(-1)
    if wrench.shape != (regressor.shape[0],):
        raise ValueError(
            f"wrench must have shape ({regressor.shape[0]},), got {wrench.shape}"
        )
    if not np.all(np.isfinite(wrench)):
        raise ValueError("wrench must contain only finite values")

    parameters, _, rank, singular_values = np.linalg.lstsq(
        regressor, wrench, rcond=None
    )
    if rank < regressor.shape[1]:
        raise ValueError(
            f"suspension regressor is rank deficient: rank {rank} < 12"
        )

    predicted = regressor @ parameters
    condition_number = singular_values[0] / singular_values[-1]
    return SuspensionFitResult(
        parameters=parameters,
        rank=rank,
        condition_number=float(condition_number),
        rmse=float(np.sqrt(np.mean((predicted - wrench) ** 2))),
        predicted_wrench=predicted,
    )
