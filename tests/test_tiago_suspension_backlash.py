"""Unit tests for experimental TIAGo suspension and backlash examples."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from pathlib import Path

from examples.tiago.utils.backlash_surface import EmpiricalBacklashSurface
from examples.tiago.utils.suspension_data import (
    SuspensionTrajectory,
    load_vicon_forceplate_trajectory,
)
from examples.tiago.utils.backlash_surface import load_backlash_joint_trajectory
from examples.tiago.utils.suspension_model import (
    build_generalized_base_regressor,
    fit_generalized_base_suspension,
)


def _excited_base_motion(
    sample_count: int = 80,
) -> tuple[np.ndarray, np.ndarray]:
    time = np.linspace(0.0, 2.0, sample_count)
    displacement = np.column_stack(
        [
            0.02 * np.sin(2.0 * np.pi * time),
            0.01 * np.cos(3.0 * np.pi * time),
            0.015 * np.sin(4.0 * np.pi * time),
            0.04 * np.sin(1.5 * np.pi * time),
            0.03 * np.cos(2.5 * np.pi * time),
            0.02 * np.sin(3.5 * np.pi * time),
        ]
    )
    velocity = np.gradient(displacement, time, axis=0, edge_order=2)
    return displacement, velocity


def test_suspension_regressor_recovers_known_coefficients() -> None:
    displacement, velocity = _excited_base_motion()
    expected = np.array(
        [
            1200.0,
            45.0,
            900.0,
            30.0,
            1500.0,
            55.0,
            180.0,
            8.0,
            220.0,
            9.0,
            140.0,
            7.0,
        ]
    )

    regressor = build_generalized_base_regressor(displacement, velocity)
    wrench = regressor @ expected
    result = fit_generalized_base_suspension(displacement, velocity, wrench)

    assert regressor.shape == (displacement.shape[0] * 6, 12)
    assert result.rank == 12
    np.testing.assert_allclose(result.parameters, expected, rtol=1e-10, atol=1e-10)
    assert result.rmse == pytest.approx(0.0, abs=1e-10)


def test_suspension_fit_rejects_rank_deficient_motion() -> None:
    displacement = np.zeros((12, 6))
    velocity = np.zeros((12, 6))
    wrench = np.zeros(72)

    with pytest.raises(ValueError, match="rank deficient"):
        fit_generalized_base_suspension(displacement, velocity, wrench)


def test_trajectory_contract_validates_time_and_wrench_shapes() -> None:
    time = np.array([0.0, 0.01, 0.02])
    trajectory = SuspensionTrajectory(
        time=time,
        base_displacement=np.zeros((3, 6)),
        base_velocity=np.zeros((3, 6)),
        wrench=np.zeros((3, 6)),
        frame="base_link",
    )

    assert trajectory.sample_count == 3
    assert trajectory.wrench_vector.shape == (18,)

    with pytest.raises(ValueError, match="strictly increasing"):
        SuspensionTrajectory(
            time=np.array([0.0, 0.01, 0.01]),
            base_displacement=np.zeros((3, 6)),
            base_velocity=np.zeros((3, 6)),
            wrench=np.zeros((3, 6)),
            frame="base_link",
        )


def test_real_suspension_log_loads_as_a_validated_trajectory() -> None:
    data_path = (
        Path(__file__).parent.parent
        / "examples"
        / "tiago"
        / "data"
        / "suspension"
        / "tiago_xyz_vicon_1640.csv"
    )

    trajectory = load_vicon_forceplate_trajectory(data_path)

    assert trajectory.sample_count > 100
    assert trajectory.frame == "vicon_world"
    assert trajectory.base_displacement.shape == (trajectory.sample_count, 6)
    assert trajectory.base_velocity.shape == (trajectory.sample_count, 6)
    assert trajectory.wrench.shape == (trajectory.sample_count, 6)
    assert np.all(np.diff(trajectory.time) > 0.0)


def test_real_backlash_log_loads_joint_encoder_signals() -> None:
    data_dir = (
        Path(__file__).parent.parent
        / "examples"
        / "tiago"
        / "data"
        / "backlash"
        / "sinus_amp3_period10_2023-07-24-13-28-42"
    )

    trajectory = load_backlash_joint_trajectory(data_dir, "arm_6_joint")

    assert trajectory.sample_count > 100
    assert trajectory.joint_name == "arm_6_joint"
    assert np.all(np.diff(trajectory.time) > 0.0)
    assert np.all(np.isfinite(trajectory.relative_position))
    assert np.all(np.isfinite(trajectory.absolute_position))
    assert np.all(np.isfinite(trajectory.velocity))
    assert np.all(np.isfinite(trajectory.effort))


@pytest.mark.parametrize(
    ("filename", "experiment_type"),
    [
        ("tiago_suspension_config.yaml", "tiago_generalized_base_suspension"),
        (
            "tiago_backlash_surface_config.yaml",
            "tiago_empirical_backlash_surface",
        ),
    ],
)
def test_experimental_configs_are_explicit_and_opt_in(
    filename: str, experiment_type: str
) -> None:
    config_path = (
        Path(__file__).parent.parent / "examples" / "tiago" / "config" / filename
    )
    with config_path.open() as config_file:
        config = yaml.safe_load(config_file)

    assert config["experiment"] == {
        "type": experiment_type,
        "status": "experimental",
    }
    assert config["data"]["units"] == "SI"
    assert config["solver"]["method"] == "linear_least_squares"


def test_empirical_backlash_surface_recovers_directional_linear_model() -> None:
    position = np.linspace(-0.8, 0.8, 100)
    gravity_torque = 2.0 * np.cos(position)
    velocity = np.where(np.arange(position.size) % 2 == 0, 0.2, -0.2)
    positive = 0.01 + 0.04 * position + 0.002 * gravity_torque
    negative = -0.02 + 0.03 * position - 0.001 * gravity_torque
    target = np.where(velocity > 0.0, positive, negative)

    model = EmpiricalBacklashSurface(degree=1, transition_steepness=100.0)
    model.fit(position, gravity_torque, velocity, target)

    np.testing.assert_allclose(
        model.predict(position, gravity_torque, velocity), target, atol=1e-10
    )


def test_empirical_backlash_surface_rejects_extrapolation_by_default() -> None:
    position = np.linspace(-0.5, 0.5, 24)
    torque = np.cos(np.linspace(0.0, 2.0 * np.pi, 24))
    velocity = np.where(np.arange(position.size) % 2 == 0, 0.1, -0.1)
    target = 0.02 * position + 0.01 * torque

    model = EmpiricalBacklashSurface(degree=1)
    model.fit(position, torque, velocity, target)

    with pytest.raises(ValueError, match="outside fitted domain"):
        model.predict(np.array([0.8]), np.array([0.0]), np.array([0.1]))
