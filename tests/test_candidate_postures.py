"""Sampled candidates of the TX40 and TALOS posture designs (Phase 7).

The sampler is seeded, moves only the calibrated chain, stays inside the URDF
limits less the configured margin, and with collision checking returns no
self-colliding posture.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

from examples.candidate_postures import (
    chain_joint_names,
    configured_margin,
    sample_chain_postures,
)
from figaroh.tools.robot import load_robot

ROOT = Path(__file__).resolve().parents[1]
MODELS = str(ROOT / "models")
TX40 = ROOT / "examples" / "staubli_tx40"
TALOS = ROOT / "examples" / "talos"
TALOS_SRDF = str(ROOT / "models" / "talos_description" / "srdf" / "talos.srdf")


@pytest.fixture(scope="module")
def tx40():
    return load_robot(
        str(TX40 / "urdf" / "tx40_mdh_modified.urdf"),
        package_dirs=MODELS,
        load_by_urdf=True,
    )


@pytest.fixture(scope="module")
def talos():
    return load_robot(
        str(TALOS / "urdf" / "talos_full_v2.urdf"),
        package_dirs=MODELS,
        load_by_urdf=True,
    )


def test_same_seed_same_pool_and_another_seed_differs(tx40):
    a = sample_chain_postures(tx40, range(6), 20, seed=3)
    b = sample_chain_postures(tx40, range(6), 20, seed=3)
    c = sample_chain_postures(tx40, range(6), 20, seed=4)
    np.testing.assert_array_equal(a.q, b.q)
    assert not np.allclose(a.q, c.q)


def test_samples_stay_inside_the_limits_less_the_margin(tx40):
    m = tx40.model
    pool = sample_chain_postures(tx40, range(6), 500, margin=0.1)
    assert pool.q.shape == (500, m.nq)
    assert np.all(pool.q >= m.lowerPositionLimit + 0.1)
    assert np.all(pool.q <= m.upperPositionLimit - 0.1)


def test_only_the_chain_moves_and_names_follow_idx_q(talos):
    idx = list(range(12, 21))
    pool = sample_chain_postures(talos, idx, 30)
    others = np.setdiff1d(np.arange(talos.model.nq), idx)
    np.testing.assert_array_equal(
        pool.q[:, others], np.tile(talos.q0, (30, 1))[:, others]
    )
    assert pool.joint_names[:2] == ["torso_1_joint", "torso_2_joint"]
    assert pool.joint_names[2:] == [f"arm_left_{i}_joint" for i in range(1, 8)]
    cfg = pool.as_configs()
    assert np.allclose(cfg["calibration_joint_configurations"], pool.q[:, idx])


def test_srdf_collision_check_rejects_and_keeps_none_colliding(talos):
    idx = list(range(12, 21))
    pool = sample_chain_postures(talos, idx, 40, collision="srdf", srdf=TALOS_SRDF)
    assert pool.n_rejected > 0  # about half of the uniform arm samples collide
    geom = talos.collision_model.copy()
    geom.addAllCollisionPairs()
    pin.removeCollisionPairs(talos.model, geom, TALOS_SRDF)
    data, gdata = talos.model.createData(), geom.createData()
    for q in pool.q:
        assert not pin.computeCollisions(talos.model, data, geom, gdata, q, True)


def test_bad_inputs_fail_loudly(tx40, talos):
    with pytest.raises(ValueError, match="collision"):
        sample_chain_postures(tx40, range(6), 5, collision="floor")
    with pytest.raises(ValueError, match="SRDF"):
        sample_chain_postures(tx40, range(6), 5, collision="srdf")
    with pytest.raises(ValueError, match="no range"):
        sample_chain_postures(tx40, range(6), 5, margin=10.0)
    with pytest.raises(ValueError, match="one-DOF"):
        chain_joint_names(tx40.model, [99])
    with pytest.raises(RuntimeError, match="collision-free"):
        sample_chain_postures(
            talos, range(12, 21), 50, collision="srdf", srdf=TALOS_SRDF, max_draws=10
        )


def test_margin_is_read_from_the_robot_configs():
    assert (
        configured_margin(str(TX40 / "config" / "staubli_tx40_unified_config.yaml"))
        == 0.05
    )
    assert (
        configured_margin(str(TALOS / "config" / "talos_unified_config.yaml")) == 0.05
    )


@pytest.mark.parametrize(
    "folder, module, cls, nq_chain",
    [
        (
            TX40,
            "examples.staubli_tx40.utils.staubli_tx40_tools",
            "TX40OptimalCalibration",
            6,
        ),
        (TALOS, "examples.talos.utils.talos_tools", "TALOSOptimalCalibration", 9),
    ],
)
def test_design_classes_sample_the_calibration_chain(
    folder, module, cls, nq_chain, monkeypatch
):
    import importlib

    monkeypatch.chdir(folder)
    urdf = next((folder / "urdf").glob("*.urdf"))
    robot = load_robot(str(urdf), package_dirs=MODELS, load_by_urdf=True)
    config = next((folder / "config").glob("*_unified_config.yaml"))
    opt = getattr(importlib.import_module(module), cls)(
        robot, str(config), n_candidates=25, seed=1
    )
    opt.load_candidate_configurations()
    assert opt.calib_config["NbSample"] == 25
    assert len(opt.candidate_pool.idx_q) == nq_chain
    assert list(opt.candidate_pool.idx_q) == list(opt.calib_config["config_idx"])
    assert opt.margin == 0.05
