"""TX40 trajectory design: the base regressor it optimises is well-posed."""

import os

import numpy as np
import pytest
import yaml

from examples.staubli_tx40.utils import staubli_tx40_tools as tools
from figaroh.tools.robot import load_robot

TX40_DIR = os.path.join(os.path.dirname(tools.__file__), os.pardir)


@pytest.fixture(scope="module")
def traj():
    cwd = os.getcwd()
    os.chdir(TX40_DIR)
    try:
        robot = load_robot(
            "urdf/tx40_mdh_modified.urdf", package_dirs="../../models", load_by_urdf=True
        )
        with open("config/staubli_tx40_unified_config.yaml") as f:
            cfg = yaml.safe_load(f)
        joints = cfg["robot"]["properties"]["joints"]["active_joints"]
        np.random.seed(0)
        t = tools.TX40OptimalTrajectory(robot, joints)
        t.initialize()
    finally:
        os.chdir(cwd)
    return t


def test_wrist_coupling_columns():
    N = 4
    W = np.zeros((6 * N, 2))
    v = np.arange(6 * N, dtype=float).reshape(N, 6)
    a = -v
    Wc = tools.add_wrist_coupling(W, N, v, a)
    assert Wc.shape == (6 * N, 5)
    np.testing.assert_array_equal(Wc[4 * N : 5 * N, 2], a[:, 5])
    np.testing.assert_array_equal(Wc[5 * N : 6 * N, 2], a[:, 4])
    np.testing.assert_array_equal(Wc[:4 * N, 2:], 0)


def test_base_parameters_exclude_collinear_columns(traj):
    # 58 base parameters of the identification model plus 3 coupled-wrist
    # terms. Core's absolute QR tolerance kept Izz_joint_2 next to Ia_joint_2.
    assert len(traj.idx_b) == 61


def test_random_segment_is_well_conditioned(traj):
    # Before the relative tolerance, every trajectory gave cond ~1e13.
    np.random.seed(1)
    n = 60
    q = np.random.uniform(-1, 1, (n, 6))
    v = np.random.uniform(-1, 1, (n, 6))
    a = np.random.uniform(-3, 3, (n, 6))
    W = traj._stack_base_regressors(q, v, a, None)
    assert np.linalg.cond(W) < 1e5
