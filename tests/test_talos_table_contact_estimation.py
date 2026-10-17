"""Name-driven parameter handling of the TALOS table-contact calibration
(figaroh-examples#98): the parameter vector is read by name from
``calib_config["param_name"]``, not by position, and the sample count
follows ``q_measured``. The structural/ordered numbers must not move.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

EXAMPLE_DIR = PROJECT_ROOT / "examples" / "talos_table_contact"
if str(EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_DIR))

pytest.importorskip("pinocchio")

from run_calibration_real_data import (  # noqa: E402
    DATA_DIR,
    LEFT_CONFIG,
    RIGHT_CONFIG,
    _build_chain,
    _load_real_touches,
    _load_split,
    two_chain_metrics,
)
from generate_synthetic_data import _load_robot  # noqa: E402


@pytest.fixture(scope="module")
def robot():
    return _load_robot()


def test_param_order_does_not_change_the_gap(robot):
    calib = _build_chain(robot, LEFT_CONFIG, DATA_DIR / "left_train.csv")
    names = list(calib.calib_config["param_name"])
    rng = np.random.default_rng(0)
    var = rng.normal(0.0, 1e-3, len(names))
    gap = calib.get_pose_from_measure(var)
    split = calib.split_params(var)

    perm = rng.permutation(len(names))
    calib.calib_config["param_name"] = [names[i] for i in perm]
    gap_perm = calib.get_pose_from_measure(var[perm])
    assert np.allclose(gap, gap_perm, rtol=0, atol=1e-12)
    assert calib.split_params(var[perm]) == split

    # tail only (plane + contact) reversed
    n_dx = calib.n_deltaX
    tail = list(range(n_dx, len(names)))[::-1]
    order = list(range(n_dx)) + tail
    calib.calib_config["param_name"] = [names[i] for i in order]
    assert np.allclose(gap, calib.get_pose_from_measure(var[order]), rtol=0, atol=1e-12)


def test_sample_count_follows_q_measured(robot):
    calib = _build_chain(robot, LEFT_CONFIG, DATA_DIR / "left_train.csv")
    var = np.random.default_rng(1).normal(
        0.0, 1e-3, len(calib.calib_config["param_name"])
    )
    full = calib.get_pose_from_measure(var).reshape(3, -1)
    n = 7
    calib.q_measured = calib.q_measured[:n]
    calib.PEE_measured = np.zeros(3 * n)
    calib.calib_config["NbSample"] = n  # as core estimation._samples does
    calib.session_ids = calib.session_ids[:n]
    sub = calib.get_pose_from_measure(var).reshape(3, -1)
    assert sub.shape == (3, n)
    assert np.allclose(sub, full[:, :n], rtol=0, atol=1e-12)
    assert calib.gap_rms(var) == pytest.approx(
        np.sqrt(np.mean(calib.cost_function(var)[: 3 * n] ** 2))
    )


@pytest.mark.slow
def test_structural_unchanged(robot):
    """Real data, structural + ordered selection: held-out z and
    parameter counts as before the name-driven split."""
    held_z = {}
    n_dx = {}
    calibs = {}
    for side, cfg in (("left", LEFT_CONFIG), ("right", RIGHT_CONFIG)):
        calib = _build_chain(robot, cfg, DATA_DIR / f"{side}_train.csv")
        result = calib.solve(
            method="lm", max_iterations=3, enable_logging=False, html_report=False
        )
        n_dx[side] = calib.n_deltaX
        _load_split(calib, _load_real_touches(DATA_DIR / f"{side}_validation.csv"))
        held_z[side] = calib.gap_metrics(result.x)["z_rmse_mm"]
        calibs[side] = calib

    assert n_dx == {"left": 50, "right": 57}
    assert held_z["left"] == pytest.approx(7.55, abs=0.005)
    assert held_z["right"] == pytest.approx(7.69, abs=0.005)

    two = two_chain_metrics(robot)
    assert two["left_z_rmse_mm"] == pytest.approx(5.098, abs=0.005)
    assert two["right_z_rmse_mm"] == pytest.approx(8.394, abs=0.005)


# ---------------------------------------------------------------------------
# Estimation methods and the table-only baseline (figaroh-examples#98,
# figaroh-plus#210)
# ---------------------------------------------------------------------------
TAIL = [
    "plane_z_s0",
    "plane_phix_s0",
    "plane_thetay_s0",
    "contact_phix",
    "contact_thetay",
]
SMALL_CV = {"noise_std": 1e-3, "cv_folds": 3, "cv_multipliers": [1.0]}


def _left(robot, **kw):
    return _build_chain(robot, LEFT_CONFIG, DATA_DIR / "left_train.csv", **kw)


def _check_weights(calib):
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    w = cfg["prior_weights"]  # None for excitation (no prior rows)
    assert "contact_z" not in names
    for n in TAIL:
        assert n in names
    if w is not None:
        w = np.asarray(w)
        assert len(w) == len(names)
        for n in TAIL:
            assert w[names.index(n)] == 0.0
        assert np.all(w[: calib.n_deltaX] > 0)
    assert calib.nuisance_param_names() == TAIL
    assert calib.n_deltaX == len(names) - len(TAIL)
    assert calib._fk_config["param_name"] == names[: calib.n_deltaX]


def test_map_initialises(robot):
    calib = _left(robot, estimation=dict(SMALL_CV, method="map"))
    _check_weights(calib)
    assert calib.calib_config["prior_weights"] is not None
    assert calib.regularization_coefficient == 0.0
    # no ridge rows: the cost is the gap alone
    var = np.zeros(len(calib.calib_config["param_name"]))
    assert calib.cost_function(var).shape == (3 * len(calib.q_measured),)


@pytest.mark.slow
def test_other_methods_initialise(robot):
    for method in ("excitation", "map_cv", "cv_subset"):
        calib = _left(robot, estimation=dict(SMALL_CV, method=method))
        _check_weights(calib)


def test_non_structural_needs_one_session(robot, tmp_path):
    import pandas as pd

    df = pd.read_csv(DATA_DIR / "left_train.csv")
    df["session_id"] = np.arange(len(df)) % 2
    csv = tmp_path / "two_sessions.csv"
    df.to_csv(csv, index=False)
    from utils.talos_table_tools import TalosTableContactCalibration
    from generate_synthetic_data import NOMINAL_CONTACT_OFFSET, NOMINAL_TABLE_POSE

    calib = TalosTableContactCalibration(robot, str(LEFT_CONFIG))
    calib.calib_config["estimation"] = dict(SMALL_CV, method="map")
    calib._data_path = str(csv)
    calib.set_nominal_table_poses([NOMINAL_TABLE_POSE] * 2)
    calib.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
    with pytest.raises(ValueError, match="single session"):
        calib.initialize()


def test_exact_jacobian_matches_finite_difference(robot):
    calib = _left(robot)
    names = list(calib.calib_config["param_name"])
    rng = np.random.default_rng(3)
    fitted = calib.solve(
        method="lm", max_iterations=1, enable_logging=False, html_report=False
    ).x
    for x in (np.zeros(len(names)), fitted, rng.normal(0.0, 2e-3, len(names))):
        exact = calib.measurement_jacobian(names, x)
        calib._fd_jacobian = True
        try:
            fd = calib.measurement_jacobian(names, x)
        finally:
            calib._fd_jacobian = False
        assert exact.shape == (3 * len(calib.q_measured), len(names))
        assert np.linalg.norm(exact - fd) / np.linalg.norm(fd) < 1e-7


def test_table_contact_baseline(robot):
    calib = _left(robot, fit_level="table_contact")
    assert calib.calib_config["param_name"] == TAIL
    assert calib.n_deltaX == 0
    assert calib.regularization_coefficient == 0.0
    result = calib.solve(
        method="lm", max_iterations=1, enable_logging=False, html_report=False
    )
    assert np.all(np.isfinite(result.x)) and len(result.x) == 5
    m = calib.gap_metrics(result.x)
    assert all(np.isfinite(v) for v in m.values())
    # a full Delta X fit cannot do worse on the training postures
    assert m["z_rmse_mm"] > 1.0
