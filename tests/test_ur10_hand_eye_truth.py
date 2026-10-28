"""UR10 hand-eye synthetic-truth study (#99).

Fast checks of the fixture: noise geometry, reproducibility, exact frame
recovery without noise or joint errors, and the dataset round-trip. The
seed-0 numbers and the noise-free held-out check are slow.
"""

import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.ur10 import hand_eye_heldout as heh  # noqa: E402
from examples.ur10 import hand_eye_truth as het  # noqa: E402

GOLDEN = ROOT / "tests" / "golden" / "ur10_hand_eye.json"


def test_camera_frame_noise_covariance_matches_the_sigmas():
    n = 200_000
    rot = Rotation.random(n, random_state=3).as_matrix()
    pos = np.zeros((n, 3))
    level = 2.0
    pos2, rot2 = het.add_noise(pos, rot, level, np.random.default_rng(5))
    cam = np.einsum("nji,nj->ni", rot, pos2 - pos)  # R^T dp: camera frame
    sig = level * np.array([het.SIGMA_XY, het.SIGMA_XY, het.SIGMA_Z])
    assert np.allclose(cam.std(axis=0), sig, rtol=0.02)
    assert np.allclose(np.corrcoef(cam.T) - np.eye(3), 0, atol=0.01)
    # rotation noise is in the fixed frame: exp(w) = R2 R^T
    w = Rotation.from_matrix(np.einsum("nij,nkj->nik", rot2, rot)).as_rotvec()
    sig_r = level * np.array([het.TILT, het.TILT, het.INPLANE])
    assert np.allclose(w.std(axis=0), sig_r, rtol=0.02)


def test_noise_rotation_whitens_the_cost_function_residual():
    """The fits' noise model is the simulated one (figaroh-plus#215).

    The cost function's residual of a noisy measurement, rotated into the
    sensor frames by ``noise_rotation`` (core: R_i^T r) and divided by
    ``residual_sigma``, has unit variance per component.
    """
    import pinocchio as pin

    n, level = 20_000, 2.0
    rot = Rotation.random(n, random_state=4).as_matrix()
    pos = np.random.default_rng(6).uniform(-1, 1, (n, 3))
    pos2, rot2 = het.add_noise(pos, rot, level, np.random.default_rng(7))
    rpy2 = np.array([pin.rpy.matrixToRpy(r) for r in rot2])
    R = heh.noise_rotation(np.hstack([pos2, rpy2]).T.ravel(), n)
    assert R.shape == (n, 6, 6)
    assert np.allclose(R @ np.swapaxes(R, 1, 2), np.eye(6), atol=1e-9)
    weight = np.r_[1.0, 1.0, 1.0, [heh.ORIENT_WEIGHT] * 3]
    r = np.array(
        [
            weight * pin.log(pin.SE3(m_r, m_p).inverse() * pin.SE3(t_r, t_p)).vector
            for m_r, m_p, t_r, t_p in zip(rot2, pos2, rot, pos)
        ]
    )
    sensor = np.einsum("ikc,ik->ic", R, r)
    z = sensor / np.array(het.residual_sigma(level))
    assert np.allclose(z.std(axis=0), 1.0, rtol=0.03)
    assert np.allclose(np.corrcoef(z.T) - np.eye(6), 0, atol=0.03)


def test_fit_estimation_gives_the_regularised_fits_the_noise():
    est = {"method": "map", "priors": heh.PRIORS}
    assert het.fit_estimation(est, 2.0)["noise_std"] == het.residual_sigma(2.0)
    assert het.fit_estimation(est, 2.0, estimated=True)["noise_std"] == (
        "per_component"
    )
    assert "noise_std" not in est  # not modified in place
    structural = {"method": "structural"}
    assert het.fit_estimation(structural, 2.0) is structural
    assert het.fit_estimation(est, 0.0) is est  # nothing to whiten by


def test_simulation_is_reproducible_and_seed_dependent():
    st = het.study()
    q = het.training_postures()
    truth = het.draw_truth("full_params", np.random.default_rng(0))
    a, _ = st.simulate(st, truth, "full_params", q, 1.0, 0)
    b, _ = st.simulate(st, truth, "full_params", q, 1.0, 0)
    c, _ = st.simulate(st, truth, "full_params", q, 1.0, 1)
    assert np.array_equal(a, b)
    assert not np.allclose(a, c)
    again = het.draw_truth("full_params", np.random.default_rng(0))
    assert truth == again


def test_heldout_postures_are_seeded_and_in_range():
    q = het.heldout_postures()
    real = het.training_postures()
    assert q.shape == (het.N_HELD, 6)
    assert (q.min() >= real.min()).all() and (q.max() <= real.max()).all()
    pd.testing.assert_frame_equal(q, het.heldout_postures())


def test_dataset_round_trip():
    st = het.study()
    q = het.training_postures()
    truth = het.draw_truth("full_params", np.random.default_rng(2))
    meas = st.predict(truth, "full_params", q)
    df = st.to_dataset(meas, q)
    assert list(df.columns) == het.POSE_COLUMNS + heh.JOINTS
    pos, rot = het.decode(meas, het._reference(_probe(), q))
    rpy_rot = het._rotations(df[het.POSE_COLUMNS[3:]].to_numpy())
    assert np.allclose(df[het.POSE_COLUMNS[:3]].to_numpy(), pos, atol=1e-12)
    assert np.allclose(rpy_rot, rot, atol=1e-9)


def _probe():
    return heh.fit(heh.TRAINING, "full_params", True, het.STRUCTURAL)


def test_frames_match_a_fresh_registration_fit():
    calib = _probe()
    fresh = dict(zip(calib.calib_config["param_name"], calib.LM_result.x))
    for name, value in het.FRAMES.items():
        assert fresh[name] == pytest.approx(value, abs=2e-6)


def test_noise_free_registration_recovers_the_frames():
    """No joint errors, no noise: the registration fit returns FRAMES."""
    st = het.study()
    q = het.training_postures()
    meas = st.predict(dict(het.FRAMES), "full_params", q)
    with tempfile.TemporaryDirectory() as tmp:
        fit = st.fit(st.to_dataset(meas, q), "registration only", None, Path(tmp))
    for name, value in het.FRAMES.items():
        assert fit.params[name] == pytest.approx(value, abs=1e-9)


@pytest.mark.slow
def test_noise_free_structural_fit_predicts_the_truth():
    res = het.run_case("full_params", "full_params", 0, 0.0, tempfile.mkdtemp())
    assert res["heldout_rmse_mm"] < 0.01


@pytest.mark.slow
def test_seed0_matches_golden():
    golden = json.loads(GOLDEN.read_text())["simulation_seed0"]
    df = het.run(seeds=(0,), noise=(1.0,))
    for row in golden:
        got = df[df["fit"] == row["fit"]].iloc[0]
        assert got["n_params"] == row["n_params"]
        assert got["heldout_rmse_mm"] == pytest.approx(row["heldout_rmse_mm"], abs=2e-3)
        assert got["heldout_rot_deg"] == pytest.approx(row["heldout_rot_deg"], abs=2e-3)
