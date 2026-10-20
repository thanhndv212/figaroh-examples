"""TIAGo mocap across hardware (#100): the library's building blocks.

Synthetic checks of examples/tiago/mocap_hardware.py (forward kinematics
against core, re-registration, leave-one-out, placements, effective noise).
No joint-level model is fitted on real data here.
"""

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
from figaroh.calibration.calibration_tools import calc_updated_fkm
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import mocap_hardware as mh  # noqa: E402
from examples.tiago.utils import square_motion_extraction as sme  # noqa: E402


@pytest.fixture(scope="module")
def sessions():
    return mh.load_sessions()


@pytest.fixture(scope="module")
def chain(sessions):
    # initialisation only (parameter names, model): nothing is fitted
    calib = mh.make_calib(sessions["vicon/2023-09-20-1508"].path, solve=False)
    return mh.Chain(calib)


@pytest.fixture(scope="module")
def postures(sessions):
    return mh.read_data(sessions["vicon/2023-09-20-1508"].path, six_d=True).q


def _offsets(seed=0):
    return np.random.default_rng(seed).normal(0.0, 0.03, 8)


def test_offsets_are_q_plus_offset_in_core(chain, postures):
    """FK(q + offset) on the nominal model equals core's updated FK."""
    off = _offsets()
    R, p = chain.tool_pose(postures, off)
    R2, p2 = chain.frame_pose(postures, off, "wrist_ft_tool_link")
    assert np.allclose(p, p2, atol=1e-9)
    assert np.allclose(R, R2, atol=1e-9)


def test_registered_frames_match_core_names_and_conventions(chain, postures):
    """Base (rpy) and tool point of a re-registration, in core's parameters."""
    off = _offsets(1)
    R, p = chain.tool_pose(postures, off)
    x = np.r_[
        Rotation.from_euler("xyz", [0.2, -0.1, 0.4]).as_rotvec(),
        [1.0, -2.0, 0.5],
        [0.06, 0.0, 0.07],
    ]
    pos, _ = mh._predict(R, p, x, False)
    params = dict(zip(mh.OFFSET_NAMES, off)) | mh.frames_to_params(x)
    cfg = dict(
        chain.cfg,
        param_name=list(params),
        NbSample=len(postures),
    )
    ref = calc_updated_fkm(
        chain.model,
        chain.data,
        np.array(list(params.values())),
        chain.configurations(postures),
        cfg,
    )
    assert np.allclose(ref.reshape(3, -1).T, pos, atol=1e-9)


def test_rpy_matrices_follow_pinocchio():
    rpy = np.random.default_rng(2).uniform(-1.0, 1.0, (5, 3))
    ref = np.array([pin.rpy.rpyToMatrix(r) for r in rpy])
    assert np.allclose(mh.rpy_to_matrices(rpy), ref, atol=1e-12)


def _session_like(chain, postures, six_d, seed=3):
    rng = np.random.default_rng(seed)
    off = _offsets(seed)
    R, p = chain.tool_pose(postures, off)
    x = np.r_[
        Rotation.from_euler("xyz", [1.2, -0.4, 2.0]).as_rotvec(),
        [7.9, 0.7, -1.8],
        [0.06, -0.003, 0.07],
    ]
    if six_d:
        x = np.r_[x, Rotation.from_euler("xyz", [0.05, -0.08, 0.1]).as_rotvec()]
    pos, rot = mh._predict(R, p, x, six_d)
    return R, p, x, pos, rot, rng


@pytest.mark.parametrize("six_d", [False, True])
def test_kabsch_and_reregister_recover_a_rigid_transform(chain, postures, six_d):
    R, p, x, pos, rot, _ = _session_like(chain, postures, six_d)
    x0 = mh.kabsch_init(p, pos, six_d)
    assert np.linalg.norm(mh._predict(R, p, x0, six_d)[0] - pos, axis=1).max() < 0.2
    got = mh.reregister(R, p, pos, rot)
    e_pos, e_rot = mh.residuals(R, p, got, pos, rot)
    assert np.abs(e_pos).max() < 1e-9
    if six_d:
        assert np.abs(e_rot).max() < 1e-8
    assert np.allclose(got[6:9], x[6:9], atol=1e-6)


def test_loo_equals_a_brute_force_loop(chain, postures):
    R, p, x, pos, _, rng = _session_like(chain, postures, False, seed=4)
    pos = pos + rng.normal(0.0, 2e-3, pos.shape)
    err, _ = mh.loo_errors(R, p, pos)
    brute = np.empty_like(err)
    for i in range(len(p)):
        keep = np.arange(len(p)) != i
        xi = mh.reregister(R[keep], p[keep], pos[keep])  # cold start
        brute[i] = mh.residuals(R[i : i + 1], p[i : i + 1], xi, pos[i : i + 1])[0][0]
    assert np.allclose(err, brute, atol=1e-6)


def test_score_reports_loo_above_insample_and_judges_against_truth(chain, postures):
    off = _offsets(5)
    R, p, x, clean, _, rng = _session_like(chain, postures, False, seed=5)
    noisy = clean + rng.normal(0.0, 2e-3, clean.shape)
    s = mh.score(chain, mh.Data(postures, noisy), off, truth=clean)
    assert s["loo_mm"] > s["insample_mm"]
    assert s["loo_vs_truth_mm"] < s["loo_mm"]  # the truth has no noise of its own


def test_six_d_metric_weights_a_milliradian_as_a_tenth_of_a_millimetre():
    pos = np.zeros((4, 3))
    rot = np.tile([1e-3, 0.0, 0.0], (4, 1))
    assert mh.rms_6d_mm(pos, rot) == pytest.approx(0.1)


def test_placement_model_round_trips_through_measure_applied_offsets():
    """/tf = M0 @ D @ Rz(q): the placement M0 @ D gives D back."""
    model = pin.buildModelFromUrdf(str(mh.URDF))
    rng = np.random.default_rng(6)
    q = rng.uniform(-1, 1, (3, 8))
    t = np.array([0.0, 1.0, 2.0])
    tf, applied = {}, {}
    for k, (joint, (_, child)) in enumerate(
        zip(sme.ARM_JOINTS, sme.ARM_PAIRS), start=1
    ):
        origin = model.jointPlacements[model.getJointId(joint)]
        d = np.round(rng.normal(0.0, 1e-3, 3), sme.OFFSET_DECIMALS)
        rv = np.round(rng.normal(0.0, 5e-3, 3), sme.OFFSET_DECIMALS)
        applied[joint] = {"translation_m": d.tolist(), "rotation_rad": rv.tolist()}
        rows = []
        for ti, qi in zip(t, q[:, k]):
            rot = (
                Rotation.from_matrix(origin.rotation)
                * Rotation.from_rotvec(rv)
                * Rotation.from_euler("z", qi)
            )
            pos = origin.translation + origin.rotation @ d
            rows.append(np.r_[ti, pos, rot.as_quat()])
        tf[child] = np.array(rows)
    got = sme.measure_applied_offsets(tf, t, q, str(mh.URDF))
    for joint, d in applied.items():
        assert np.allclose(got[joint]["translation_m"], d["translation_m"], atol=1e-9)
        assert np.allclose(got[joint]["rotation_rad"], d["rotation_rad"], atol=1e-9)
    placed = mh.placement_model(model, got)
    for joint, d in got.items():
        jid = model.getJointId(joint)
        want = model.jointPlacements[jid] * pin.SE3(
            pin.exp3(np.array(d["rotation_rad"])), np.array(d["translation_m"])
        )
        assert np.allclose(placed.jointPlacements[jid].homogeneous, want.homogeneous)
        assert not np.allclose(
            placed.jointPlacements[jid].homogeneous,
            model.jointPlacements[jid].homogeneous,
        )


def test_sigma_eff_inverts_the_pair_difference_variance():
    rng = np.random.default_rng(7)
    n, sigma = 4000, 1.5e-3
    a = rng.normal(0.0, 1.0, (n, 3))
    b = a + rng.normal(0.0, sigma, (n, 3)) - rng.normal(0.0, sigma, (n, 3))
    assert mh.sigma_eff(float(np.sum((a - b) ** 2)), n, 0) == pytest.approx(
        1.5, rel=0.03
    )
    assert mh.rigid_ssr(a[:50], a[:50] + [1.0, 2.0, 3.0]) < 1e-18
    with pytest.raises(ValueError):
        mh.sigma_eff(1.0, 2, 6)


def test_noise_level_is_the_smallest_grid_level_at_or_above():
    assert [mh.noise_level(s) for s in (0.3, 0.5, 0.6, 2.0, 3.0, 9.0)] == [
        0.5,
        0.5,
        2.0,
        2.0,
        4.0,
        4.0,
    ]


def test_effective_noise_of_the_three_systems_is_pinned(sessions):
    """Registration only; the values are written in the pre-registration."""
    est = mh.noise_estimates(sessions)
    assert {k: v["pairs"] for k, v in est.items()} == {
        "optitrack": 4,
        "vicon": 15,
        "qualisys": 109,
    }
    assert est["optitrack"]["sigma_eff_mm"] == pytest.approx(1.246, abs=0.002)
    assert est["vicon"]["sigma_eff_mm"] == pytest.approx(0.784, abs=0.002)
    assert est["qualisys"]["sigma_eff_mm"] == pytest.approx(0.572, abs=0.002)
    assert {v["noise_level_mm"] for v in est.values()} == {2.0}


def test_clock_lag_is_inside_the_plateau_margin():
    """Static rows carry no lag term: the lag error stays well inside a plateau."""
    assert max(mh.LAG_UNCERTAINTY_S.values()) < mh.LAG_MARGIN_S


def test_not_evaluated_session_is_excluded(sessions):
    assert not sessions["optitrack/2023-09-12-1529"].evaluated
    assert sessions["optitrack/2023-09-12-1552"].evaluated
    assert (
        len(mh.read_data(sessions["optitrack/2023-09-12-1529"].path)) < mh.MIN_POSTURES
    )
