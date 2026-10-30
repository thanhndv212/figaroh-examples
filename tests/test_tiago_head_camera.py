"""TIAGo head camera (#101): the library of the study (``head_camera.py``).

Roles, the kinematic model against core, the gauge on the real postures, the
residual convention and the registration-only noise stage. No joint offset is
fitted to real data.
"""

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import head_camera as hc  # noqa: E402


@pytest.fixture(scope="module")
def sessions():
    return hc.load_sessions()


@pytest.fixture(scope="module")
def kin():
    return hc.Kinematics()


def test_roles_follow_the_protocol(sessions):
    roles = hc.role_sets(sessions)
    assert [s.id for s in roles["training"]] == ["2023-11-27-center"]
    assert [s.id for s in roles["validation"]] == ["2023-11-23-distortion_original"]
    assert sorted(s.id for s in roles["confirmation"]) == [
        f"2023-10-27-test{i}" for i in (1, 2, 3)
    ]
    # another robot: not evaluated, in no role
    every = {s.id for v in roles.values() for s in v}
    assert "2023-11-24-test_optimal" not in every
    assert "2023-11-27-topleft" in {s.id for s in roles["diagnostic"]}


def test_paired_rows_of_the_10_27_runs(sessions):
    roles = hc.role_sets(sessions)
    kept = {s.id: len(hc.paired_rows(s)) for s in roles["confirmation"]}
    assert kept == {
        "2023-10-27-test1": 11,
        "2023-10-27-test2": 11,
        "2023-10-27-test3": 8,
    }


def test_confirmation_values_are_not_readable(sessions):
    roles = hc.role_sets(sessions)
    conf = roles["confirmation"][0]
    with pytest.raises(PermissionError):
        hc.read_camera(conf)
    # the posture plan is not a measurement
    assert hc.read_plan(conf).shape == (12, 10)
    with pytest.raises(ValueError, match="confirmation"):
        hc.select_model({"m": {}}, "rmse", "confirmation")


def test_noise_stage_refuses_confirmation_sessions(sessions, monkeypatch):
    bad = dict(hc.SETTINGS, noise_pair=["2023-10-27-test1", "2023-11-27-center"])
    monkeypatch.setattr(hc, "SETTINGS", bad)
    with pytest.raises(PermissionError):
        hc.noise_estimates(sessions)


def test_kinematics_match_core_on_the_arm_chain(kin):
    """Arm pose at q + offsets equals core's joint_offset FK (z axes, sign +1)."""
    from figaroh.calibration.calibration_tools import calc_updated_fkm

    rng = np.random.default_rng(0)
    model = kin.model
    data = model.createData()
    n = 5
    plan = hc.read_plan(hc.role_sets()["training"][0])[:n]
    off = np.zeros(10)
    off[hc.ARM] = rng.normal(0, 0.02, 7)
    q = np.tile(pin.neutral(model), (n, 1))
    q[:, kin.idx] = plan
    arm = [model.getJointId(j) for j in hc.JOINTS[3:]]
    cfg = dict(
        calib_model="joint_offset",
        start_frame="universe",
        end_frame=hc.HAND_FRAME,
        base_to_ref_frame=None,
        NbMarkers=1,
        calibration_index=6,
        NbSample=n,
        actJoint_idx=arm,
        param_name=[f"offsetRZ_{model.names[j]}" for j in arm],
        measurability=[True] * 6,
    )
    core = calc_updated_fkm(model, data, off[hc.ARM], q, cfg).reshape(6, n)
    _, Ta = kin.poses(plan, off)
    assert np.allclose(core[:3].T, Ta[:, :3, 3], atol=1e-12)
    rpy = np.array([pin.rpy.matrixToRpy(T[:3, :3]) for T in Ta])
    assert np.allclose(core[3:].T, rpy, atol=1e-12)


def test_kinematics_match_core_eye_hand_anchor(kin):
    """Core's head-camera anchor follows each sample (figaroh-plus#217).

    With ``base_to_ref_frame`` at the camera, core returns
    ``T_cam(q+d) S^-1 T_cam(q+d)^-1 T_hand(q+d)``, ``S`` the camera pose
    parameters; before #217 the anchor came from one stale configuration.
    """
    from figaroh.calibration.calibration_tools import (
        calc_updated_fkm,
        cartesian_to_SE3,
    )

    rng = np.random.default_rng(0)
    model = kin.model
    data = model.createData()
    n = 5
    plan = hc.read_plan(hc.role_sets()["training"][0])[:n]
    off = np.zeros(10)
    off[1:] = rng.normal(0, 0.02, 9)  # head_1, head_2 and the arm
    base = np.concatenate([rng.normal(0, 0.01, 3), rng.normal(0, 0.02, 3)])
    q = np.tile(pin.neutral(model), (n, 1))
    q[:, kin.idx] = plan
    moved = [model.getJointId(j) for j in hc.JOINTS[1:]]
    cfg = dict(
        calib_model="joint_offset",
        start_frame="universe",
        end_frame=hc.HAND_FRAME,
        base_to_ref_frame=hc.CAMERA_FRAME,
        ref_frame=hc.CAMERA_FRAME,
        NbMarkers=1,
        calibration_index=6,
        NbSample=n,
        actJoint_idx=moved,
        param_name=[
            "base_px",
            "base_py",
            "base_pz",
            "base_phix",
            "base_phiy",
            "base_phiz",
        ]
        + [f"offsetRZ_{model.names[j]}" for j in moved],
        measurability=[True] * 6,
    )
    # the caller's last forward kinematics is the last sample, as in #217
    pin.framesForwardKinematics(model, data, q[-1])
    var = np.concatenate([base, off[1:]])
    core = calc_updated_fkm(model, data, var, q, cfg).reshape(6, n)

    Tc, Ta = kin.poses(plan, off)
    S_inv = cartesian_to_SE3(base).inverse().homogeneous
    expected = Tc @ S_inv @ np.linalg.inv(Tc) @ Ta
    assert np.allclose(core[:3].T, expected[:, :3, 3], atol=1e-12)
    rpy = np.array([pin.rpy.matrixToRpy(T[:3, :3]) for T in expected])
    assert np.allclose(core[3:].T, rpy, atol=1e-12)


def test_residual_is_a_rotation_vector_in_the_camera_frame():
    w = np.array([0.02, -0.01, 0.03])
    Rm = Rotation.random(3, random_state=0).as_matrix()
    Rp = Rotation.from_rotvec(w).as_matrix() @ Rm[0]
    M = np.tile(np.eye(4), (1, 1, 1))
    M[0, :3, :3] = Rp
    obs = hc.Obs(np.zeros((1, 10)), Rm[:1], np.zeros((1, 3)))
    r = hc.raw_residuals(M, obs)
    assert np.allclose(r[0, 3:], w, atol=1e-12)
    # angles near +-pi never enter: the rotation matrix is the measurement
    near_pi = Rotation.from_rotvec([np.pi - 1e-3, 0, 0]).as_matrix()
    obs = hc.Obs(np.zeros((1, 10)), near_pi[None], np.zeros((1, 3)))
    M[0, :3, :3] = Rotation.from_rotvec([-(np.pi - 1e-3), 0, 0]).as_matrix()
    assert np.linalg.norm(hc.raw_residuals(M, obs)[0, 3:]) < 5e-3


def test_sigma_vectors_and_grid_level():
    s = hc.sigma_vector(2.0)
    assert s[2] == pytest.approx(2e-3)
    assert s[0] == s[1] and s[3] == s[4]
    m = hc.mocap_sigma_vector(2.0)
    assert np.allclose(m[:3], 2e-3) and np.allclose(m[3:], 2e-3 / 0.1)
    assert hc.noise_level(0.3) == 0.5
    assert hc.noise_level(1.38) == 2.0
    assert hc.noise_level(2.0) == 2.0
    assert hc.noise_level(9.0) == 4.0


def test_gauge_on_the_real_training_postures(kin, sessions):
    """Naive set: rank 19 of 22; gauge-fixed set: full rank, well conditioned."""
    obs = hc.read_camera(hc.role_sets(sessions)["training"][0])
    sigma = hc.sigma_vector(2.0)
    Y0 = hc.register_board(kin, obs, np.eye(4), np.zeros(10))
    # noise-free data of the nominal model (residual zero at the origin)
    Tc, Ta = kin.poses(obs.Q, np.zeros(10))
    M = hc.predict(Tc, Ta, np.eye(4), Y0)
    clean = hc.Obs(obs.Q, M[:, :3, :3], M[:, :3, 3])
    naive = hc.Spec(hc.ALL_OFFSETS)
    J = hc.jacobian(kin, clean, naive, np.zeros(naive.n_params), sigma, np.eye(4), Y0)
    s = np.linalg.svd(J, compute_uv=False)
    assert int(np.sum(s > 1e-8 * s[0])) == 19
    assert s[18] / s[0] < 0.02
    fixed = hc.Spec()
    J = hc.jacobian(kin, clean, fixed, np.zeros(fixed.n_params), sigma, np.eye(4), Y0)
    s = np.linalg.svd(J, compute_uv=False)
    assert s.size == 18 and s[-1] > 0 and s[0] / s[-1] < 30


def test_fit_recovers_noise_free_truth(kin, sessions):
    obs = hc.read_camera(hc.role_sets(sessions)["training"][0])
    sigma = hc.sigma_vector(2.0)
    rng = np.random.default_rng(3)
    truth = np.zeros(10)
    truth[3:9] = rng.normal(0, 0.01, 6)
    X = hc.hom(np.r_[0.005, -0.004, 0.003, 0.01, -0.02, 0.015])
    Y = hc.register_board(kin, obs, np.eye(4), np.zeros(10))
    Tc, Ta = kin.poses(obs.Q, truth)
    M = hc.predict(Tc, Ta, X, Y)
    clean = hc.Obs(obs.Q, M[:, :3, :3], M[:, :3, 3])
    f = hc.fit(kin, clean, hc.Spec(), sigma)
    assert f.cost < 1e-12
    assert np.allclose(f.offsets[3:9], truth[3:9], atol=1e-7)
    assert f.rank() == 18


def test_loo_of_exact_data_is_zero(kin, sessions):
    obs = hc.read_camera(hc.role_sets(sessions)["training"][0])
    sigma = hc.sigma_vector(2.0)
    off = np.zeros(10)
    off[3:9] = 0.01
    X = hc.hom(np.r_[0.004, -0.003, 0.002, 0.01, -0.01, 0.02])
    Y = hc.register_board(kin, obs, np.eye(4), np.zeros(10))
    Tc, Ta = kin.poses(obs.Q, off)
    M = hc.predict(Tc, Ta, X, Y)
    clean = hc.Obs(obs.Q, M[:, :3, :3], M[:, :3, 3])
    err = hc.loo_errors(kin, clean, off, sigma, np.eye(4))
    assert np.abs(err).max() < 1e-7


def test_standard_errors_are_huge_on_a_null_direction(kin, sessions):
    """Simulated data at the training postures (no real-data joint fit)."""
    obs = hc.read_camera(hc.role_sets(sessions)["training"][0])
    sigma = hc.sigma_vector(2.0)
    rng = np.random.default_rng(5)
    Y = hc.register_board(kin, obs, np.eye(4), np.zeros(10))
    off = np.zeros(10)
    off[3:9] = rng.normal(0, 0.01, 6)
    Tc, Ta = kin.poses(obs.Q, off)
    M = hc.predict(Tc, Ta, np.eye(4), Y)
    noisy = hc.Obs(
        obs.Q,
        Rotation.from_rotvec(sigma[3:] * rng.normal(size=(len(M), 3))).as_matrix()
        @ M[:, :3, :3],
        M[:, :3, 3] + sigma[:3] * rng.normal(size=(len(M), 3)),
    )
    fixed = hc.fit(kin, noisy, hc.Spec(), sigma)
    naive = hc.fit(kin, noisy, hc.Spec(hc.ALL_OFFSETS), sigma)
    se_n = dict(zip(naive.names, naive.se()))
    se_f = np.median(fixed.se()[:6])
    assert se_n["d_head_2"] > 1e3 * se_f and se_n["d_arm_7"] > 1e3 * se_f


def test_noise_stage_is_registration_only(sessions):
    est = hc.noise_estimates(sessions)
    c = est["camera"]
    assert c["pairs"] == 17
    assert 1.2 < c["sigma_z_mm"] < 1.6
    assert c["level_mm"] == hc.SETTINGS["noise_level_mm"] == 2.0
    for key, name in (
        ("xy", "xy"),
        ("tilt_mrad_per_mm", "tilt_mrad_per_mm"),
        ("roll_mrad_per_mm", "roll_mrad_per_mm"),
    ):
        assert c["ratios"][key] == pytest.approx(
            hc.SETTINGS["noise_ratios"][name], rel=2e-3
        )
    assert est["mocap"]["pose_std_mm"] == pytest.approx(0.762, abs=0.005)
    assert est["mocap"]["level_mm"] == 2.0


def test_pal_reference_and_gauge_mapping():
    ref = hc.pal_reference()
    assert ref["offsets"]["arm_1_joint"] == pytest.approx(0.02)
    g = hc.pal_gauge_offsets()
    assert g["arm_1_minus_head_1"] == pytest.approx(0.02 - 0.0111854)
    assert hc.pal_gauge_offsets(-1.0)["arm_5"] == pytest.approx(0.02)
