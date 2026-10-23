"""TIAGo base suspension (#70), part 1: loader, models and guards.

Synthetic data only, plus hash checks and the read guards of the shipped
sessions. No wrench of a real session is read and no suspension parameter is
fitted to one.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import suspension_model as sm  # noqa: E402
from examples.tiago.utils import suspension_data as sd  # noqa: E402

RATE = 100.0
TRIAD = np.array([[0.90, 0.45, 0.31], [1.00, 0.46, 0.31], [0.93, 0.52, 0.31]])
GRIP_TRIAD = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.08, 0.0]])
# the real base triad is labelled so that its z axis points down
TRIAD_DOWN = TRIAD[[0, 2, 1]]


def _raw(n, markers, force=None, moment=None, cop=None, seed=0):
    """(n, 39) rows from (n, 3, 3) base markers; arm markers fixed."""
    v = np.zeros((n, 39))
    v[:, 0:9] = markers.reshape(n, 9)
    v[:, 9:21] = np.tile([0.8, 0.4, 0.9], 4)
    v[:, 21:30] = (np.array([1.3, 0.4, 1.0]) + GRIP_TRIAD).reshape(9)
    v[:, 30:33] = [0.0, 0.0, -730.0] if force is None else force
    v[:, 33:36] = 0.0 if moment is None else moment
    v[:, 36:39] = [0.9, 0.45, 0.0] if cop is None else cop
    return v


def _clock_rows(n_frames, drops, jitter, seed=0):
    """Receive stamps of frames 0..n-1 minus ``drops``, latency 2 ms + jitter."""
    rng = np.random.default_rng(seed)
    frames = np.array([f for f in range(n_frames) if f not in set(drops)])
    t = frames / RATE + 0.002 + jitter * rng.random(frames.size)
    return frames, t


def test_clock_recovers_drops_and_total_frames():
    drops = [120, 121, 400, 777]
    frames, t = _clock_rows(1200, drops, jitter=0.003)
    clk = sd.rebuild_clock(t)
    assert clk.drops == len(drops)
    assert clk.n_frames == frames[-1] + 1 and clk.duplicates == 0
    assert np.all(np.diff(clk.frame) >= 1)
    assert clk.latency.min() >= 0.0
    # every recovered drop sits within a few rows of the true one
    got = np.flatnonzero(np.diff(clk.frame) > 1)
    for d in (120, 400, 777):
        assert np.min(np.abs(got - np.searchsorted(frames, d))) <= 3
    assert clk.stats()["max_gap_frames"] >= 2


def test_clock_drops_exact_duplicates_and_counts_them():
    frames, t = _clock_rows(300, [150], jitter=0.002)
    values = np.random.default_rng(1).normal(size=(frames.size, 39))
    values = np.insert(values, 50, values[49], axis=0)  # a repeated row
    t = np.insert(t, 50, 0.5 * (t[49] + t[50]))
    clk = sd.rebuild_clock(t, values)
    assert clk.duplicates == 1 and not clk.keep[50]
    assert clk.frame.size == values.shape[0] - 1


def test_clock_rejects_non_increasing_stamps():
    with pytest.raises(ValueError):
        sd.rebuild_clock(np.array([0.0, 0.01, 0.01, 0.03]))


def test_to_grid_interpolates_and_flags_dropped_frames():
    frames, t = _clock_rows(100, [40], jitter=0.0)
    clk = sd.rebuild_clock(t)
    sig = (0.5 * frames)[:, None]  # linear in the frame number
    grid, flag = sd.to_grid(clk, sig)
    assert grid.shape == (100, 1) and flag.sum() == 1
    assert np.allclose(grid[:, 0], 0.5 * np.arange(100))


@pytest.mark.parametrize("axis", [[0, 0, 1], [1, 0, 0], [1, 1, 1]])
def test_rotation_about_an_axis_through_the_centroid_has_no_translation(axis):
    n = 300
    ang = 0.01 * np.sin(2 * np.pi * 0.5 * np.arange(n) / RATE)
    rots = Rotation.from_rotvec(np.outer(ang, np.array(axis) / np.linalg.norm(axis)))
    c = TRIAD.mean(axis=0)
    markers = np.einsum("nij,kj->nki", rots.as_matrix(), TRIAD - c) + c
    trans, rotvec = sd.triad_pose(markers, TRIAD)
    assert np.abs(trans).max() < 1e-12
    assert np.linalg.norm(rotvec, axis=1).max() == pytest.approx(0.01, rel=1e-3)


def test_pure_translation_has_no_rotation():
    shift = np.array([0.001, -0.002, 0.0005])
    markers = np.tile(TRIAD + shift, (5, 1, 1))
    trans, rotvec = sd.triad_pose(markers, TRIAD)
    assert np.allclose(trans, shift) and np.abs(rotvec).max() < 1e-12


def test_moment_transport_of_a_point_force():
    c = np.array([0.9, 0.45, 0.0])
    a = np.array([1.0, 0.4, 0.3])  # where the force acts
    f = np.array([[10.0, -5.0, -700.0]])
    m_c = np.cross(a - c, f)
    assert np.allclose(sd.transport_moment(m_c, f, c, a), 0.0)
    p = np.array([0.95, 0.5, 0.31])
    assert np.allclose(sd.transport_moment(m_c, f, c, p), np.cross(a - p, f))


def test_plate_centre_fit_recovers_the_centre():
    rng = np.random.default_rng(3)
    c = np.array([0.9, 0.45, 0.0])
    f = rng.normal(size=(200, 3)) * 20.0 + [0, 0, -730]
    cop = c + np.column_stack(
        (0.01 * rng.normal(size=200), 0.01 * rng.normal(size=200), np.zeros(200))
    )
    m = np.cross(cop - c, f)
    got, res = sd.plate_centre_fit(f, m, cop)
    assert np.allclose(got[:2], c[:2], atol=1e-9) and res < 1e-9


def test_base_frame_is_right_handed_z_up_whatever_the_marker_order():
    for ref in (TRIAD, TRIAD[[0, 2, 1]]):
        r = sd.base_frame(ref)
        assert np.linalg.det(r) == pytest.approx(1.0)
        assert r[2, 2] > 0.99 and np.allclose(r.T @ r, np.eye(3), atol=1e-12)
    # the status-quo normal of the reordered triad points down
    flipped = np.cross(TRIAD[2] - TRIAD[0], TRIAD[1] - TRIAD[0])
    assert flipped[2] < 0 < np.cross(TRIAD[1] - TRIAD[0], TRIAD[2] - TRIAD[0])[2]
    yaw = np.degrees(np.arctan2(TRIAD[1, 1] - TRIAD[0, 1], TRIAD[1, 0] - TRIAD[0, 0]))
    assert sd.frame_yaw_deg(TRIAD) == pytest.approx(yaw)


def _synthetic_run(seconds=40, seed=0):
    n = int(seconds * RATE)
    s = np.arange(n) / RATE
    rng = np.random.default_rng(seed)
    ramp = np.clip(s - 1.0, 0.0, 1.0)  # at rest in the first second
    shift = ramp[:, None] * np.column_stack(
        (
            2e-3 * np.sin(2 * np.pi * 0.20 * s),
            1.5e-3 * np.sin(2 * np.pi * 0.31 * s + 1),
            3e-4 * np.sin(2 * np.pi * 0.47 * s + 2),
        )
    )
    ang = ramp[:, None] * np.column_stack(
        (
            2e-3 * np.sin(2 * np.pi * 0.23 * s + 0.5),
            5e-3 * np.sin(2 * np.pi * 0.29 * s),
            2e-2 * np.sin(2 * np.pi * 0.17 * s + 2),
        )
    )
    c = TRIAD_DOWN.mean(axis=0)
    rots = Rotation.from_rotvec(ang).as_matrix()
    markers = np.einsum("nij,kj->nki", rots, TRIAD_DOWN - c) + c + shift[:, None, :]
    t = s + 0.003
    return n, t, markers, shift, ang, rng


def test_regressor_shapes_and_noise_free_ls_recovery():
    n, t, markers, shift, ang, _ = _synthetic_run()
    raw = _raw(n, markers)
    reg, _ = sm.regressors_from_raw(t, raw)
    assert reg.x.shape == reg.v.shape == (reg.n, 6) and reg.valid.sum() > 0.8 * reg.n
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
    w = reg.x * k + reg.v * c + np.array([1.0, -2.0, 3.0, 0.5, 0.1, 0.0])
    assert sm.design("S1", reg).shape == (reg.n, 6, 12)
    assert sm.design("S2", reg, h_e=0.2).shape == (reg.n, 6, 12)
    assert sm.design("S3", reg).shape == (reg.n, 6, 42)
    assert sm.design("B2", reg).shape == (reg.n, 6, 4)
    fit = sm.fit_model("S1", reg, w)
    assert np.allclose(fit.k(), k, rtol=1e-6) and np.allclose(fit.c(), c, rtol=1e-6)
    pred = sm.predict(fit, reg)
    r2 = sm.score(w, pred, reg.valid)["r2"]
    assert np.all(r2 > 1 - 1e-9)
    full = sm.fit_model("S3", reg, w)
    assert np.allclose(full.k(), k, rtol=1e-5)


def test_elastic_s2_equals_s1_at_zero_height():
    n, t, markers, *_ = _synthetic_run(seconds=20)
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    assert np.allclose(sm.design("S2", reg, 0.0), sm.design("S1", reg))
    assert not np.allclose(sm.design("S2", reg, 0.25), sm.design("S1", reg))


def test_transport_and_origin_switches_change_only_what_they_name():
    n, t, markers, *_ = _synthetic_run(seconds=20)
    raw = _raw(n, markers, force=[5.0, -3.0, -730.0])
    base = sd.process_raw(t, raw)
    nt = sd.process_raw(t, raw, transport=False)
    assert np.allclose(base.w[:, :3], nt.w[:, :3]) and np.allclose(base.x, nt.x)
    co = sd.process_raw(t, raw, origin="corner")
    assert not np.allclose(co.x[:, :3], base.x[:, :3])
    assert np.allclose(co.x[:, 3:], base.x[:, 3:])


def test_mz_about_cop_adds_the_cop_lever_term_to_mz_only():
    n, t, markers, *_ = _synthetic_run(seconds=20)
    f = np.array([5.0, -3.0, -730.0])
    dx = 0.02 * np.sin(np.arange(n) / 50.0)  # moving COP: the term is not a constant
    cop = np.column_stack((0.95 + dx, np.full(n, 0.42), np.zeros(n)))
    raw = _raw(n, markers, force=f, moment=[1.0, 2.0, 0.5], cop=cop)
    a = sd.process_raw(t, raw, frame="world")
    b = sd.process_raw(t, raw, frame="world", mz_about="cop")
    assert np.allclose(a.w[:, :5], b.w[:, :5]) and np.allclose(a.x, b.x)
    # (COP - c) x F, z: only the moving part survives the filter and the static offset
    term = sd.lowpass((dx * f[1])[:, None], RATE, sd.CUTOFF_HZ)[:, 0]
    term = term - term[:100].mean()
    assert np.allclose((b.w[:, 5] - a.w[:, 5])[a.valid], term[a.valid], atol=1e-9)
    with pytest.raises(ValueError):
        sd.process_raw(t, raw, mz_about="plate")


def test_s3_psd_fit_solves_when_the_free_fit_wants_a_negative_stiffness():
    n, t, markers, *_ = _synthetic_run()
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.array([-1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([-150.0, -150.0, 500.0, 100.0, 80.0, 12.0])
    w = reg.x * k + reg.v * c
    fit = sm.fit_model("S3", reg, w)
    kk, cc = fit.matrices()
    assert np.linalg.eigvalsh(kk)[0] >= -1e-6 * np.abs(kk).max()
    assert np.linalg.eigvalsh(cc)[0] >= -1e-6 * np.abs(cc).max()
    assert fit.stiffness_bound_dofs().all()


def test_corner_origin_gives_translation_for_a_pure_rotation():
    n = 400
    ang = np.zeros((n, 3))
    ang[:, 2] = 0.02 * np.clip(np.arange(n) / RATE - 1.0, 0, 1)
    c = TRIAD_DOWN.mean(axis=0)
    markers = (
        np.einsum("nij,kj->nki", Rotation.from_rotvec(ang).as_matrix(), TRIAD_DOWN - c)
        + c
    )
    t = np.arange(n) / RATE + 0.003
    raw = _raw(n, markers)
    good = sd.process_raw(t, raw, cutoff_hz=None)
    bad = sd.process_raw(t, raw, origin="corner", cutoff_hz=None)
    assert np.abs(good.x[:, :3]).max() < 1e-9
    assert np.abs(bad.x[:, :3]).max() > 1e-4


# ------------------------------------------------- footprint origin (O1)


def _footprint():
    fp = sm.LOADER["footprint"]
    return dict(offset=fp["offset"], rotvec=fp["rotvec"])


def test_footprint_derivation_reproduces_the_frozen_config():
    got = sm.footprint_derivation()
    cfg = sm.SETTINGS["footprint"]
    assert np.allclose(got["offset_triad_m"], cfg["offset_triad_m"], atol=1e-6)
    assert np.allclose(got["offset_sd_m"], cfg["offset_sd_m"], atol=1e-5)
    assert np.allclose(
        got["rotation_triad_rotvec"], cfg["rotation_triad_rotvec"], atol=1e-5
    )
    assert got["rotation_sd_rad"] == pytest.approx(cfg["rotation_sd_rad"], abs=1e-4)
    # about 7-8 cm horizontal, 0.31 m below the (down-pointing) triad: the triad z is down
    assert 0.07 < got["horizontal_m"] < 0.085
    assert got["triad_z_world"][2] < -0.99 and 0.30 < got["offset_triad_m"][2] < 0.33


def test_footprint_rest_is_below_the_triad_with_z_up_and_rejects_a_flipped_triad():
    point, axes = sd.footprint_rest(TRIAD_DOWN, **_footprint())
    assert axes[2, 2] > 0.99 and np.linalg.det(axes) == pytest.approx(1.0)
    assert point[2] == pytest.approx(TRIAD_DOWN.mean(axis=0)[2] - 0.313235, abs=1e-3)
    with pytest.raises(
        ValueError
    ):  # a triad labelled z up would put the footprint above
        sd.footprint_rest(TRIAD, **_footprint())


def test_rotation_about_the_footprint_vertical_leaves_the_footprint_fixed():
    n = 300
    point, axes = sd.footprint_rest(TRIAD_DOWN, **_footprint())
    ang = np.zeros((n, 3))
    ang[:, 2] = 0.03 * np.clip(np.arange(n) / RATE - 1.0, 0, 1)
    rot = Rotation.from_rotvec(ang).as_matrix()
    markers = np.einsum("nij,kj->nki", rot, TRIAD_DOWN - point) + point
    trans, angle, p_b, ax = sd.base_pose(
        markers,
        TRIAD_DOWN,
        origin="footprint",
        frame="footprint",
        footprint=_footprint(),
    )
    assert np.allclose(p_b, point) and np.allclose(ax, axes)
    assert np.abs(trans).max() < 1e-12  # the footprint does not move
    assert np.allclose(angle[-1], np.array([0.0, 0.0, 0.03]) @ axes, atol=1e-6)
    centroid = sd.base_pose(markers, TRIAD_DOWN, origin="centroid", frame="world")[0]
    assert np.abs(centroid).max() > 1e-3  # the centroid does


def test_wrench_is_transported_to_the_footprint_and_rotated_to_its_axes():
    n, t, markers, *_ = _synthetic_run(seconds=20)
    raw = _raw(n, markers, force=[5.0, -3.0, -730.0])
    reg = sd.process_raw(
        t, raw, origin="footprint", frame="footprint", footprint=_footprint()
    )
    world = sd.process_raw(
        t, raw, origin="footprint", frame="world", footprint=_footprint()
    )
    assert np.allclose(reg.w[:, :3], world.w[:, :3] @ reg.axes)
    assert np.allclose(reg.w[:, 3:], world.w[:, 3:] @ reg.axes)
    assert reg.origin[2] < TRIAD_DOWN.mean(axis=0)[2] - 0.3 and reg.axes[2, 2] > 0.99


# ------------------------------------------- constrained fits (O4) and B2 (O2)


def test_s1_fit_is_bounded_and_flags_the_active_bound():
    n, t, markers, *_ = _synthetic_run()
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, -500.0, 100.0, 80.0, 12.0])  # a negative damping
    w = reg.x * k + reg.v * c
    free = sm.fit_model("S1", reg, w, constrained=False)
    fit = sm.fit_model("S1", reg, w)
    assert free.c()[2] == pytest.approx(-500.0, rel=1e-6)
    assert fit.c()[2] == 0.0 and np.all(fit.theta >= 0.0)
    assert fit.bound_dofs().tolist() == [False, False, True, False, False, False]
    ok = sm.fit_model("S1", reg, reg.x * k + reg.v * np.abs(c))
    assert not ok.bound_dofs().any()


def test_s3_fit_is_positive_semidefinite():
    n, t, markers, *_ = _synthetic_run()
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.diag([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.diag([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
    c[0, 1] = c[1, 0] = 300.0  # indefinite: det(c[:2, :2]) < 0
    w = reg.x @ k + reg.v @ c
    fit = sm.fit_model("S3", reg, w)
    for m in fit.matrices():
        ev = np.linalg.eigvalsh(m)
        assert ev[0] > -1e-6 * ev[-1]
    assert fit.bound_dofs().all()  # a zero eigenvalue flags every DOF
    pd = sm.fit_model("S3", reg, reg.x @ k + reg.v @ np.diag(np.diag(c)))
    assert not pd.bound_dofs().any()


def _cube_run(seconds=40):
    n, t, markers, *_ = _synthetic_run(seconds)
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    s = np.arange(reg.n) / RATE
    ang = np.column_stack(
        (
            0.1 * np.sin(2 * np.pi * 0.3 * s),
            0.08 * np.sin(2 * np.pi * 0.2 * s + 1),
            0.15 * np.sin(2 * np.pi * 0.25 * s),
        )
    )
    reg.grip = np.array([1.3, 0.4, 1.0]) + 0.1 * np.column_stack(
        (
            np.sin(2 * np.pi * 0.2 * s),
            np.sin(2 * np.pi * 0.3 * s + 1),
            np.sin(2 * np.pi * 0.25 * s + 2),
        )
    )
    reg.grip_rot = Rotation.from_rotvec(ang).as_matrix()
    return reg


def test_b2_cube_is_linear_in_mass_and_first_moment_and_bounded():
    reg = _cube_run()
    theta = np.array([25.0, 25.0 * 0.04, 25.0 * -0.03, 25.0 * 0.05])
    w = sm.arm_design(reg) @ theta + np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
    fit = sm.fit_model("B2", reg, w)
    assert np.allclose(fit.theta, theta, rtol=1e-6) and not fit.b2_at_bound()
    neg = sm.fit_model("B2", reg, sm.arm_design(reg) @ -theta)
    assert neg.theta[0] == 0.0 and neg.b2_at_bound()
    # a point mass at the centroid (o = 0): force m (g - a), moment about the origin
    d = sm.arm_design(reg)
    cdd = sm.cube_kinematics(reg.grip, reg.grip_rot)["cdd"]
    f = np.array([0.0, 0.0, -sm.G]) - cdd
    assert np.allclose(d[:, :3, 0] @ reg.axes.T, f)


def test_b2_cube_inertia_is_the_isotropic_term():
    reg = _cube_run()
    d0, d1 = sm.arm_design(reg, edge=0.0), sm.arm_design(reg, edge=0.11)
    kin = sm.cube_kinematics(reg.grip, reg.grip_rot)
    diff = (d1[:, 3:, 0] - d0[:, 3:, 0]) @ reg.axes.T
    assert np.allclose(diff, -(0.11**2 / 6.0) * kin["omega_dot"])
    assert np.allclose(d1[:, :3], d0[:, :3]) and np.allclose(d1[:, :, 1:], d0[:, :, 1:])


def test_verdict_fails_on_an_active_bound_or_a_vacuous_b2():
    ones, zero = np.ones(6), np.zeros(6)
    good = dict(
        r2_model=0.9 * ones,
        r2_b2=0.1 * ones,
        repeatable=np.ones(6, bool),
        nonlinear=np.zeros(6, bool),
    )
    assert sm.verdict(at_bound=np.zeros(6, bool), **good)["enough"]
    bound = np.zeros(6, bool)
    bound[2] = True
    assert not sm.verdict(at_bound=bound, **good)["enough"]
    assert not sm.verdict(at_bound=np.zeros(6, bool), b2_at_bound=True, **good)[
        "enough"
    ]
    assert zero.sum() == 0


def test_s1b2_nests_s1_and_b2_and_bounds_both():
    reg = _cube_run()
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
    arm = np.array([25.0, 25.0 * 0.04, 25.0 * -0.03, 25.0 * 0.05])
    d = sm.design("S1B2", reg)
    assert d.shape == (reg.n, 6, 16)
    assert np.allclose(d[:, :, :12], sm.design("S1", reg))
    assert np.allclose(d[:, :, 12:], sm.design("B2", reg))
    assert sm.parameter_names("S1B2")[12:] == sm.B2_NAMES
    w = reg.x * k + reg.v * c + sm.arm_design(reg) @ arm
    fit = sm.fit_model("S1B2", reg, w)
    assert np.allclose(fit.k(), k, rtol=1e-5) and np.allclose(fit.c(), c, rtol=1e-5)
    assert np.allclose(fit.theta[12:], arm, rtol=1e-5) and not fit.b2_at_bound()
    neg = sm.fit_model("S1B2", reg, reg.x * k + reg.v * c - sm.arm_design(reg) @ arm)
    assert neg.theta[12] == 0.0 and neg.b2_at_bound() and np.all(neg.theta[:12] >= 0)


def test_a_damping_at_its_bound_is_unidentified_not_a_failed_dof():
    n, t, markers, *_ = _synthetic_run()
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, -500.0, 100.0, 80.0, 12.0])
    fit = sm.fit_model("S1", reg, reg.x * k + reg.v * c)
    assert fit.bound_dofs()[2] and not fit.stiffness_bound_dofs().any()
    assert fit.damping_unidentified().tolist() == [False, False, True] + [False] * 3
    soft = sm.fit_model("S1", reg, reg.x * -k + reg.v * np.abs(c))
    assert soft.stiffness_bound_dofs().all()


def test_damping_with_a_large_relative_se_is_unidentified():
    n, t, markers, *_ = _synthetic_run()
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
    fit = sm.fit_model("S1", reg, reg.x * k + reg.v * c)
    fit.se = np.zeros(12)
    fit.se[1::2] = 0.1 * c
    fit.se[9] = 0.6 * c[4]  # c_ry: relative SE above rules.damping_rel_se_max
    assert fit.damping_unidentified().tolist() == [False] * 4 + [True, False]


def test_q2_needs_a_gain_over_b2_and_a_stiffness_off_its_bound():
    r2_b2 = np.full(6, 0.5)
    r2 = np.array([0.7, 0.55, 0.7, 0.5, 0.5, 0.5])
    bound = np.array([False, False, True, False, False, False])
    q2 = sm.suspension_adds(r2, r2_b2, bound)
    assert q2["adds"] == [True, False, False, False, False, False] and q2["present"]
    assert not sm.suspension_adds(r2_b2, r2_b2, np.zeros(6, bool))["present"]


def test_errors_in_variables_correction_removes_the_attenuation(monkeypatch):
    n, t, markers, *_ = _synthetic_run(seconds=200)
    reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
    k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
    c = np.array([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
    w = reg.x * k + reg.v * c
    rng = np.random.default_rng(3)
    xs = 0.5 * reg.x[reg.valid].std(axis=0)
    vs = 0.5 * reg.v[reg.valid].std(axis=0)
    reg.x = reg.x + xs * rng.normal(size=reg.x.shape)
    reg.v = reg.v + vs * rng.normal(size=reg.v.shape)
    noise = dict(sm.SETTINGS["noise"], x_sd=xs.tolist(), v_sd=vs.tolist())
    monkeypatch.setitem(sm.SETTINGS, "noise", noise)
    plain = sm.fit_model("S1", reg, w, constrained=False)
    eiv = sm.fit_s1_eiv(reg, w, level=1.0)
    assert np.all(plain.k() < 0.85 * k)  # attenuated by 1 / (1 + 0.25)
    assert np.allclose(eiv[0::2], k, rtol=0.05)


def test_near_duplicates_are_frozen_rows_of_a_moving_arm():
    n = 400
    s = np.arange(n) / RATE
    v = _raw(n, np.broadcast_to(TRIAD_DOWN, (n, 3, 3)).copy())
    v[:, 21:30] += np.tile(0.05 * np.sin(2 * np.pi * 0.5 * s)[:, None], (1, 9))
    v[:, :30] += 1e-3 * 0.011 * np.random.default_rng(0).normal(size=(n, 30))
    assert sm.near_duplicates(v)["count"] == 0
    v[201, :30] = v[200, :30]  # a repeated (frozen) row while the arm moves
    v[301, :30] = v[300, :30] + 1e-6
    out = sm.near_duplicates(v)
    assert out["count"] == 2 and out["rate"] == pytest.approx(2 / (n - 1))


def test_selection_pools_runs_and_picks_s1_for_an_s1_truth():
    runs = []
    for seed in (0, 1):
        n, t, markers, *_ = _synthetic_run(seconds=40, seed=seed)
        reg, _ = sm.regressors_from_raw(t, _raw(n, markers))
        k = np.array([1e4, 1e4, 1e5, 3e4, 2e4, 1e3])
        c = np.array([150.0, 150.0, 500.0, 100.0, 80.0, 12.0])
        w = reg.x * k + reg.v * c
        w = w + 1e-3 * w.std(axis=0) * np.random.default_rng(seed).normal(size=w.shape)
        runs.append((reg, w))
    sel = sm.select_model(runs)
    assert sel["model"] == "S1" and set(sel["cv"]) == {"S1", "S2", "S3"}
    assert sm.h_e_grid()[0] == 0.0 and sm.h_e_grid()[-1] == pytest.approx(0.6)


# --------------------------------------------------------------- guards


def test_shipped_files_match_their_hashes():
    ids = sm.verify_hashes()
    assert "xyz_1640" in ids and "xyz_mirror_1630" in ids


def test_confirmation_runs_are_never_read():
    sess = sm.load_sessions(verify=False)
    for sid in ("xyz_mirror_1630", "xyz_mirror_1642"):
        with pytest.raises(PermissionError):
            sm.read_regressors(sess[sid])
        with pytest.raises(PermissionError):
            sm.read_target(sess[sid])


def test_confirmation_regressors_need_the_confirmation_unlock(monkeypatch):
    sess = sm.load_sessions(verify=False)
    monkeypatch.setattr(sm, "_run", lambda sid, *a: None)
    monkeypatch.setattr(sm, "_regressors", lambda s, r: s.id)
    with pytest.raises(PermissionError):
        sm.read_regressors(sess["xyz_mirror_1630"])
    assert sm.read_regressors(sess["xyz_mirror_1630"], unlock_confirmation=True) == "xyz_mirror_1630"


def test_validation_wrench_needs_the_unlock_and_the_postures_are_no_targets():
    sess = sm.load_sessions(verify=False)
    with pytest.raises(PermissionError):
        sm.read_target(sess["xyz_1628"])
    with pytest.raises(PermissionError):
        sm.read_target(sess["postures_1720"])


def test_selection_diagnostics_are_targets_but_never_confirmation(monkeypatch):
    sess = sm.load_sessions(verify=False)
    selection = sm.SETTINGS["sessions"]["selection_diagnostics"]
    assert selection and all(sess[i].role == "diagnostic" for i in selection)
    assert not set(selection) & {"postures_1720", "postures_1734"}
    monkeypatch.setattr(
        sm, "_run", lambda sid, *a: type("R", (), {"w": np.zeros((3, 6))})()
    )
    assert sm.read_target(sess[selection[0]]).shape == (3, 6)  # no real wrench read
    with pytest.raises(PermissionError):
        sm.read_target(sess["xyz_mirror_1630"])


def test_frozen_config_is_complete():
    s = sm.SETTINGS
    n = s["noise"]
    assert n["used_level"] in n["levels"] and n["levels"][0] == 0.0
    for key in ("plate_sd", "x_sd", "v_sd"):
        assert len(n[key]) == 6 and all(v > 0 for v in n[key]), key
    assert 0 <= n["clock"]["drop_rate"] < 0.1
    assert s["sessions"]["confirmation"] == ["xyz_mirror_1630", "xyz_mirror_1642"]
    assert s["sampling"]["rate_hz"] == 100.0
    fp = s["footprint"]
    assert s["origin"] == "base_footprint" and s["frame"] == "footprint"
    assert len(fp["offset_triad_m"]) == len(fp["offset_sd_m"]) == 3
    assert len(fp["rotation_triad_rotvec"]) == 3 and fp["rotation_sd_rad"] > 0
    assert s["h_e_grid"]["min"] == 0.0 and s["b2"]["edge_m"] > 0
    assert "neg_damping_se" not in s["rules"] and s["rules"]["psd_zero_rel"] > 0
    # revision (maintainer, 2026-10-23)
    assert s["filter"]["cutoff_hz"] == 5.0 == sd.CUTOFF_HZ
    assert s["rules"]["damping_rel_se_max"] == 0.5 and s["rules"]["suspension_adds"] == 0.1
    assert s["clock"]["near_duplicate_sd"] == 3.0
    fx = s["fixture"]
    assert fx["truth"]["h_e"] == 0.0985 and fx["truth"]["h_e_sensitivity"] == [0.065, 0.25]
    assert fx["f2"]["bias_max"] == {"k": 0.01, "c": 0.10}
    assert "s2_gain_min" not in fx["f4"] and fx["f4"]["h_e_tol"] == 0.05
