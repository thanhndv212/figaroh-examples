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


def _raw(n, markers, force=None, moment=None, cop=None, seed=0):
    """(n, 39) rows from (n, 3, 3) base markers; arm markers fixed."""
    v = np.zeros((n, 39))
    v[:, 0:9] = markers.reshape(n, 9)
    v[:, 9:21] = np.tile([0.8, 0.4, 0.9], 4)
    v[:, 21:30] = np.tile([1.3, 0.4, 1.0], 3)
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
    c = TRIAD.mean(axis=0)
    rots = Rotation.from_rotvec(ang).as_matrix()
    markers = np.einsum("nij,kj->nki", rots, TRIAD - c) + c + shift[:, None, :]
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
    assert sm.design("S2", reg, h_e=-0.2).shape == (reg.n, 6, 12)
    assert sm.design("S3", reg).shape == (reg.n, 6, 42)
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
    assert not np.allclose(sm.design("S2", reg, -0.25), sm.design("S1", reg))


def test_transport_and_origin_switches_change_only_what_they_name():
    n, t, markers, *_ = _synthetic_run(seconds=20)
    raw = _raw(n, markers, force=[5.0, -3.0, -730.0])
    base = sd.process_raw(t, raw)
    nt = sd.process_raw(t, raw, transport=False)
    assert np.allclose(base.w[:, :3], nt.w[:, :3]) and np.allclose(base.x, nt.x)
    co = sd.process_raw(t, raw, origin="corner")
    assert not np.allclose(co.x[:, :3], base.x[:, :3])
    assert np.allclose(co.x[:, 3:], base.x[:, 3:])


def test_corner_origin_gives_translation_for_a_pure_rotation():
    n = 400
    ang = np.zeros((n, 3))
    ang[:, 2] = 0.02 * np.clip(np.arange(n) / RATE - 1.0, 0, 1)
    c = TRIAD.mean(axis=0)
    markers = (
        np.einsum("nij,kj->nki", Rotation.from_rotvec(ang).as_matrix(), TRIAD - c) + c
    )
    t = np.arange(n) / RATE + 0.003
    raw = _raw(n, markers)
    good = sd.process_raw(t, raw, cutoff_hz=None)
    bad = sd.process_raw(t, raw, origin="corner", cutoff_hz=None)
    assert np.abs(good.x[:, :3]).max() < 1e-9
    assert np.abs(bad.x[:, :3]).max() > 1e-4


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


def test_validation_wrench_needs_the_unlock_and_diagnostics_are_no_targets():
    sess = sm.load_sessions(verify=False)
    with pytest.raises(PermissionError):
        sm.read_target(sess["xyz_1628"])
    with pytest.raises(PermissionError):
        sm.read_target(sess["postures_1720"])


def test_frozen_config_is_complete():
    s = sm.SETTINGS
    n = s["noise"]
    assert n["used_level"] in n["levels"] and n["levels"][0] == 0.0
    for key in ("plate_sd", "x_sd", "v_sd"):
        assert len(n[key]) == 6 and all(v > 0 for v in n[key]), key
    assert 0 <= n["clock"]["drop_rate"] < 0.1
    assert s["sessions"]["confirmation"] == ["xyz_mirror_1630", "xyz_mirror_1642"]
    assert s["sampling"]["rate_hz"] == 100.0
