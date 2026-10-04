"""Static-posture extraction rules for the TIAGo Qualisys sessions (#27).

The original bags are not distributed; these synthetic cases pin the rules
in examples/tiago/utils/mocap_extraction.py that produced the shipped files.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import (
    POINTS,
    estimate_clock_lag,
    extract_static_postures,
    static_plateaus,
)

RATE = 100.0


def _joint_trajectory():
    """Three 4 s static postures joined by 2 s ramps, at 100 Hz."""
    postures = np.array([[0.1] * 8, [0.2] * 8, [-0.3] * 8])
    segments, t0 = [], 0.0
    for k, p in enumerate(postures):
        segments.append(np.tile(p, (int(4 * RATE), 1)))
        if k + 1 < len(postures):
            ramp = np.linspace(0, 1, int(2 * RATE))[:, None]
            segments.append(p + ramp * (postures[k + 1] - p))
    q = np.vstack(segments)
    t = t0 + np.arange(len(q)) / RATE
    return t, q, postures


def test_static_plateaus_found_with_minimum_duration():
    t, q, postures = _joint_trajectory()
    plateaus = static_plateaus(t, q)
    assert len(plateaus) == 3
    for (a, b), p in zip(plateaus, postures):
        # within the 1 mrad rule tolerance (a ramp's first step may qualify)
        assert np.abs(q[a:b] - p).max() < 1e-3
        assert (b - a) / RATE >= 4.0
    # a 1.5 s plateau is too short
    assert static_plateaus(t[: int(1.5 * RATE)], q[: int(1.5 * RATE)]) == []


def test_clock_lag_recovered():
    t = np.arange(0, 120, 0.01)
    tool = np.column_stack(
        [np.sin(0.3 * t) * np.sin(0.05 * t**1.3), np.cos(0.2 * t), 0 * t]
    )
    lag = 3.9  # mocap clock behind: t_mocap = t_robot - lag
    t_mocap = np.arange(-10, 110, 0.01)
    mocap = np.column_stack([np.interp(t_mocap + lag, t, tool[:, c]) for c in range(3)])
    est, corr = estimate_clock_lag(t, tool, t_mocap, mocap)
    assert est == pytest.approx(lag, abs=0.1)
    assert corr > 0.95


def _mocap_streams(t, lag, R_base, p_base, local_points, drop=None):
    t_m = t - lag  # mocap clock
    quat = Rotation.from_matrix(R_base).as_quat()
    base = np.column_stack([t_m, np.tile(np.r_[p_base, quat], (len(t), 1))])
    streams = {}
    for name, local in local_points.items():
        world = R_base @ local + p_base
        s = np.column_stack([t_m, np.tile(world, (len(t), 1))])
        if drop is not None and name == "TL":
            s = s[(s[:, 0] < drop[0]) | (s[:, 0] > drop[1])]
        streams[name] = s
    return base, streams


def test_points_expressed_in_base_body_and_joints_averaged():
    t, q, postures = _joint_trajectory()
    R_base = Rotation.from_euler("z", 30, degrees=True).as_matrix()
    p_base = np.array([1.0, -2.0, 0.5])
    local = {n: np.array([0.1 * k, 0.05, 0.3]) for k, n in enumerate(POINTS)}
    base, streams = _mocap_streams(t, 2.6, R_base, p_base, local)

    table = extract_static_postures(t, q, base, streams, lag=2.6)

    assert len(table) == 3
    for k, name in enumerate(POINTS, start=1):
        np.testing.assert_allclose(
            table[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy(),
            np.tile(local[name], (3, 1)),
            atol=1e-12,
        )
    np.testing.assert_allclose(table.iloc[:, 12:20].to_numpy(), postures, atol=1e-4)
    np.testing.assert_allclose(table.marker_std_mm, 0.0, atol=1e-9)
    # averaging window: [start + 0.5 s, end - 0.3 s] on the robot clock
    assert (table.t_end_robot - table.t_start_robot).min() >= 1.2


def test_plateau_skipped_when_a_point_is_missing():
    t, q, _ = _joint_trajectory()
    local = {n: np.array([0.1, 0.0, 0.0]) for n in POINTS}
    # TL lost during the second plateau (robot 6-10 s, mocap 3.4-7.4 s)
    base, streams = _mocap_streams(
        t, 2.6, np.eye(3), np.zeros(3), local, drop=(3.0, 8.0)
    )
    table = extract_static_postures(t, q, base, streams, lag=2.6)
    assert len(table) == 2
