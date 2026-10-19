"""Static-posture extraction rules for the TIAGo Qualisys sessions (#27).

The original bags are not distributed; these synthetic cases pin the rules
in examples/tiago/utils/mocap_extraction.py that produced the shipped files.
"""

import os
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import (
    POINTS,
    extract_session,
    file_sha256,
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


def test_world_frame_keeps_points_and_flags_the_frame():
    """No base stream (#104): points stay in the Qualisys world frame."""
    t, q, postures = _joint_trajectory()
    R_base = Rotation.from_euler("z", 30, degrees=True).as_matrix()
    p_base = np.array([1.0, -2.0, 0.5])
    local = {n: np.array([0.1 * k, 0.05, 0.3]) for k, n in enumerate(POINTS)}
    _, streams = _mocap_streams(t, 0.0, R_base, p_base, local)

    table = extract_static_postures(t, q, None, streams, lag=0.0)

    assert len(table) == 3
    assert list(table.columns[-2:]) == ["marker_std_mm", "points_in_base_frame"]
    assert (table.points_in_base_frame == 0).all()
    for k, name in enumerate(POINTS, start=1):
        np.testing.assert_allclose(
            table[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy(),
            np.tile(R_base @ local[name] + p_base, (3, 1)),
            atol=1e-12,
        )


def test_base_frame_output_has_no_flag_column():
    t, q, _ = _joint_trajectory()
    local = {n: np.array([0.1, 0.0, 0.0]) for n in POINTS}
    base, streams = _mocap_streams(t, 0.0, np.eye(3), np.zeros(3), local)
    table = extract_static_postures(t, q, base, streams, lag=0.0)
    assert "points_in_base_frame" not in table.columns


def test_extract_session_refuses_a_wrong_bag_hash(tmp_path):
    bag = tmp_path / "not_a_bag.bag"
    bag.write_bytes(b"x")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_session(str(bag), lag=0.0, expected_sha256="0" * 64)
    with pytest.raises(ValueError, match="frame must be"):
        extract_session(str(bag), lag=0.0, frame="sideways")


MOCAP = Path(__file__).resolve().parents[1] / "examples/tiago/data/calibration/mocap"


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
@pytest.mark.parametrize("session", ["2021-10-26-1051", "2021-10-26-1102"])
def test_shipped_october_files_are_re_created_byte_for_byte(session, tmp_path):
    """Original bag (named by sha256 in protocol.yaml) to the shipped CSV."""
    manifest = yaml.safe_load((MOCAP / "protocol.yaml").read_text())
    entry = next(s for s in manifest["sessions"] if s["id"] == session)
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    bag = next(root.rglob(entry["source"]["bag"]), None)
    assert bag is not None, f"{entry['source']['bag']} not under {root}"
    assert file_sha256(bag) == entry["source"]["sha256"]
    frame = "world" if entry["frame"] == "qualisys:world" else "base"
    table, _ = extract_session(str(bag), lag=entry["source"]["lag_s"], frame=frame)
    out = tmp_path / "out.csv"
    table.to_csv(out, index=False)
    (name,) = entry["files"]
    assert out.read_bytes() == (MOCAP / name).read_bytes()
