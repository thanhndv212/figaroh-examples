"""Static-posture extraction rules for the TIAGo OptiTrack runs (#105).

The original bags are not distributed; these synthetic cases pin the rules in
examples/tiago/utils/optitrack_extraction.py that produced the shipped files.
"""

import os
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import file_sha256
from examples.tiago.utils.optitrack_extraction import (
    COLUMNS,
    drop_frozen,
    extract_session,
    extract_static_postures_6d,
    measure_clock_lag,
)

RATE = 100.0


def _joint_trajectory():
    """Three 4 s static postures joined by 2 s ramps, at 100 Hz."""
    postures = np.array([[0.1] * 8, [0.2] * 8, [-0.3] * 8])
    segments = []
    for k, p in enumerate(postures):
        segments.append(np.tile(p, (int(4 * RATE), 1)))
        if k + 1 < len(postures):
            ramp = np.linspace(0, 1, int(2 * RATE))[:, None]
            segments.append(p + ramp * (postures[k + 1] - p))
    q = np.vstack(segments)
    return np.arange(len(q)) / RATE, q, postures


def _body(t, lag, R, p, header_shift=0.0):
    """Body stream with header clock t - lag, receive clock t."""
    quat = Rotation.from_matrix(R).as_quat()
    n = len(t)
    pose = np.tile(np.r_[p, quat], (n, 1))
    pose[:, 0] += 1e-9 * (-1.0) ** np.arange(n)  # live: no two samples repeat
    return np.column_stack([t - lag + header_shift, t, pose])


def _freeze(stream, t0, t1):
    """Repeat the last pre-window pose over [t0, t1] (NatNet tracking loss)."""
    out = stream.copy()
    sel = (out[:, 1] >= t0) & (out[:, 1] <= t1)
    first = np.flatnonzero(sel)[0]
    out[sel, 2:] = out[first - 1, 2:]
    return out


def _scene():
    t, q, postures = _joint_trajectory()
    R_b = Rotation.from_euler("xyz", [0.1, -0.2, 0.5]).as_matrix()
    p_b = np.array([1.0, -2.0, 0.5])
    R_loc = Rotation.from_euler("xyz", [0.3, 0.2, -0.4]).as_matrix()
    p_loc = np.array([0.4, -0.1, 0.7])
    lag = -1.3
    base = _body(t, lag, R_b, p_b)
    grip = _body(t, lag, R_b @ R_loc, R_b @ p_loc + p_b)
    return t, q, postures, base, grip, lag, (R_b, p_b, R_loc, p_loc)


def test_drop_frozen_keeps_the_first_sample_of_a_run():
    n = 300
    x = np.cumsum(np.ones((n, 7)) * 1e-3, axis=0)
    s = np.column_stack([np.arange(n) / 100, np.arange(n) / 100, x])
    s[100:200, 2:] = s[99, 2:]  # 1 s frozen
    kept = drop_frozen(s)
    assert len(kept) == n - 100
    np.testing.assert_array_equal(kept[99], s[99])
    # close but unequal rows are all measurements
    s2 = s.copy()
    s2[100:200, 2:] += np.arange(100)[:, None] * 1e-12
    assert len(drop_frozen(s2)) == n - 1  # only the exact repeat goes


def test_measure_clock_lag_sign_and_value():
    rng = np.random.default_rng(0)
    t = np.arange(0, 60, 0.01)
    robot = (t, t + 0.020 + rng.uniform(-0.01, 0.01, len(t)))
    th = np.arange(0, 60, 1 / 120)
    mocap = (th, th - 1.363 + rng.uniform(-0.003, 0.003, len(th)))
    # mocap stamps run ahead of the recorder: t_robot - t_mocap = -1.383
    assert measure_clock_lag(robot, mocap) == pytest.approx(-1.383, abs=1e-3)


def test_world_frame_rows_hold_gripper_and_base_poses():
    pin = pytest.importorskip("pinocchio")
    t, q, postures, base, grip, lag, (R_b, p_b, R_loc, p_loc) = _scene()
    table, drops = extract_static_postures_6d(t, q, base, grip, lag, 2)
    assert list(table.columns) == COLUMNS
    assert drops == {"plateaus": 3, "gripper_lost": 0}
    assert len(table) == 3
    R_g, p_g = R_b @ R_loc, R_b @ p_loc + p_b
    want = np.r_[p_g, pin.rpy.matrixToRpy(R_g)]
    np.testing.assert_allclose(
        table[["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]].to_numpy(),
        np.tile(want, (3, 1)),
        atol=1e-8,
    )
    want_b = np.r_[p_b, pin.rpy.matrixToRpy(R_b)]
    np.testing.assert_allclose(
        table[["xb", "yb", "zb", "phixb", "phiyb", "phizb"]].to_numpy(),
        np.tile(want_b, (3, 1)),
        atol=1e-8,
    )
    np.testing.assert_allclose(table.iloc[:, 6:14].to_numpy(), postures, atol=1e-4)
    assert (table.points_in_base_frame == 0).all()
    assert (table.rigid_body_def == 2).all()
    assert (table.t_end_robot - table.t_start_robot).min() >= 1.2
    np.testing.assert_allclose(table.pose_std_mm, 0, atol=1e-5)


def test_base_frame_recovers_the_known_local_pose():
    pin = pytest.importorskip("pinocchio")
    t, q, _, base, grip, lag, (_, _, R_loc, p_loc) = _scene()
    table, drops = extract_static_postures_6d(t, q, base, grip, lag, 1, frame="base")
    assert drops == {"plateaus": 3, "gripper_lost": 0, "base_lost": 0}
    want = np.r_[p_loc, pin.rpy.matrixToRpy(R_loc)]
    np.testing.assert_allclose(
        table[["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]].to_numpy(),
        np.tile(want, (3, 1)),
        atol=1e-8,
    )
    assert (table.points_in_base_frame == 1).all()


def test_mocap_window_is_shifted_by_the_lag():
    t, q, _, base, grip, lag, _ = _scene()
    # a pose step 0.2 s before the true window start of plateau 1 (robot 0.5 s)
    step = grip.copy()
    step[(step[:, 1] >= 0.2) & (step[:, 1] < 0.4), 2] += 0.5
    ok, _ = extract_static_postures_6d(t, q, base, step, lag, 1)
    bad, _ = extract_static_postures_6d(t, q, base, step, 0.0, 1)
    assert ok.x1[0] == pytest.approx(grip[0, 2], abs=1e-8)
    assert bad.x1[0] != pytest.approx(grip[0, 2], abs=1e-6)


def test_tracking_loss_counts_and_partial_tracking():
    t, q, _, base, grip, lag, _ = _scene()
    # gripper lost over all of plateau 2 (robot 6-10 s)
    lost = _freeze(grip, 5.9, 10.1)
    table, drops = extract_static_postures_6d(t, q, base, lost, lag, 1)
    assert len(table) == 2
    assert drops == {"plateaus": 3, "gripper_lost": 1}
    # base lost over plateau 3 (robot 12-16 s): kept in world frame, NaN base
    nobase = _freeze(base, 11.9, 16.1)
    table, drops = extract_static_postures_6d(t, q, nobase, grip, lag, 1)
    assert len(table) == 3 and drops["gripper_lost"] == 0
    assert table.xb.isna().tolist() == [False, False, True]
    assert table.n_base_live.iloc[2] == 0
    # dropped in the base frame, and counted
    table, drops = extract_static_postures_6d(t, q, nobase, grip, lag, 1, "base")
    assert len(table) == 2 and drops["base_lost"] == 1
    # every other sample frozen: kept, with about half of the window live
    half = grip.copy()
    half[::2, 3] += 1e-9 * np.arange(len(half[::2]))  # successive live samples differ
    half[1::2, 2:] = half[0::2, 2:]
    table, _ = extract_static_postures_6d(t, q, base, half, lag, 1)
    assert len(table) == 3
    assert (table.n_live / table.n_window).between(0.45, 0.55).all()


def test_integer_columns_are_written_as_integers(tmp_path):
    t, q, _, base, grip, lag, _ = _scene()
    table, _ = extract_static_postures_6d(t, q, base, grip, lag, 1)
    out = tmp_path / "t.csv"
    table.to_csv(out, index=False)
    back = pd.read_csv(out)
    for name in ("n_window", "n_live", "n_base_live", "rigid_body_def"):
        assert back[name].dtype.kind == "i"
    header = out.read_text().splitlines()[0].split(",")
    row = out.read_text().splitlines()[1].split(",")
    for name in ("n_window", "n_live", "n_base_live", "rigid_body_def"):
        assert "." not in row[header.index(name)]


def test_extract_session_refuses_a_wrong_bag_hash(tmp_path):
    bag = tmp_path / "not_a_bag.bag"
    bag.write_bytes(b"x")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_session(str(bag), 1, lag=0.0, expected_sha256="0" * 64)
    t, q, _, base, grip, lag, _ = _scene()
    with pytest.raises(ValueError, match="frame must be"):
        extract_static_postures_6d(t, q, base, grip, lag, 1, frame="sideways")


OPTITRACK = (
    Path(__file__).resolve().parents[1] / "examples/tiago/data/calibration/optitrack"
)


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
@pytest.mark.parametrize(
    "session", ["2023-09-12-1529", "2023-09-12-1552", "2023-09-12-1658"]
)
def test_shipped_files_are_re_created_byte_for_byte(session, tmp_path):
    """Original bag (named by sha256 in protocol.yaml) to the shipped CSV."""
    manifest = yaml.safe_load((OPTITRACK / "protocol.yaml").read_text())
    entry = next(s for s in manifest["sessions"] if s["id"] == session)
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    bag = next(root.rglob(entry["source"]["bag"]), None)
    assert bag is not None, f"{entry['source']['bag']} not under {root}"
    assert file_sha256(bag) == entry["source"]["sha256"]
    table, lag, drops = extract_session(str(bag), entry["rigid_body"], frame="world")
    assert abs(lag - entry["source"]["lag_s"]) < 5e-4
    assert drops == entry["drops"]
    out = tmp_path / "out.csv"
    table.to_csv(out, index=False)
    (name,) = entry["files"]
    assert out.read_bytes() == (OPTITRACK / name).read_bytes()
