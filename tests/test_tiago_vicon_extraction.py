"""Static-posture extraction rules for the TIAGo Vicon runs (#106).

The original recordings are not distributed; these synthetic cases pin the
rules in examples/tiago/utils/vicon_extraction.py that produced the shipped
files. The c3d reader needs the optional ``c3d`` package and skips without it.
"""

import os
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import file_sha256
from examples.tiago.utils.optitrack_extraction import COLUMNS
from examples.tiago.utils.vicon_extraction import (
    BASE_TRIAD,
    GRIPPER_TRIAD,
    MARKERS,
    extract_session,
    extract_static_postures_6d,
    read_vicon_c3d,
    read_vicon_csv,
    repair_triad_labels,
    triad_frames,
)

RATE = 100.0
# triad constellations (m): marker 1, 2, 3 in the body frame
BASE_LOCAL = np.array([[0, 0, 0], [0.41, 0, 0], [0.2, 0.3, 0]])
GRIPPER_LOCAL = np.array([[0, 0, 0], [0.143, 0, 0], [0.05, 0.03, 0.0]])


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


def _scene(lag=-0.6):
    """Markers (n, 10, 3) of a fixed base and a gripper at three poses."""
    t, q, postures = _joint_trajectory()
    t_mocap = t - lag
    R_b = Rotation.from_euler("xyz", [0.0, 0.0, 0.5]).as_matrix()
    p_b = np.array([1.0, -2.0, 0.3])
    markers = np.full((len(t), 10, 3), np.nan)
    markers[:, list(BASE_TRIAD)] = BASE_LOCAL @ R_b.T + p_b
    step = np.minimum(np.searchsorted([5.9, 11.9], t), 2)  # plateau index
    R_g, p_g = [], []
    for k in range(3):
        R = Rotation.from_euler("xyz", [0.3 * k, 0.2, -0.4 + k]).as_matrix()
        R_g.append(R)
        p_g.append(np.array([0.5 + k, 0.1, 1.0]))
    for i in range(len(t)):
        markers[i, list(GRIPPER_TRIAD)] = GRIPPER_LOCAL @ R_g[step[i]].T + p_g[step[i]]
    return t, q, t_mocap, markers, lag, (R_b, p_b, R_g, p_g)


def test_triad_frame_axes():
    R, p = triad_frames(GRIPPER_LOCAL)
    np.testing.assert_allclose(R[:, 0], [1, 0, 0], atol=1e-12)  # marker 1 -> 2
    np.testing.assert_allclose(R[:, 2], [0, 0, 1], atol=1e-12)  # normal
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)
    np.testing.assert_allclose(p, GRIPPER_LOCAL.mean(axis=0))


def test_world_frame_rows_hold_gripper_and_base_poses():
    pin = pytest.importorskip("pinocchio")
    t, q, tm, markers, lag, (R_b, p_b, R_g, p_g) = _scene()
    table, drops = extract_static_postures_6d(t, q, tm, markers, lag, 2)
    assert list(table.columns) == COLUMNS
    assert drops == {"plateaus": 3, "gripper_lost": 0}
    assert len(table) == 3
    for k in range(3):
        R_t, p_t = triad_frames(GRIPPER_LOCAL @ R_g[k].T + p_g[k])
        want = np.r_[p_t, pin.rpy.matrixToRpy(R_t)]
        np.testing.assert_allclose(
            table[["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]].iloc[k],
            want,
            atol=1e-8,
        )
    R_t, p_t = triad_frames(BASE_LOCAL @ R_b.T + p_b)
    want_b = np.r_[p_t, pin.rpy.matrixToRpy(R_t)]
    np.testing.assert_allclose(
        table[["xb", "yb", "zb", "phixb", "phiyb", "phizb"]].to_numpy(),
        np.tile(want_b, (3, 1)),
        atol=1e-8,
    )
    np.testing.assert_allclose(
        table.iloc[:, 6:14].to_numpy(), q[[200, 800, 1400]], atol=1e-4
    )
    assert (table.points_in_base_frame == 0).all()
    assert (table.rigid_body_def == 2).all()
    assert (table.t_end_robot - table.t_start_robot).min() >= 1.2
    np.testing.assert_allclose(table.pose_std_mm, 0, atol=1e-5)


def test_mocap_window_is_shifted_by_the_lag():
    t, q, tm, markers, lag, _ = _scene()
    step = markers.copy()
    # a gripper jump 0.2 s before the true window start of plateau 1
    sel = (t >= 0.2) & (t < 0.4)
    step[np.flatnonzero(sel)[:, None], list(GRIPPER_TRIAD)] += 0.5
    ok, _ = extract_static_postures_6d(t, q, tm, step, lag)
    bad, _ = extract_static_postures_6d(t, q, tm, step, 0.0)
    ref, _ = extract_static_postures_6d(t, q, tm, markers, lag)
    assert ok.x1[0] == pytest.approx(ref.x1[0], abs=1e-8)
    assert bad.x1[0] != pytest.approx(ref.x1[0], abs=1e-4)


def test_tracking_loss_counts_and_partial_tracking():
    t, q, tm, markers, lag, _ = _scene()
    # gripper lost over all of plateau 2 (robot 6-10 s)
    lost = markers.copy()
    lost[(t > 5.9) & (t < 10.1), 7:10] = np.nan
    table, drops = extract_static_postures_6d(t, q, tm, lost, lag)
    assert len(table) == 2
    assert drops == {"plateaus": 3, "gripper_lost": 1}
    # base lost over plateau 3 (robot 12-16 s): kept, with NaN base columns
    nobase = markers.copy()
    nobase[(t > 11.9) & (t < 16.1), 0:3] = np.nan
    table, drops = extract_static_postures_6d(t, q, tm, nobase, lag)
    assert len(table) == 3 and drops["gripper_lost"] == 0
    assert table.xb.isna().tolist() == [False, False, True]
    assert table.n_base_live.iloc[2] == 0
    # one lost gripper marker makes the whole triad frame not live
    part = markers.copy()
    part[::2, 8] = np.nan
    table, _ = extract_static_postures_6d(t, q, tm, part, lag)
    assert (table.n_live / table.n_window).between(0.45, 0.55).all()


def test_label_exchange_is_repaired_by_side_lengths():
    t, q, tm, markers, lag, _ = _scene()
    swapped = markers.copy()
    idx = np.flatnonzero((t >= 1.0) & (t < 3.0))
    a, b = swapped[idx, 7].copy(), swapped[idx, 8].copy()
    swapped[idx, 7], swapped[idx, 8] = b, a
    fixed, n = repair_triad_labels(swapped, GRIPPER_TRIAD)
    assert n == len(idx)
    np.testing.assert_array_equal(fixed, markers)
    assert not np.array_equal(swapped, markers)  # the input is untouched
    # majority state is kept: swap most of the run and it flips back to it
    most = markers.copy()
    most[:, [7, 8]] = most[:, [8, 7]]
    most[:100, [7, 8]] = most[:100, [8, 7]]
    fixed, n = repair_triad_labels(most, GRIPPER_TRIAD)
    assert n == 100 or n == len(t) - 100
    # base is untouched, and a lost frame stays lost
    fixed2, n_base = repair_triad_labels(swapped, BASE_TRIAD)
    assert n_base == 0
    np.testing.assert_array_equal(fixed2, swapped)
    # the extraction through the exchange equals the clean extraction
    clean, _ = extract_static_postures_6d(t, q, tm, markers, lag)
    table, _ = extract_static_postures_6d(
        t, q, tm, repair_triad_labels(swapped, GRIPPER_TRIAD)[0], lag
    )
    np.testing.assert_allclose(table.to_numpy(), clean.to_numpy(), atol=1e-12)


def test_near_isosceles_triad_is_not_repaired():
    """When |m1m3| and |m2m3| differ by < 2 cm the exchange is undetectable."""
    local = np.array([[0, 0, 0], [0.16, 0, 0], [0.08, 0.1, 0]])
    markers = np.full((50, 10, 3), np.nan)
    markers[:, 7:10] = local
    markers[::2, 7:10] = local[[1, 0, 2]]  # label exchange nobody can see
    out, n = repair_triad_labels(markers, GRIPPER_TRIAD)
    assert n == 0
    np.testing.assert_array_equal(out, markers)


def test_csv_reader_units_clock_and_lost_markers(tmp_path):
    header = "time,base1," + ",".join(MARKERS[1:]) + ",Fx4,Fy4\n"
    live = [0.1 * (k + 1) for k in range(30)]
    lost = [0.0] * 3 + live[3:]
    rows = [
        "2023-09-20_16:08:38.157770," + ",".join(map(str, live)) + ",1,2,3,4,5,6,7,8,9",
        "2023-09-20_16:08:39," + ",".join(map(str, lost)) + ",1,2,3,4,5,6,7,8,9",
    ]
    path = tmp_path / "run.csv"
    path.write_text(header + "\n".join(rows) + "\n")
    t, markers = read_vicon_csv(str(path))
    assert markers.shape == (2, 10, 3)
    # 16:08 local is 14:08 UTC (UTC+2); the second stamp has no fraction
    assert t[0] == pytest.approx(1695218918.15777, abs=1e-5)
    assert t[1] - t[0] == pytest.approx(0.84223, abs=1e-5)
    np.testing.assert_allclose(markers[0].ravel(), live)
    assert np.isnan(markers[1, 0]).all() and not np.isnan(markers[1, 1:]).any()


def test_c3d_reader_converts_units_and_flags_lost_markers(tmp_path):
    c3d = pytest.importorskip("c3d")
    writer = c3d.Writer(point_rate=100.0, point_scale=-0.01)
    writer.set_point_labels(MARKERS + ["*10", "*11"])
    frames = []
    for k in range(3):
        pts = np.zeros((12, 5), dtype=np.float32)
        pts[:, :3] = 1000.0 * (k + 1) + np.arange(36).reshape(12, 3)  # mm
        pts[:, 3] = 1.0
        frames.append((pts, np.zeros((0, 0), dtype=np.float32)))
    frames[1][0][3, 3] = -1.0  # shoulder1 lost in frame 1
    writer.add_frames(frames)
    path = tmp_path / "run.c3d"
    with open(path, "wb") as f:
        writer.write(f)
    t, markers = read_vicon_c3d(str(path))
    assert markers.shape == (3, 10, 3)
    np.testing.assert_allclose(t, [0.0, 0.01, 0.02])
    assert markers[0, 0, 0] == pytest.approx(1.0)  # 1000 mm -> 1 m
    assert np.isnan(markers[1, 3]).all() and not np.isnan(markers[1, 4]).any()


def test_extract_session_refuses_wrong_hash_and_kind(tmp_path):
    bag = tmp_path / "not_a_bag.bag"
    bag.write_bytes(b"x")
    mocap = tmp_path / "run.csv"
    mocap.write_bytes(b"y")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_session(
            str(mocap), str(bag), "x.urdf", lag=0.0, expected_sha256={"bag": "0" * 64}
        )
    with pytest.raises(ValueError, match="kind must be"):
        extract_session(str(mocap), str(bag), "x.urdf", kind="bag")


VICON = Path(__file__).resolve().parents[1] / "examples/tiago/data/calibration/vicon"


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
@pytest.mark.parametrize("session", ["2023-09-20-1508", "2023-09-20-1607"])
def test_shipped_files_are_re_created_byte_for_byte(session, tmp_path):
    """Original Vicon export plus bag (named by sha256) to the shipped CSV."""
    manifest = yaml.safe_load((VICON / "protocol.yaml").read_text())
    entry = next(s for s in manifest["sessions"] if s["id"] == session)
    source = entry["source"]
    if source["mocap"].endswith(".c3d"):
        pytest.importorskip("c3d")
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    paths = {}
    for key, name in (("mocap", source["mocap"]), ("bag", source["bag"])):
        paths[key] = next(root.rglob(name), None)
        assert paths[key] is not None, f"{name} not under {root}"
    assert file_sha256(paths["mocap"]) == source["mocap_sha256"]
    assert file_sha256(paths["bag"]) == source["sha256"]
    urdf = (
        Path(__file__).resolve().parents[1] / "examples/tiago/urdf/tiago_48_schunk.urdf"
    )
    table, lag, drops = extract_session(
        str(paths["mocap"]),
        str(paths["bag"]),
        str(urdf),
        rigid_body_def=entry["rigid_body"],
    )
    assert abs(lag - source["lag_s"]) < 5e-4
    assert {k: drops[k] for k in entry["drops"]} == entry["drops"]
    assert {
        "gripper": drops["swapped_gripper"],
        "base": drops["swapped_base"],
    } == entry["relabelled_frames"]
    out = tmp_path / "out.csv"
    table.to_csv(out, index=False)
    (name,) = entry["files"]
    assert out.read_bytes() == (VICON / name).read_bytes()
