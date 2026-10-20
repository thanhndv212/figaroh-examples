"""Static-posture extraction rules for the TALOS torso-arm sessions (#114).

The original recordings are not distributed; these synthetic cases pin the
rules in examples/talos/utils/mocap_extraction.py that produced the shipped
files. The byte-for-byte re-creation needs the bags (TALOS_RECORDINGS_ROOT).
"""

import os
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from examples.talos.utils.mocap_extraction import (
    BODIES,
    MAX_STD_MM,
    audit_streams,
    columns,
    estimate_clock_lag,
    extract_session,
    extract_static_postures,
    file_sha256,
    joint_names,
    static_plateaus,
)

RATE = 100.0


def _joints():
    """Three 4 s static postures joined by 2 s ramps, nine joints, 100 Hz."""
    postures = np.array([[0.1] * 9, [0.2] * 9, [-0.3] * 9])
    segments = []
    for k, p in enumerate(postures):
        segments.append(np.tile(p, (int(4 * RATE), 1)))
        if k + 1 < len(postures):
            ramp = np.linspace(0, 1, int(2 * RATE))[:, None]
            segments.append(p + ramp * (postures[k + 1] - p))
    q = np.vstack(segments)
    return np.arange(len(q)) / RATE, q


def _body(t, lag, rate, poses):
    """Stream ``t, x, y, z, qx, qy, qz, qw`` at ``rate`` Hz; pose per plateau."""
    tm = np.arange(t[0], t[-1], 1.0 / rate) - lag
    step = np.minimum(np.searchsorted([5.9, 11.9], tm + lag), 2)
    out = np.zeros((len(tm), 8))
    out[:, 0] = tm
    for i, k in enumerate(step):
        out[i, 1:] = poses[k]
    return out


def _poses(offset):
    return [
        np.r_[
            [offset + k, 0.1 * k, 1.0],
            Rotation.from_euler("xyz", [0.2 * k, 0.1, -0.3]).as_quat(),
        ]
        for k in range(3)
    ]


def _scene(lag=0.3):
    t, q = _joints()
    bodies = {
        "waist": _body(t, lag, 50.0, _poses(0.0)),
        "upper_body": _body(t, lag, 33.0, _poses(10.0)),
        "endeffector": _body(t, lag, 2.0, _poses(20.0)),
    }
    return t, q, bodies, lag


def test_joint_names_and_columns():
    assert joint_names("left")[:2] == ["arm_left_1_joint", "arm_left_2_joint"]
    assert joint_names("right")[6:] == [
        "arm_right_7_joint",
        "torso_1_joint",
        "torso_2_joint",
    ]
    with pytest.raises(ValueError, match="arm must be"):
        joint_names("both")
    cols = columns("left")
    assert len(cols) == 3 + 9 + 3 * 9 and len(set(cols)) == len(cols)
    assert cols[:3] == ["plateau", "t_start_robot", "t_end_robot"]


def test_plateaus_found_by_joint_tolerance():
    t, q = _joints()
    found = static_plateaus(t, q)
    assert len(found) == 3
    assert all(t[b - 1] - t[a] >= 3.0 for a, b in found)
    q2 = q.copy()
    q2[:, 3] += 2e-3 * np.sin(np.arange(len(t)))  # jitter above 1 mrad
    assert static_plateaus(t, q2) == []


def test_rows_hold_world_frame_means_and_windows():
    t, q, bodies, lag = _scene()
    table, drops = extract_static_postures(t, q, bodies, lag, "left")
    assert list(table.columns) == columns("left")
    assert drops == {"plateaus": 3, "too_few_samples": 0, "unsteady": 0}
    assert table.plateau.tolist() == [0, 1, 2]
    np.testing.assert_allclose(
        table[joint_names("left")].to_numpy()[:, 0], [0.1, 0.2, -0.3], atol=2e-3
    )
    for k in range(3):
        np.testing.assert_allclose(
            table[["upper_body_x", "upper_body_y", "upper_body_z"]].iloc[k],
            [10.0 + k, 0.1 * k, 1.0],
            atol=1e-9,
        )
        want = Rotation.from_euler("xyz", [0.2 * k, 0.1, -0.3]).as_quat()
        want = want if want[3] >= 0 else -want
        np.testing.assert_allclose(
            table[[f"endeffector_q{c}" for c in "xyzw"]].iloc[k], want, atol=1e-9
        )
    assert (table.endeffector_qw >= 0).all()
    assert (table.n_endeffector >= 3).all() and (table.n_waist >= 20).all()
    np.testing.assert_allclose(table.waist_std_mm, 0, atol=1e-9)
    assert (table.t_end_robot - table.t_start_robot).min() >= 2.5


def test_mocap_window_is_shifted_by_the_lag():
    t, q, bodies, lag = _scene()
    jump = {k: v.copy() for k, v in bodies.items()}
    # a waist jump on the mocap clock inside the window of a wrong (zero) lag
    # but past the end of the true window of plateau 0
    sel = (jump["waist"][:, 0] >= 3.5) & (jump["waist"][:, 0] < 3.65)
    jump["waist"][sel, 1] += 0.5
    ok, _ = extract_static_postures(t, q, jump, lag, "left")
    bad, _ = extract_static_postures(t, q, jump, 0.0, "left")
    ref, _ = extract_static_postures(t, q, bodies, lag, "left")
    assert ok.waist_x[0] == pytest.approx(ref.waist_x[0], abs=1e-9)
    assert bad.waist_x[0] != pytest.approx(ref.waist_x[0], abs=1e-4)


def test_sparse_body_and_unsteady_plateaus_are_dropped():
    t, q, bodies, lag = _scene()
    few = {k: v.copy() for k, v in bodies.items()}
    ee = few["endeffector"]
    few["endeffector"] = ee[(ee[:, 0] + lag < 5.9) | (ee[:, 0] + lag > 11.9)]
    table, drops = extract_static_postures(t, q, few, lag, "left")
    assert drops["too_few_samples"] == 1 and len(table) == 2
    assert table.plateau.tolist() == [0, 2]
    noisy = {k: v.copy() for k, v in bodies.items()}
    w = noisy["waist"]
    inside = (w[:, 0] + lag > 6.5) & (w[:, 0] + lag < 9.5)
    w[inside, 1] += np.where(np.arange(inside.sum()) % 2 == 0, 0.05, -0.05)
    table, drops = extract_static_postures(t, q, noisy, lag, "left")
    assert drops["unsteady"] == 1 and table.plateau.tolist() == [0, 2]
    assert MAX_STD_MM == 5.0


def test_clock_lag_is_recovered_from_torso_motion():
    rng = np.random.default_rng(0)
    t = np.arange(0, 120, 0.016)
    # torso motions of random speed in bursts
    burst = np.sin(2 * np.pi * t / 7.3) + 0.5 * np.sin(2 * np.pi * t / 3.1)
    q_torso = np.column_stack([burst, 0.6 * np.cos(2 * np.pi * t / 5.3)])
    lag = 0.44
    tm = np.arange(t[0], t[-1], 0.03) - lag
    angle = np.interp(tm + lag, t, q_torso[:, 0])
    quat = Rotation.from_euler("z", angle).as_quat()
    body = np.column_stack([tm, np.zeros((len(tm), 3)), quat])
    body[:, 1:4] += 1e-5 * rng.standard_normal((len(tm), 3))
    got, corr = estimate_clock_lag(t, q_torso, body)
    assert got == pytest.approx(lag, abs=0.04)
    assert corr > 0.9


def test_audit_counts_frozen_samples_and_tracking_loss():
    t, q, bodies, _ = _scene()
    w = bodies["waist"]
    w = np.delete(w, np.arange(100, 140), axis=0)  # a 0.8 s gap, 40 median periods
    w[300] = w[299]  # a frozen sample
    audit = audit_streams(t, {**bodies, "waist": w})
    assert audit["waist"]["tracking_loss_gaps"] == 1
    assert audit["waist"]["tracking_loss_s"] == pytest.approx(0.82, abs=0.05)
    assert audit["waist"]["frozen"] >= 1 and audit["waist"]["nan"] == 0
    assert audit["robot"]["rate_hz"] == RATE
    assert audit["upper_body"]["tracking_loss_gaps"] == 0


def test_extract_session_refuses_wrong_hash(tmp_path):
    bag = tmp_path / "not_a_bag.bag"
    bag.write_bytes(b"x")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_session(str(bag), "left", lag=0.0, expected_sha256="0" * 64)
    with pytest.raises(ValueError, match="arm must be"):
        extract_session(str(bag), "both", lag=0.0)


MOCAP = Path(__file__).resolve().parents[1] / "examples/talos/data/calibration/mocap"


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TALOS_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TALOS_RECORDINGS_ROOT",
)
@pytest.mark.parametrize(
    "session",
    ["2022-03-15-1354-left", "2022-03-15-1418-right", "2022-03-15-1439-right"],
)
def test_shipped_files_are_re_created_byte_for_byte(session, tmp_path):
    """Original bag (named by sha256) to the shipped CSV, lag, drops, audit."""
    pytest.importorskip("rosbags")
    manifest = yaml.safe_load((MOCAP / "protocol.yaml").read_text())
    entry = next(s for s in manifest["sessions"] if s["id"] == session)
    source = entry["source"]
    root = Path(os.environ["TALOS_RECORDINGS_ROOT"])
    bag = next(root.rglob(source["bag"]), None)
    assert bag is not None, f"{source['bag']} not under {root}"
    assert file_sha256(bag) == source["sha256"]
    table, lag, drops, audit = extract_session(str(bag), entry["arm"])
    assert lag == pytest.approx(source["lag_s"], abs=1e-9)
    assert drops == entry["drops"]
    assert audit == entry["audit"]
    out = tmp_path / "out.csv"
    table.to_csv(out, index=False)
    (name,) = entry["files"]
    assert out.read_bytes() == (MOCAP / name).read_bytes()
    assert BODIES  # the three bodies of the layout
