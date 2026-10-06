"""TIAGo dynamic-signal regressions from the D2 audit (#20).

Synthetic CSVs with a known clock, known derivatives and a known velocity
delay; expected values are computed here, not by the code under test.
See docs/development/tiago-signal-audit-2026-10-03.md.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from examples.tiago.utils.tiago_tools import (
    TiagoIdentification,
    duplicate_channel_fractions,
    estimate_velocity_lag,
    zero_fractions,
)

JOINTS = ["torso_lift_joint", "arm_1_joint", "arm_2_joint"]
TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"


def _signals(n=1200, rate=100.0, delay=0, seed=3):
    """Jittered ~rate Hz clock, smooth positions, velocity delayed by `delay`."""
    rng = np.random.default_rng(seed)
    t = np.cumsum(np.r_[0.0, (1 / rate) * (1 + 0.1 * rng.uniform(-1, 1, n - 1))])
    freqs = np.array([0.3, 0.5, 0.7])
    q = np.sin(2 * np.pi * freqs * t[:, None])
    dq = 2 * np.pi * freqs * np.cos(2 * np.pi * freqs * t[:, None])
    v = np.vstack(
        [np.repeat(dq[:1], delay, axis=0), dq[: n - delay]]
    )  # v[n] = dq[n - delay]
    return t, q, v


def _write(dirpath, t, q, v, tau, joints=JOINTS):
    for kind, arr in (("position", q), ("velocity", v), ("effort", tau)):
        df = pd.DataFrame(arr, columns=[f"- {j}_{kind}" for j in joints])
        df.insert(0, "t", t)
        df.to_csv(dirpath / f"tiago_{kind}.csv", index=False)


def _robot():
    import types

    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(TIAGO / "urdf" / "tiago_48_schunk.urdf"))
    return types.SimpleNamespace(model=model, data=model.createData())


def _adapter(dirpath, f_sample=100.0, lag="auto"):
    iden = TiagoIdentification.__new__(TiagoIdentification)
    iden.robot = _robot()
    iden.identif_config = {
        "active_joints": JOINTS,
        "pos_data": str(dirpath / "tiago_position.csv"),
        "vel_data": str(dirpath / "tiago_velocity.csv"),
        "torque_data": str(dirpath / "tiago_effort.csv"),
        # unit drive constants: the converted effort is the recorded one
        # (+ the torso gravity term); the recorded one is kept as effort_raw
        "reduction_ratio": {j: 1 for j in JOINTS},
        "kmotor": {j: 1 for j in JOINTS},
    }
    iden.filter_config = {"filter_params": {"f_sample": f_sample}}
    iden.velocity_lag = lag
    return iden


@pytest.mark.parametrize("delay", [0, 7, 18])
def test_velocity_lag_estimate_recovers_known_delay(delay):
    t, q, v = _signals(delay=delay)
    assert estimate_velocity_lag(t, q, v, max_lag=40) == delay


def test_velocity_lag_estimate_refuses_search_limit():
    t, q, v = _signals(delay=30)
    with pytest.raises(ValueError, match="search limit"):
        estimate_velocity_lag(t, q, v, max_lag=10)


def test_loader_aligns_velocity_and_keeps_recorded_clock(tmp_path):
    delay = 7
    t, q, v = _signals(delay=delay)
    tau = np.arange(len(t) * 3, dtype=float).reshape(-1, 3)
    _write(tmp_path, t, q, v, tau)

    iden = _adapter(tmp_path)
    traj = iden.load_trajectory_data()
    data = traj.to_legacy()

    n = len(t) - delay
    close = {"rtol": 0, "atol": 1e-12}  # CSV round trip
    np.testing.assert_allclose(data["timestamps"].ravel(), t[:n], **close)
    np.testing.assert_allclose(data["positions"], q[:n], **close)
    np.testing.assert_allclose(traj.effort_raw, tau[:n], **close)
    np.testing.assert_array_equal(traj.sample_index, np.arange(n))
    # Shifted velocity equals the true derivative at the same timestamps.
    true_dq = v[delay:]  # v[n + delay] = dq[n]
    np.testing.assert_allclose(data["velocities"], true_dq, **close)
    assert data["accelerations"] is None
    prov = iden.trajectory_provenance["training"]
    assert prov["velocity_lag_samples"] == delay
    assert prov["timing_source"] == "recorded"
    assert prov["recorded_rate_hz"] == pytest.approx(100.0, rel=0.02)
    assert prov["dropped_trailing_rows"] == delay


def test_fixed_lag_overrides_estimate(tmp_path):
    t, q, v = _signals(delay=7)
    _write(tmp_path, t, q, v, np.zeros((len(t), 3)))
    traj = _adapter(tmp_path, lag=0).load_trajectory_data()
    np.testing.assert_allclose(traj.dq, v, rtol=0, atol=1e-12)


def test_filter_clock_must_match_recorded_clock(tmp_path):
    t, q, v = _signals()
    _write(tmp_path, t, q, v, np.zeros((len(t), 3)))
    with pytest.raises(ValueError, match="does not match the recorded clock"):
        _adapter(tmp_path, f_sample=500.0).load_trajectory_data()


def test_files_must_share_one_clock(tmp_path):
    t, q, v = _signals()
    _write(tmp_path, t, q, v, np.zeros((len(t), 3)))
    eff = pd.read_csv(tmp_path / "tiago_effort.csv")
    eff["t"] += 0.001
    eff.to_csv(tmp_path / "tiago_effort.csv", index=False)
    with pytest.raises(ValueError, match="effort timestamps differ"):
        _adapter(tmp_path).load_trajectory_data()


def test_channels_are_selected_by_exact_name(tmp_path):
    t, q, v = _signals()
    _write(
        tmp_path,
        t,
        q,
        v,
        np.zeros((len(t), 3)),
        joints=["torso_lift_joint", "arm_1_joint", "arm_22_joint"],
    )
    with pytest.raises(ValueError, match="missing columns"):
        _adapter(tmp_path).load_trajectory_data()


def test_shared_zeros_are_not_duplicates():
    rng = np.random.default_rng(0)
    a = np.where(rng.uniform(size=1000) < 0.9, 0.0, rng.normal(size=1000))
    b = np.where(rng.uniform(size=1000) < 0.9, 0.0, rng.normal(size=1000))
    channels = np.c_[a, b, a]  # third channel copies the first
    found = duplicate_channel_fractions(channels, ["a", "b", "c"])
    assert found == {"a/c": 1.0}
    zeros = zero_fractions(channels, ["a", "b", "c"])
    assert zeros["a"] == pytest.approx(np.mean(a == 0))


def test_shipped_config_filters_at_the_recorded_clock():
    """The 500 Hz design applied to ~100 Hz data cut at ~0.36 Hz (#20)."""
    cfg = yaml.safe_load(open(TIAGO / "config" / "tiago_unified_config.yaml"))
    sp = cfg["tasks"]["identification"]["signal_processing"]
    t = pd.read_csv(
        TIAGO / "data/identification/dynamic/tiago_position.csv", usecols=["t"]
    )["t"]
    recorded = 1 / np.median(np.diff(t.to_numpy()))
    assert sp["filter_params"]["f_sample"] == pytest.approx(recorded, rel=0.05)
    assert sp["sampling_frequency"] == sp["filter_params"]["f_sample"]
    assert sp["cutoff_frequency"] == sp["filter_params"]["f_butter"]
