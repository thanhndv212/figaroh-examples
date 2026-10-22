"""TIAGo backlash surface (#71): the model on synthetic data and its guards.

Synthetic signals only, plus the validation guard and the frozen-config
consistency. No real-data fit.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import backlash_model as bm  # noqa: E402
from examples.tiago import backlash_truth as bt  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_backlash_truth.json"


def _signal(n=12000, dt=0.01, seed=0):
    t = np.arange(n) * dt
    q = 0.6 * np.sin(2 * np.pi * t / 10.0) + 0.3 * np.sin(2 * np.pi * t / 37.0)
    v = np.gradient(q, dt)
    return t, q, v


def _joint_data(t, q, d, width_h=None, tau=None):
    h, valid = bm.direction_state(t, np.gradient(q, t))
    tau = np.zeros_like(t) if tau is None else tau
    return bm.JointData("j", t, q, q - d, h, tau, np.zeros(t.size, int), valid)


def test_direction_state_follows_the_sign_and_masks_the_reversal():
    t, q, v = _signal()
    h, valid = bm.direction_state(t, v)
    moving = np.abs(v) > 0.1
    assert np.all(h[moving & valid] == np.sign(v[moving & valid]))
    assert bm.reversal_count(h) >= 10
    rev = np.flatnonzero(np.diff(h) != 0) + 1
    assert not valid[rev].any()
    assert np.mean(valid) > 0.4


def test_play_operator_is_recovered_by_the_h_model():
    """d = c + E(q_abs) + b (play of width W): the fit returns W within 3 %."""
    t, q, _ = _signal()
    width = 0.012
    b = bt.play(q, width, 1.0)
    d = 0.05 + 0.02 * q + b
    jd = _joint_data(t, q, d)
    f = bm.fit_groups(jd, bm.make_knots(jd.q_abs), ("E", "W"))
    assert f.W == pytest.approx(width, rel=0.03)


def test_spline_separates_an_angle_step_from_the_width():
    """A 170 mrad step in E (function of q_abs) gives W = 0 when there is none."""
    t, q, _ = _signal()
    t, x, _ = _signal()  # x is q_abs
    d = 0.01 + 0.17 * np.clip((x - 0.2) / 0.1, 0.0, 1.0)  # knot-aligned ramp
    q = x + d
    h, valid = bm.direction_state(t, np.gradient(q, t))
    jd = bm.JointData("j", t, q, x, h, np.zeros(t.size), np.zeros(t.size, int), valid)
    f = bm.fit_groups(jd, bm.make_knots(jd.q_abs), ("E", "W"))
    assert abs(f.W) < 1e-4


def test_hat_basis_is_a_partition_of_unity_and_clamps():
    knots = bm.make_knots(np.array([0.0, 1.0]), 0.1)
    x = np.array([-0.5, 0.0, 0.123, 0.5, 1.0, 2.0])
    B = bm.hat_basis(x, knots)
    assert np.allclose(B.sum(1), 1.0)
    assert np.allclose(B[0], B[1]) and np.allclose(B[-1], B[-2])


def test_smoothness_penalty_interpolates_an_unvisited_stretch():
    knots = np.linspace(0.0, 1.0, 11)
    x = np.concatenate([np.linspace(0.0, 0.35, 400), np.linspace(0.65, 1.0, 400)])
    y = 0.03 * x
    jd = bm.JointData(
        "j",
        np.arange(x.size) * 0.01,
        x,
        x - y,
        np.ones(x.size),
        np.zeros(x.size),
        np.zeros(x.size, int),
        np.ones(x.size, bool),
    )
    f = bm.fit_groups(jd, knots, ("E",))
    gap = bm.JointData(
        "j",
        np.arange(3) * 0.01,
        np.full(3, 0.5),
        np.full(3, 0.5),
        np.ones(3),
        np.zeros(3),
        np.zeros(3, int),
        np.ones(3, bool),
    )
    ref = bm.JointData(
        "j",
        np.arange(3) * 0.01,
        np.full(3, 0.0),
        np.full(3, 0.0),
        np.ones(3),
        np.zeros(3),
        np.zeros(3, int),
        np.ones(3, bool),
    )
    e = f.terms(gap) - f.terms(ref)
    assert e[0] == pytest.approx(0.03 * 0.5, abs=1e-3)


def test_jump_detector_finds_an_encoder_jump_and_not_a_backlash_flip():
    t, q, _ = _signal()
    d = 0.01 + 0.004 * bt.play(q, 0.04, 1.0) / 0.02
    assert bm.jump_indices(d, q).size == 0
    still = np.flatnonzero(np.abs(np.diff(q, prepend=q[0])) < 5e-4)
    i1, i2 = (int(still[still > k][0]) for k in (5000, 8000))
    d2 = d.copy()
    d2[i1:] += 0.06
    d2[i2:] -= 0.03
    found = bm.jump_indices(d2, q)
    assert [int(i) for i in found] == [i1, i2]
    seg = bm.segment_ids(t.size, found)
    assert seg[0] == 0 and seg[i1] == 1 and seg[-1] == 2


def test_validation_absolute_encoder_is_refused_without_the_unlock():
    sessions = bm.load_sessions(verify=False)
    roles = bm.role_sets(sessions)
    val = roles["validation"][0]
    with pytest.raises(PermissionError):
        bm.read_target(val, "arm_5_joint")
    with pytest.raises(PermissionError):
        bm.read_targets(val, ["arm_5_joint"])
    for diag in roles["diagnostic"]:
        with pytest.raises(PermissionError):
            bm.read_target(diag, "arm_5_joint")
    # the regressor columns of the validation run are readable
    run = bm.read_regressors(val)
    assert run.q.shape[1] == len(bm.CONFIG_JOINTS)


def test_frozen_config_is_complete_and_the_golden_quotes_it():
    s = bm.SETTINGS
    for key in ("sigma_d_mrad", "rho", "quantization_mrad"):
        assert set(s["noise"][key]) == set(bm.JOINTS), key
    assert s["noise"]["used_level"] in s["noise"]["levels"]
    assert all(0 <= r < 1 for r in s["noise"]["rho"].values())
    g = json.loads(GOLDEN.read_text())
    for key in ("sigma_d_mrad", "rho", "quantization_mrad"):
        assert g["noise"][key] == s["noise"][key]
    assert g["seeds"] == s["fixture"]["seeds"]
    assert s["urdf"]["backlash_days"] == "tiago_48_schunk.urdf"
    label = (
        ROOT / "examples/tiago/config/tiago_backlash_surface_config.yaml"
    ).read_text()
    assert "target: relative_encoder_minus_absolute_encoder" in label
