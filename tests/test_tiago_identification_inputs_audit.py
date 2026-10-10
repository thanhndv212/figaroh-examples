"""Findings of the TIAGo identification-input audit reproduce from shipped files (#68)."""

import sys
from pathlib import Path

import pytest

TIAGO = Path(__file__).resolve().parents[1] / "examples/tiago"
sys.path.insert(0, str(TIAGO.parents[1]))

import examples.tiago.identification_inputs_audit as audit  # noqa: E402


@pytest.fixture(scope="module")
def filters():
    return {run: audit.velocity_filter(run) for run in audit.RUNS}


def test_velocity_is_a_first_order_filter_on_the_header_clock(filters):
    for run, joints in filters.items():
        for joint, fit in joints.items():
            assert fit["a"] == pytest.approx(0.95, abs=0.002), (run, joint)
            assert fit["residual"] < 2e-4, (run, joint)
            # a pure shift is two orders of magnitude worse than the filter
            assert fit["shift_residual"] > 100 * fit["residual"], (run, joint)
            # the nominal 10 ms clock is 10x worse than the header clock
            assert fit["nominal_clock_residual"] > 10 * fit["residual"]
    shifts = {f["best_shift"] for j in filters.values() for f in j.values()}
    assert min(shifts) <= 10 and max(shifts) >= 20  # the shift is not constant


def test_torso_levels():
    levels = audit.torso_levels()
    assert set(levels) == {20, 40, 60, 80}
    for n, r in levels.items():
        assert 1.70 <= r["lifting"] <= 1.80, n
        assert -1.9 <= r["rest"] <= -1.0, n
        assert r["rest_before"] < -3.5 < 0 < r["rest_after"], n
    assert levels[20]["peak_speed"] < levels[80]["peak_speed"]


def test_converted_torso_force_is_the_urdf_weight():
    share = audit.torso_model_share()
    assert share["subtree_mass"] == pytest.approx(18.53, abs=0.01)
    assert share["share"] > 0.98


def test_controller_constants():
    constants = {j: v for j, (v, _) in audit.controller_constants().items()}
    assert constants == {
        "arm_1_joint": 0.0,
        "arm_2_joint": 0.136,
        "arm_3_joint": -0.087,
        "arm_4_joint": -0.087,
    }
    # the config's drive table agrees except for arm_1 (#68)
    assert audit.pc.KMOTOR["arm_1_joint"] == 0.136
    assert all(audit.pc.KMOTOR[j] == c for j, c in constants.items() if c)


def test_differential_wrist_identities_are_exact():
    d = audit.differential_wrist()
    assert d["samples"] == 2006
    for key in ("q6", "q7", "tau6", "tau7"):
        assert d[key] < 1e-12, key
    assert d["equal_efforts"] == pytest.approx(0.944, abs=0.005)


def test_wrist_effort_quantisation():
    q = audit.quantisation()
    for joint in (
        "arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
    ):
        assert q[joint]["step"] == pytest.approx(0.001, abs=1e-9), joint
    assert q["torso_lift_joint"]["step"] == pytest.approx(0.01, abs=1e-9)
    for joint in ("arm_5_joint", "arm_6_joint", "arm_7_joint"):
        assert 0.87 <= q[joint]["zero_fraction"] <= 0.91, joint
        assert q[joint]["distinct"] < 50
    for joint in ("arm_2_joint", "arm_3_joint", "arm_4_joint"):
        assert q[joint]["zero_fraction"] < 0.01


def test_end_effector():
    e = audit.end_effector(("dynamic",))
    assert e["hey5"] == pytest.approx(1.032, abs=1e-3)
    assert e["schunk"] == pytest.approx(0.865, abs=1e-3)
    assert e["gripper_channels"] == 0 and e["hand_joint_channels"] == 36
    # the sensor sees 0.79 kg below it: lighter than either URDF's hand
    mass = e["ft_mass"]["dynamic"]
    assert 0.78 <= mass <= 0.81
    assert mass < e["schunk"] < e["hey5"]


def test_channel_status_and_end_effector_files():
    import pandas as pd

    status = pd.read_csv(audit.AUDIT / "channel_status.csv")
    assert (status["torque_sensor_nan_fraction"] == 1.0).all()
    assert (status["motor_mode_values"].astype(str) == "0").all()
    assert (status["max_abs_effort_command"].dropna() == 0.0).all()


def test_arm1_axis_is_vertical_and_payload_torque_is_small():
    a = audit.arm1_payload()
    assert a["axis"]["min_abs_z"] > 0.9999
    assert a["axis"]["max_gravity"] < 1e-9
    assert a["payload_mass"] == pytest.approx(0.489)
    assert a["payload_rms"] == pytest.approx(0.10, abs=0.02)
    assert a["payload_rms"] < 0.15 * a["model_error_rms"]
