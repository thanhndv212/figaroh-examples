"""Synthetic touches at the real TALOS postures (figaroh-examples#98).

The real recorded postures are projected onto the *true* table surface
(planar or a 1 mm bowl) and re-solved by IK, so counts and posture
distribution match the hardware data while the true gap is exactly zero.
"""

from __future__ import annotations

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

pytest.importorskip("pinocchio")

import examples.talos_table_contact.generate_synthetic_data as gen  # noqa: E402

COUNTS = {"left": (21, 9), "right": (29, 9)}
# Real postures sit up to ~25 mm off the true surface; moving them onto it
# takes up to ~65 mrad of joint motion (measured, seed 0).
MAX_DEV_RAD = 0.1


@pytest.fixture(scope="module")
def robot():
    return gen._load_robot()


@pytest.fixture(scope="module")
def built(robot):
    """(side, surface kind) -> build_anchored_truth output, seed 0."""
    cache = {}

    def get(side, kind, seed=0):
        key = (side, kind, seed)
        if key not in cache:
            with contextlib.redirect_stdout(io.StringIO()):
                cache[key] = gen.build_anchored_truth(robot, side, seed, kind)
        return cache[key]

    return get


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("kind", ["planar", "curved"])
def test_projected_postures_have_zero_true_gap(built, side, kind):
    out = built(side, kind)
    chain = gen.CHAINS[side]
    n_train, n_val = COUNTS[side]
    assert len(out["q_train"]) == n_train and len(out["q_val"]) == n_val
    for q in (out["q_train"], out["q_val"]):
        gap = gen.true_gap(
            out["truth"], q, out["surface"], chain["base"], chain["wrist"]
        )
        assert np.abs(gap).max() < 1e-6
    # the real postures are NOT on the surface (gap of order mm), so the
    # test above is not vacuous
    real = gen.true_gap(
        out["truth"], out["q_real_train"], out["surface"], chain["base"], chain["wrist"]
    )
    assert np.abs(real[:, 0]).max() > 1e-3


@pytest.mark.parametrize("side", ["left", "right"])
def test_synthesized_postures_stay_near_the_real_ones(built, side):
    out = built(side, "curved")
    for part in ("train", "val"):
        assert out["info"][part]["max_dev_rad"] < MAX_DEV_RAD
    ids = out["truth"]["active_joint_ids"]
    # joints outside the chain are untouched
    inactive = np.ones(out["q_train"].shape[1], bool)
    robot_model = out["truth"]["true_model"]
    inactive[[robot_model.joints[j].idx_q for j in ids]] = False
    np.testing.assert_array_equal(
        out["q_train"][:, inactive], out["q_real_train"][:, inactive]
    )


def test_curved_surface_sag(built):
    out = built("left", "curved")
    s = out["surface"]
    assert s.kind == "curved" and s.sag == gen.CURVED_SAG
    # sag is reached at the stored (90th-percentile) radius; centre is flat
    assert float(s.height(s.center[0] + s.radius, s.center[1])) == pytest.approx(1e-3)
    assert float(s.height(*s.center)) == 0.0
    assert np.allclose(s.normal(*s.center), [0.0, 0.0, 1.0])


def test_planar_true_gap_is_the_plane_gap(built):
    out = built("left", "planar")
    chain = gen.CHAINS["left"]
    truth, q = out["truth"], out["q_real_train"]
    gap = gen.true_gap(truth, q, out["surface"], chain["base"], chain["wrist"])
    rels = gen._contacts_in_table(
        truth["true_model"], truth, q, chain["base"], chain["wrist"]
    )
    expected = np.array([gen.gap_of_pose(gen.pin.SE3.Identity(), r) for r in rels])
    np.testing.assert_allclose(gap, expected, atol=1e-12)


def test_same_seed_is_bit_identical(built, robot):
    a = built("left", "curved")
    with contextlib.redirect_stdout(io.StringIO()):
        b = gen.build_anchored_truth(robot, "left", 0, "curved")
    np.testing.assert_array_equal(a["q_train"], b["q_train"])
    np.testing.assert_array_equal(a["q_val"], b["q_val"])
    c = built("left", "curved", seed=1)
    assert not np.array_equal(a["q_train"], c["q_train"])


def test_failed_targets_raise(built, monkeypatch):
    out = built("left", "planar")
    chain = gen.CHAINS["left"]
    monkeypatch.setattr(
        gen, "solve_touch_ik", lambda *a, **k: (np.asarray(a[6]).copy(), False)
    )
    with pytest.raises(RuntimeError, match="IK failed for 21 of 21"):
        gen.synthesize_touches_at_postures(
            out["truth"]["true_model"],
            out["truth"],
            out["q_real_train"],
            0,
            out["surface"],
            chain["base"],
            chain["wrist"],
        )
