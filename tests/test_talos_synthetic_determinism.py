"""The synthetic TALOS dataset must not depend on IK retry outcomes.

IK convergence can differ between platforms (Pinocchio build, BLAS). If
retries drew from the same random stream as the touch targets, one extra
retry would shift every later target and silently produce a different
dataset (examples#14). These tests replace the IK solve with a stub, so
they check the random-stream layout without running IK.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

pytest.importorskip("pinocchio")

import examples.talos_table_contact.generate_synthetic_data as gen  # noqa: E402


@pytest.fixture(scope="module")
def setup():
    robot = gen._load_robot()
    ground_truth = gen.build_ground_truth(
        robot.model,
        np.random.default_rng(0),
        n_sessions=2,
        session_offsets=[(0.0, 0.0), (0.03, -0.05)],
    )
    return robot.model, ground_truth


def _targets(monkeypatch, setup, fail_first_attempt_every):
    """Run synthesize_touches with a stub IK; return the requested targets."""
    model, ground_truth = setup
    targets, calls = [], {"n": 0}

    def stub(model_, data_, base, wrist, offset, target, q0, config_idx):
        calls["n"] += 1
        first_attempt = len(targets) == 0 or target is not targets[-1]
        if first_attempt:
            targets.append(target)
        fail = (
            first_attempt
            and fail_first_attempt_every
            and len(targets) % fail_first_attempt_every == 0
        )
        return q0.copy(), not fail

    monkeypatch.setattr(gen, "solve_touch_ik", stub)
    df = gen.synthesize_touches(
        model, ground_truth, 2, 6, np.random.default_rng(1), encoder_noise_std=1e-4
    )
    return [t.homogeneous.copy() for t in targets], df, calls["n"]


def test_targets_do_not_depend_on_ik_retries(monkeypatch, setup):
    clean, df_clean, calls_clean = _targets(monkeypatch, setup, 0)
    retried, df_retried, calls_retried = _targets(monkeypatch, setup, 3)

    assert calls_retried > calls_clean  # retries actually happened
    assert len(clean) == len(retried) == 12
    for a, b in zip(clean, retried):
        np.testing.assert_array_equal(a, b)
    # Every target still converges (on its second attempt), so both runs
    # record the same number of touches.
    assert len(df_clean) == len(df_retried) == 12


def test_noise_for_a_target_does_not_depend_on_earlier_retries(monkeypatch, setup):
    _, df_clean, _ = _targets(monkeypatch, setup, 0)
    _, df_retried, _ = _targets(monkeypatch, setup, 3)
    # Targets before the first retry are unaffected, and so are targets
    # after it: their recorded angles (stub q = seed + noise) match exactly
    # except where the retried seed itself was perturbed.
    retried_rows = {i for i in range(12) if (i + 1) % 3 == 0}
    for i in range(12):
        if i in retried_rows:
            continue
        np.testing.assert_array_equal(
            df_clean.iloc[i].to_numpy(), df_retried.iloc[i].to_numpy()
        )
