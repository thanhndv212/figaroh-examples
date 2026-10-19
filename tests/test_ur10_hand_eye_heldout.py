"""UR10 hand-eye k-fold held-out comparison (#99).

Fast checks of the wiring: model list, the registration baseline, the
selection guard and one registration fold. The full table (all models, all
folds) is slow and pinned in tests/golden/ur10_hand_eye.json.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.ur10 import hand_eye_heldout as heh  # noqa: E402

GOLDEN = ROOT / "tests" / "golden" / "ur10_hand_eye.json"


def test_models_and_label():
    assert list(heh.MODELS) == [
        "registration only",
        "joint_offset",
        "full structural",
        "full map",
        "full map_cv",
        "full excitation",
        "full cv_subset",
    ]
    assert heh.PLAN.split == "kfold"
    assert "k-fold within session" in heh.PLAN.label
    assert len(heh.PLAN.folds) == 5
    assert heh.PRIMARY_METRIC != heh.SECONDARY_METRIC


def test_selection_never_looks_at_confirmation():
    res = {"m": {0: {"s": {"rmse_pos_mm": 1.0, "role": "confirmation"}}}}
    with pytest.raises(ValueError, match="confirmation"):
        harness.select(res, heh.PRIMARY_METRIC, role="confirmation")


def test_pose_errors_are_zero_for_identical_poses_and_geodesic():
    pose = np.array([[0.1, 0.2, 0.3, 3.1, 0.2, -3.1]])  # near +-pi: no wrap issue
    pos, rot = heh.pose_errors(pose, pose)
    assert pos[0] == pytest.approx(0, abs=1e-9) and rot[0] == pytest.approx(0, abs=1e-6)
    shifted = pose + np.array([[0.003, 0, 0, 0, 0, 0]])
    assert heh.pose_errors(pose, shifted)[0][0] == pytest.approx(3.0)


def test_registration_baseline_fits_one_fold():
    calib = heh.fit_fold("registration only", heh.PLAN.folds[0])
    names = list(calib.calib_config["param_name"])
    assert len(names) == 12
    assert all(heh.is_frame(n) for n in names)
    held = heh.PLAN.folds[0].evaluated[0]
    m = heh.evaluate(calib, held)
    assert m["n"] == len(held.rows)
    assert m["n_joint_params"] == 0
    assert 0 < m["rmse_pos_mm"] < 50 and 0 < m["rmse_rot_deg"] < 10


@pytest.mark.slow
def test_full_table_matches_golden():
    golden = json.loads(GOLDEN.read_text())["kfold"]
    table = heh.summarize(heh.run())
    assert list(table) == list(golden["models"])
    for model, row in golden["models"].items():
        got = table[model]
        assert got["n_params"] == row["n_params"]
        assert got["rmse_pos_mm"] == pytest.approx(row["rmse_pos_mm"], abs=2e-3)
        assert got["rmse_rot_deg"] == pytest.approx(row["rmse_rot_deg"], abs=2e-3)
    assert heh.select_model(heh.run())[0] == golden["selected"]
