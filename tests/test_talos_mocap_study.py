"""TALOS torso-arm mocap study (#172): rules and the simulated fixture.

Fit-free rules of the frozen protocol (flip clusters, repair eligibility,
the k-fold plan) on the shipped files, and the simulated fixture
(examples/talos/mocap_truth.py): the flip repair recovers its transforms,
the k-fold plan selects a joint-level model, and the fits are judged
against a known truth. No test here fits real data.
"""

import dataclasses

import numpy as np
import pandas as pd
import pytest

from examples.talos import mocap_heldout as mh
from examples.talos import mocap_truth as mt

VALIDATION_FLIPS = [7, 8, 9, 10, 14, 15, 16, 21, 22, 28, 35, 36, 37, 38]


def test_frozen_rules():
    assert mh.PLAN.protocol == "talos-mocap-2022-03-15 v2"
    assert mh.KFOLD_PLAN.split == "kfold" and len(mh.KFOLD_PLAN.folds) == 5
    assert [r for r, _ in mh.SETS] == ["training", "validation", "confirmation"]
    assert (mh.LEVER, mh.FLIP_DEG, mh.MIN_CLUSTER) == (0.1, 45.0, 3)
    assert mh.BASELINE in mh.MODELS
    # every k-fold row is held out once, from the training file only
    held = sorted(r for f in mh.KFOLD_PLAN.folds for s in f.evaluated for r in s.rows)
    assert held == list(range(56))
    assert {s.path.name for f in mh.KFOLD_PLAN.folds for s in f.evaluated} == {mh.TRAINING}


def test_selection_never_looks_at_confirmation():
    with pytest.raises(ValueError):
        mh.harness.select({}, mh.PRIMARY_METRIC, role="confirmation")


def test_flip_rule_on_the_shipped_runs():
    """Fit-free: training has no flips; the validation run's 14 flipped rows
    form 3 repairable clusters; the confirmation run's single flipped row is
    too small a cluster and is dropped."""
    model = mh.robot().model
    labels = {}
    for role, name in mh.SETS:
        df = pd.read_csv(mh.MOCAP / name)
        labels[role] = mh.flip_clusters(model, df, mh.ARM[name])
    assert not labels["training"].any()
    assert np.flatnonzero(labels["validation"]).tolist() == VALIDATION_FLIPS
    assert labels["validation"].max() == 3
    assert (mh.repairable(labels["validation"]) == labels["validation"]).all()
    assert np.flatnonzero(labels["confirmation"]).tolist() == [0]
    assert mh.repairable(labels["confirmation"]).tolist()[0] == -1


def test_repairable_renumbers_and_drops():
    lab = np.array([0, 1, 1, 2, 3, 3, 3, 0, 1])
    assert mh.repairable(lab).tolist() == [0, 1, 1, -1, 2, 2, 2, 0, 1]


def test_training_loader_rejects_flipped_rows():
    path = mh.MOCAP / mh.SETS[1][1]  # the validation run (right arm)
    calib = mh.TalosMocapCalibration(mh.robot(), mh.CONFIG, path)
    calib.calib_config["end_frame"] = "arm_right_7_link"
    with pytest.raises(ValueError, match="flipped"):
        calib.load_data_set()


def test_flip_repair_recovers_the_synthetic_flips():
    truth = mt.draw_truth("full_params", np.random.default_rng(0))
    q = mt.postures(mt.RIGHT)
    Y = mt.right_truth(truth, q)
    joint = {n: v for n, v in truth.items() if not mh.is_frame(n)}
    F = mh.chain_poses(mh._nominal_calibrator(), joint, mt._q(q, "right"), "right",
                       "full_params")
    B0 = mh.from_xyzrpy([truth[n] for n in mh.BASE_TPL]) * mh.pin.exp6(
        np.r_[0.01, 0, 0, 0, 0.02, 0]
    )
    re = mh.register(F, Y, np.array(mt.right_labels()), B0)
    pos, ori = mh.pose_errors(re["pred"], Y)
    assert pos.max() < 1e-6 and ori.max() < 1e-6
    np.testing.assert_allclose(re["centroid_m"], mt.CENTROID, atol=1e-4)
    for fl, w in zip(re["flips"], mt.FLIPS):
        R = mh.pin.exp3(np.radians(fl["angle_deg"]) * np.array(fl["axis"]))
        assert mh._angle_deg(R, mh.pin.exp3(w)) < 0.1


def _simulated_training(tmp_path, seed=0, noise_mm=0.17):
    truth = mt.draw_truth("full_params", np.random.default_rng(seed))
    q = mt.postures(mh.TRAINING)
    clean = mt.to_array(mt.left_poses(truth, "full_params", q),
                        mt._reference(q, "left"))
    noisy = clean + np.random.default_rng(10_000 + seed).normal(
        0, noise_mm * 1e-3, clean.shape
    )
    path = tmp_path / "sim_train.csv"
    mt.to_dataset(noisy, q).to_csv(path, index=False)
    return path


@pytest.mark.slow
def test_kfold_selects_a_joint_level_model_on_simulated_data(tmp_path, monkeypatch):
    path = _simulated_training(tmp_path)

    def swap(s):
        return dataclasses.replace(s, path=path)

    plan = dataclasses.replace(mh.KFOLD_PLAN, folds=tuple(
        dataclasses.replace(f, training=tuple(map(swap, f.training)),
                            evaluated=tuple(map(swap, f.evaluated)))
        for f in mh.KFOLD_PLAN.folds
    ))
    monkeypatch.setattr(mh, "KFOLD_PLAN", plan)
    monkeypatch.setitem(mh.ARM, path.name, "left")
    k = mh.run_kfold([mh.BASELINE, "full_params"])
    best, scores = mh.select_model(k)
    assert best == "full_params"
    assert scores["full_params"] < 0.5 < 3 < scores[mh.BASELINE]
    # C1's overfitting ratio holds on the fixture
    ratio = k["full_params"][mh.KFOLD_METRIC] / np.mean(
        k["full_params"]["fold_train_rmse_mm"]
    )
    assert ratio < 1.5


@pytest.mark.slow
@pytest.mark.parametrize(
    "truth, model, left_max, right_max",
    [
        ("full_params", "full map", 0.6, 4.5),
        ("full_params", mh.BASELINE, None, None),
        ("right_exact", "full map", 0.6, 0.5),
    ],
)
def test_fixture_case(tmp_path, truth, model, left_max, right_max):
    """Seed 0 at the training run's noise (0.17 mm), against the truth."""
    r = mt.sim.run_case(mt.study(), truth, truth, 0, 0.17, tmp_path,
                        estimation={"label": model}, label=model)
    left, right = r["strata_rmse_mm"]["left"], r["strata_rmse_mm"]["right"]
    if model == mh.BASELINE:
        assert left > 5  # nominal joints: the errors stay in the prediction
        return
    assert left < left_max and right < right_max
    z = np.array(list(r["z_scores"].values()))
    assert len(z) > 0 and np.sqrt(np.mean(z**2)) < 1.5
