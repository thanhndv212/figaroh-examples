"""TALOS torso-arm mocap study on the real runs (#172).

Slow: the k-fold plan, the selection and every model's held-out errors on
the shipped 2022-03-15 runs against ``golden/talos_mocap.json`` (rel 1e-4
plus abs 1e-3, the golden stores 3 decimals). The config's estimation method
is the model the k-fold plan selected, never one chosen on the confirmation
run.

The k-fold scores of ``full map_cv`` are not recomputed: on fold 4 its
preliminary free fit and its 25 inner fits run against their evaluation
caps (hours). Its golden score and the selection come from the full script;
every other number, its whole-run fit included, is recomputed here.

Regenerate the golden from examples/talos with::

    python mocap_heldout.py --json ../../tests/golden/talos_mocap.json
"""

import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
GOLDEN = json.loads((ROOT / "tests/golden/talos_mocap.json").read_text())

pytest.importorskip("pinocchio")

from examples.talos import mocap_heldout as mh  # noqa: E402

APPROX = dict(rel=1e-4, abs=1e-3)


def test_golden_is_the_frozen_protocol():
    assert GOLDEN["protocol"] == mh.PLAN.protocol
    assert GOLDEN["primary_metric"] == mh.PRIMARY_METRIC
    assert GOLDEN["kfold_metric"] == mh.KFOLD_METRIC
    assert set(GOLDEN["models"]) == set(mh.MODELS)


def test_config_method_is_the_kfold_selection():
    """The configured reference model is the golden's k-fold selection."""
    params = yaml.safe_load(mh.CONFIG.read_text())["tasks"]["calibration"][
        "parameters"]
    level, _, est = mh.MODELS[GOLDEN["selected"]]
    assert params["calibration_level"] == level
    assert params["estimation"]["method"] == est["method"]


#: recomputed in the k-fold plan (see the module docstring)
KFOLD_RECOMPUTED = [m for m in mh.MODELS if m != "full map_cv"]


@pytest.fixture(scope="module")
def got():
    kfold = mh.run_kfold(KFOLD_RECOMPUTED)
    _, scores = mh.select_model(kfold)
    g = GOLDEN["models"]["full map_cv"]
    kfold["full map_cv"] = {
        mh.KFOLD_METRIC: g["kfold_" + mh.KFOLD_METRIC],
        "fold_train_rmse_mm": [g["kfold_train_mean_mm"]],
    }
    scores["full map_cv"] = GOLDEN["selection_scores"]["full map_cv"]
    best = min(scores, key=scores.get)  # harness.select's rule
    return mh.snapshot(kfold, mh.run(), selection=(best, scores))


@pytest.mark.slow
def test_selection_matches_the_golden(got):
    assert got["selected"] == GOLDEN["selected"]
    for m in KFOLD_RECOMPUTED:
        assert got["selection_scores"][m] == pytest.approx(
            GOLDEN["selection_scores"][m], **APPROX), m


@pytest.mark.slow
@pytest.mark.parametrize("model", list(GOLDEN["models"]))
def test_matches_golden(got, model):
    g, r = GOLDEN["models"][model], got["models"][model]
    assert r["n_params"] == g["n_params"]
    assert r["n_joint_params"] == g["n_joint_params"]
    for k, v in g.items():
        if model == "full map_cv" and k.startswith("kfold_"):
            continue  # from the golden itself
        if isinstance(v, float):
            assert r[k] == pytest.approx(v, **APPROX), k
    for name, gs in g["sets"].items():
        e = r["sets"][name]
        for k in ("role", "n", "n_flipped", "dropped_rows", "flip_rows"):
            assert e[k] == gs[k], (name, k)
        for k, v in gs.items():
            if isinstance(v, float):
                assert e[k] == pytest.approx(v, **APPROX), (name, k)
        for s, gv in gs["strata"].items():
            assert e["strata"][s]["n"] == gv["n"]
            if gv["rmse"] is not None:
                assert e["strata"][s]["rmse"] == pytest.approx(gv["rmse"], **APPROX)


@pytest.mark.slow
def test_structural_fits_recover_from_the_baseline_frames():
    """Post hoc: the frozen structural fits end above the frames-only fit in
    training (a wrong basin from core's closed-form frame guess); started from
    the baseline's fitted frames they fall below it."""
    g = GOLDEN["models"]
    frames = mh.fitted_params(mh.fit_model(mh.BASELINE))
    for label in ("joint_offset", "full_params"):
        assert g[label]["train_pos_rmse_mm"] > g[mh.BASELINE]["train_pos_rmse_mm"]
        e = mh.training_errors(mh.warm_started(label, frames))
        assert e["pos_rmse_mm"] < g[mh.BASELINE]["train_pos_rmse_mm"] / 2, label
