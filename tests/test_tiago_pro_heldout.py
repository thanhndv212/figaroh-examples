"""TIAGo Pro mocap held-out protocol (#97).

Checks the protocol's wiring cheaply (roles, models, the k-fold diagnostic
plan, the selection guard) and pins its results against
``golden/tiago_pro_mocap.json`` (rel 1e-4 plus abs 1e-3, the golden stores 3 decimals: solver round-off differs by about
1e-6 across platforms, a changed model or protocol moves the numbers by
O(1)). The structural full_params and the cv_subset selection break ties by
platform (figaroh-plus#113), so their values are compared only when the
platform keeps the golden's parameter count.

Regenerate the golden from examples/tiago_pro with::

    python ../../tests/test_tiago_pro_heldout.py > ../../tests/golden/tiago_pro_mocap.json
"""

import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRO = ROOT / "examples" / "tiago_pro"
GOLDEN = json.loads((ROOT / "tests/golden/tiago_pro_mocap.json").read_text())
sys.path.insert(0, str(ROOT))

#: models whose kept parameter set depends on the platform
PLATFORM_DEPENDENT = ("full_params", "full cv_subset")


def snapshot() -> dict:
    """The golden's content, computed from the current code."""
    from examples.tiago_pro import calibration_truth as ct
    from examples.tiago_pro import data_audit as da
    from examples.tiago_pro import heldout_protocol as hp

    res = hp.run()
    models = {}
    for m, r in res.items():
        if m.startswith("_"):
            continue
        models[m] = {
            "n_params": r["n_params"],
            "n_joint_params": r["n_joint_params"],
            "train_pos_rmse_mm": round(r["train_pos_rmse_mm"], 3),
            "train_ori_rmse_deg": round(r["train_ori_rmse_deg"], 3),
            "sets": {
                n: {
                    "rmse_rereg": round(e["rmse_rereg"], 3),
                    "rmse_transfer": round(e["rmse_transfer"], 3),
                    "n": e["n"],
                    "strata": {k: v[0] for k, v in e["strata"].items()},
                }
                for n, e in r["sets"].items()
            },
        }
    df = ct.run(seeds=(0,))
    keep = (
        "fit",
        "noise_mm",
        "n_params",
        "train_rms_mm",
        "heldout_rmse_mm",
        "heldout_max_mm",
    )
    return {
        "_about": GOLDEN["_about"],
        "primary_metric": hp.PRIMARY_METRIC,
        "selected": hp.select_model(res)[0],
        "models": models,
        "noise": {
            "sigma_mm": round(da.reregistration_noise()["sigma_mm"], 3),
            "grid": list(da.noise_grid()),
        },
        "simulation_seed0": [
            {
                k: (round(v, 3) if isinstance(v, float) else v)
                for k, v in r.items()
                if k in keep
            }
            for r in df.to_dict("records")
        ],
    }


@pytest.fixture(scope="module")
def hp():
    mp = pytest.MonkeyPatch()
    mp.chdir(PRO)  # the config uses relative paths
    from examples.tiago_pro import heldout_protocol

    yield heldout_protocol
    mp.undo()


def test_roles_match_config_and_golden(hp):
    assert [r for r, _ in hp.SETS] == ["training", "validation", "confirmation"]
    assert hp.PLAN.spec.selection_role == "validation"
    assert hp.PRIMARY_METRIC == GOLDEN["primary_metric"] == "rmse_rereg"
    for m, g in GOLDEN["models"].items():
        assert set(g["sets"]) == {n for _, n in hp.SETS[1:]}, m
    assert list(GOLDEN["models"]) == list(hp.MODELS)


def test_models_are_the_frozen_list(hp):
    assert list(hp.MODELS) == [
        "registration only",
        "full_params",
        "full map",
        "full map_cv",
        "full cv_subset",
    ]
    # joint_offset is absent: arm_right_6_joint is RevoluteUnaligned
    # (figaroh-plus#205), core's offset parameters cannot express it
    assert all(level == "full_params" for level, _, _ in hp.MODELS.values())


def test_selection_never_looks_at_confirmation(hp):
    from examples.calibration_harness import heldout as harness

    res = {"m": {0: {"s": {"role": "confirmation", hp.PRIMARY_METRIC: 1.0}}}}
    with pytest.raises(ValueError, match="confirmation"):
        harness.select(res, hp.PRIMARY_METRIC, role="confirmation")


def test_kfold_diagnostic_plan_is_fixed(hp):
    from examples.calibration_harness import heldout as harness
    from examples.tiago_pro import diagnostics

    plan = harness.load_plan(diagnostics.KFOLD_PROTOCOL)
    assert plan.split == "kfold"
    assert "weaker than held-out sessions" in plan.label
    held = [f.evaluated[0].rows for f in plan.folds]
    assert len(plan.folds) == 5
    assert sorted(i for h in held for i in h) == list(range(48))
    for f in plan.folds:
        assert not set(f.training[0].rows) & set(f.evaluated[0].rows)


@pytest.fixture(scope="module")
def results(hp):
    return hp.run()


@pytest.mark.slow
@pytest.mark.parametrize("model", list(GOLDEN["models"]))
def test_matches_golden(results, model):
    g, got = GOLDEN["models"][model], results[model]
    if model in PLATFORM_DEPENDENT and got["n_params"] != g["n_params"]:
        pytest.skip(
            f"platform keeps {got['n_params']} parameters, golden {g['n_params']}"
        )
    assert got["n_params"] == g["n_params"]
    assert got["n_joint_params"] == g["n_joint_params"]
    assert got["train_pos_rmse_mm"] == pytest.approx(
        g["train_pos_rmse_mm"], rel=1e-4, abs=1e-3
    )
    assert got["train_ori_rmse_deg"] == pytest.approx(
        g["train_ori_rmse_deg"], rel=1e-4, abs=1e-3
    )
    for name, gs in g["sets"].items():
        e = got["sets"][name]
        assert e["n"] == gs["n"]
        assert e["rmse_rereg"] == pytest.approx(gs["rmse_rereg"], rel=1e-4, abs=1e-3)
        assert e["rmse_transfer"] == pytest.approx(
            gs["rmse_transfer"], rel=1e-4, abs=1e-3
        )
        assert {k: v[0] for k, v in e["strata"].items()} == gs["strata"]


@pytest.mark.slow
def test_selection_and_ordering(hp, results):
    best, scores = hp.select_model(results)
    assert best == GOLDEN["selected"] == "full map_cv"
    # registration alone is far worse than any fitted model; the fitted
    # models differ by less than a millimetre (not a meaningful margin)
    fitted = [v for m, v in scores.items() if m != "registration only"]
    assert scores["registration only"] > max(fitted) + 5
    assert max(fitted) - min(fitted) < 1.0


@pytest.mark.slow
def test_reregistering_helps_on_every_model(results):
    for model, r in results.items():
        if model.startswith("_"):
            continue
        for e in r["sets"].values():
            assert e["rmse_rereg"] < e["rmse_transfer"], model


@pytest.mark.slow
def test_kfold_overfitting_check_for_the_selected_model():
    """Diagnostic: map_cv generalises within the training session (<= 1.5x)."""
    os.chdir(PRO)
    from examples.tiago_pro import diagnostics

    res = diagnostics.kfold(["full map_cv"])
    _, summ = diagnostics.kfold_table(res, {"full map_cv": 6.686})
    assert summ["full map_cv"]["ratio"] <= diagnostics.OVERFIT_RATIO


if __name__ == "__main__":
    os.chdir(PRO)
    print(json.dumps(snapshot(), indent=1))
