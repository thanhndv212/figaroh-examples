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
        "joint_offset",
        "full_params",
        "full map",
        "full map_cv",
        "full cv_subset",
    ]
    # joint_offset needs figaroh-plus#205 (arm_right_6_joint is
    # RevoluteUnaligned); every other model is built from full_params
    assert hp.MODELS["joint_offset"][0] == "joint_offset"
    assert all(
        level == "full_params"
        for m, (level, _, _) in hp.MODELS.items()
        if m != "joint_offset"
    )


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
    # a platform-dependent model that keeps another parameter set than the
    # golden's can win by a fraction of a millimetre (pinocchio 3.7 CI keeps
    # another cv_subset set); the golden's choice holds wherever it does not
    deviating = {
        m
        for m in PLATFORM_DEPENDENT
        if results[m]["n_params"] != GOLDEN["models"][m]["n_params"]
    }
    assert GOLDEN["selected"] == "full map_cv"
    assert best == GOLDEN["selected"] or best in deviating
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


def test_encoder_side_from_joint_data():
    """Diagnostic: July rows equal their commanded postures (motor side);
    08-05 rows differ by an offset plus a gravity deflection (link side)."""
    os.chdir(PRO)
    from examples.tiago_pro import diagnostics

    res = diagnostics.encoder_fit()
    s = res["sessions"]
    assert s["calibration_samples_20260805_1246.csv"]["on_plan"] == 0
    assert s["calibration_samples_20260701_0913.csv"]["on_plan"] == 4
    assert s["calibration_samples_20260702_0756.csv"]["on_plan"] == 8
    for name in (
        "calibration_samples_20260701_0913.csv",
        "calibration_samples_20260702_0756.csv",
    ):
        assert s[name]["on_plan_max_gap"] < 2e-4
    j = res["joints"]
    for n in (
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_6_joint",
    ):
        assert j[n]["corr"] > 0.99, n
    for n in (
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
    ):
        assert 1400 < 1 / j[n]["s"] < 1700, n  # stiffness, Nm/rad
    assert max(v["resid_rms"] for v in j.values()) < 6e-4
    assert j["arm_right_7_joint"]["s"] == 0.0  # no gravity load: offset only


@pytest.mark.slow
def test_encoder_side_correction_closes_most_of_the_gap():
    """Diagnostic: mapping July to the link side cuts map_cv's held-out error."""
    os.chdir(PRO)
    from examples.tiago_pro import diagnostics

    res = diagnostics.encoder_side(["full map_cv"])["mapped"]["full map_cv"]
    for e in res.values():
        assert e["both"] < 0.75 * e["raw"]
        assert e["both"] < min(e["offset"], e["elastic"])
    assert res["calibration_samples_20260702_0756.csv"]["both"] < 8.0


@pytest.mark.slow
def test_core_elasticity_identifies_the_encoder_compliance():
    """Diagnostic: core's k_* fitted from the mocap on motor-side data closes
    most of the gap; motor-side minus link-side k is the joint-data fit."""
    os.chdir(PRO)
    from examples.tiago_pro import diagnostics

    runs = diagnostics.elastic()["runs"]
    motor, link = runs["elastic, 08-05 motor"], runs["elastic, 08-05 link"]
    rigid = runs["rigid, 08-05 link"]
    assert motor["train_pos_rmse_mm"] < rigid["train_pos_rmse_mm"] - 1.0
    for f, e in motor["heldout"].items():
        assert e < 0.7 * rigid["heldout"][f], f
    assert motor["heldout"]["calibration_samples_20260702_0756.csv"] < 6.0
    s = diagnostics.encoder_fit()["joints"]
    for i in range(1, 6):
        n = f"arm_right_{i}_joint"
        diff = motor["k"][n] - link["k"][n]  # core's sign: link = motor + k*tau
        assert abs(diff + s[n]["s"]) < 0.1 * s[n]["s"], n
    july = runs["elastic, 07-01 motor"]
    assert july["heldout"]["calibration_samples_20260702_0756.csv"] < 0.6 * (
        runs["rigid, 07-01 motor"]["heldout"]["calibration_samples_20260702_0756.csv"]
    )


if __name__ == "__main__":
    os.chdir(PRO)
    print(json.dumps(snapshot(), indent=1))
