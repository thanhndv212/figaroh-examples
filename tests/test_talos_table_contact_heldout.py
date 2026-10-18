"""TALOS table-contact held-out protocol (figaroh-examples#98).

Fast: the per-chain plan views carry exactly the master protocol's session
entries (file, sha256, role), the models are the agreed list, and selecting
on a confirmation role raises (there is none). Slow: the real held-out
numbers against ``golden/talos_table_contact.json`` (rel 1e-4 plus abs 1e-3,
the golden stores 3 decimals). The structural and cv_subset rows break
ties by platform (figaroh-plus#113), so their values are compared only when
the platform keeps the golden's parameter count.

Regenerate the golden from examples/talos_table_contact with::

    python heldout_protocol.py --json --no-plot > ../../tests/golden/talos_table_contact.json
"""

import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "talos_table_contact"
REAL = EXAMPLE / "data" / "real"
GOLDEN = json.loads((ROOT / "tests/golden/talos_table_contact.json").read_text())
for p in (str(ROOT), str(EXAMPLE)):
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("pinocchio")


@pytest.fixture(scope="module")
def hp():
    from _talos_import import talos_module

    return talos_module("heldout_protocol")


@pytest.mark.parametrize("side", ["left", "right"])
def test_plan_view_equals_master_entries(side):
    master = yaml.safe_load((REAL / "protocol.yaml").read_text())
    view = yaml.safe_load((REAL / f"protocol_{side}.yaml").read_text())
    want = [
        {"id": s["id"], "role": s["role"], "files": {s["file"]: s["sha256"]}}
        for s in master["sessions"]
        if s["file"].startswith(side)
    ]
    assert view["sessions"] == want
    assert [s["role"] for s in want] == ["training", "validation"]
    assert view["version"] == master["version"]
    assert view["heldout"]["split"] == "sessions"
    assert view["heldout"]["selection_role"] == "validation"


@pytest.mark.parametrize("side", ["left", "right"])
def test_plan_loads_with_one_training_session(side):
    from examples.calibration_harness import heldout as harness

    plan = harness.load_plan(REAL / f"protocol_{side}.yaml")  # verifies sha256
    fold = plan.folds[0]
    assert [s.role for s in fold.training] == ["training"]
    assert [s.role for s in fold.evaluated] == ["validation"]
    assert plan.spec.strata is not None and len(plan.spec.strata.joints) == 15


def test_models_are_the_agreed_list(hp):
    assert list(hp.MODELS) == [
        "table+contact only",
        "structural",
        "map",
        "map_cv",
        "excitation",
        "cv_subset",
    ]
    for label, (_, est) in hp.MODELS.items():
        if est and est["method"] != "structural":
            assert est["noise_std"] == hp.NOISE_STD, label  # given, not estimated
    assert hp.PRIMARY_METRIC == GOLDEN["primary_metric"] == "transfer_z_mm"


def test_selection_never_looks_at_confirmation(hp):
    from examples.calibration_harness import heldout as harness

    res = {"m": {0: {"s": {"role": "validation", hp.PRIMARY_METRIC: 1.0}}}}
    with pytest.raises(ValueError, match="confirmation"):
        harness.select(res, hp.PRIMARY_METRIC, role="confirmation")
    # and no session of either plan plays that role
    for side in ("left", "right"):
        view = yaml.safe_load((REAL / f"protocol_{side}.yaml").read_text())
        assert "confirmation" not in {s["role"] for s in view["sessions"]}


@pytest.fixture(scope="module")
def got(hp):
    robot = hp._load_robot()
    results = {s: hp.run_chain(s, robot) for s in hp.CHAINS}
    return hp.snapshot(results, hp.run_two_chain(robot))


@pytest.mark.slow
@pytest.mark.parametrize("side", ["left", "right"])
def test_matches_golden(hp, got, side):
    gold, now = GOLDEN["chains"][side]["models"], got["chains"][side]["models"]
    assert set(now) == set(gold)
    for model, g in gold.items():
        r = now[model]
        if model in hp.PLATFORM_DEPENDENT and r["n_params"] != g["n_params"]:
            continue  # the platform keeps another parameter set
        assert r["n_params"] == g["n_params"], model
        assert r["n_dx"] == g["n_dx"], model
        assert r["train_z_mm"] == pytest.approx(g["train_z_mm"], rel=1e-4, abs=1e-3)
        for name, gs in g["sets"].items():
            e = r["sets"][name]
            assert e["n"] == gs["n"]
            assert e["strata"] == gs["strata"]
            for k, v in gs.items():
                if k.startswith(("transfer_", "rereg_")):
                    assert e[k] == pytest.approx(v, rel=1e-4, abs=1e-3), (model, k)


@pytest.mark.slow
@pytest.mark.parametrize("side", ["left", "right"])
def test_joint_fit_beats_table_only_in_sample(got, side):
    m = got["chains"][side]["models"]
    assert m["structural"]["train_z_mm"] < m["table+contact only"]["train_z_mm"]


if __name__ == "__main__":
    # the golden is written by heldout_protocol.py --json
    raise SystemExit(
        "regenerate with: python heldout_protocol.py --json --no-plot "
        "> ../../tests/golden/talos_table_contact.json"
    )


# --------------------------------------------------------------------------
# Draw invariance by construction (the measured spread is in the slow test of
# test_talos_two_chain_draw_invariance.py)
# --------------------------------------------------------------------------
@pytest.mark.parametrize("label", ["map", "map_cv", "excitation", "cv_subset"])
def test_non_structural_methods_never_draw_random_configurations(
    hp, monkeypatch, label
):
    """Only the structural path draws the random configurations that
    ``random_seed`` seeds (``calculate_base_kinematics_regressor``).
    ``estimation.configure`` replaces it for every other method; the
    cross-validation folds take ``estimation.cv_seed``, not ``random_seed``.
    So initialising these models with the draw function disabled succeeds and
    ``random_seed`` cannot change the parameter set."""
    from _talos_import import talos_module
    from figaroh.calibration import base_calibration, estimation

    rr = talos_module("run_calibration_real_data")
    config, plan_path = hp.CHAINS["left"]
    train = hp.harness.load_plan(plan_path).folds[0].training[0].path

    def forbidden(*a, **k):
        raise AssertionError("the random-configuration draw was used")

    monkeypatch.setattr(
        base_calibration, "calculate_base_kinematics_regressor", forbidden
    )
    robot = hp._load_robot()
    est = hp.MODELS[label][1]
    names = []
    for seed in (0, 7):
        calib = rr._build_chain(robot, config, train, estimation=est, random_seed=seed)
        names.append(list(calib.calib_config["param_name"]))
    assert names[0] == names[1]
    assert "cv_seed" in estimation.settings(calib.calib_config)
