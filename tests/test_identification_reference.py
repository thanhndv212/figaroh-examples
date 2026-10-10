"""Dynamic-identification reference workflow (D7, #23).

``examples/identification_reference.py`` fits, selects, verifies, exports
into a lumped nominal URDF, reloads it and archives the run. The UR10 case is
judged against a known truth, the TIAGo cases against the shipped recordings.
The command-line runs are sequential and use a temporary archive root.
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
UR10 = ROOT / "examples" / "ur10"
TIAGO = ROOT / "examples" / "tiago"
sys.path.insert(0, str(ROOT))

from examples import identification_reference as ref  # noqa: E402

ARTIFACTS = [
    "report.html",
    "verdict.json",
    "provenance.json",
    "reproduction.json",
    "nominal_lumped.urdf",
    "export_check.json",
    "reference.json",
]


def _run(cwd, args, root):
    env = dict(
        os.environ,
        MPLBACKEND="Agg",
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
        VECLIB_MAXIMUM_THREADS="1",
    )
    proc = subprocess.run(
        [sys.executable, "identification_reference.py", *args, "--root", str(root)],
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )
    dirs = sorted(Path(root).glob("*/identification/*"))
    return proc, dirs


def _load(path, name):
    return json.loads((path / name).read_text())


@pytest.mark.parametrize(
    "urdf", [UR10 / "urdf/ur10_robot.urdf", TIAGO / "urdf/tiago_48_hey5.urdf"]
)
def test_lumped_nominal_keeps_every_body_inertia(urdf, tmp_path):
    out = ref.lumped_nominal_urdf(urdf, tmp_path / "lumped.urdf")
    nominal, lumped = pin.buildModelFromUrdf(str(urdf)), pin.buildModelFromUrdf(
        str(out)
    )
    assert (lumped.nq, lumped.nv) == (nominal.nq, nominal.nv)
    for j in range(1, nominal.njoints):
        np.testing.assert_allclose(
            lumped.inertias[j].toDynamicParameters(),
            nominal.inertias[j].toDynamicParameters(),
            rtol=0,
            atol=1e-12,
        )
    assert ref.effort_parity(nominal, lumped, n=20) < 1e-9


def test_parity_of_identical_models_is_zero_and_sees_a_difference():
    model = pin.buildModelFromUrdf(str(UR10 / "urdf/ur10_robot.urdf"))
    assert ref.effort_parity(model, pin.Model(model)) == 0.0
    other = pin.Model(model)
    inertia = other.inertias[2]
    other.inertias[2] = pin.Inertia(1.01 * inertia.mass, inertia.lever, inertia.inertia)
    assert ref.effort_parity(model, other, n=20) > 1e-3


def test_check_expectations():
    expected = {"a": "x", "b": True, "c": ("max", 1e-6), "d": []}
    good = {"a": "x", "b": True, "c": 1e-9, "d": []}
    assert ref.check_expectations(good, expected) == []
    bad = dict(good, a="y", c=None, d=["export"])
    assert len(ref.check_expectations(bad, expected)) == 3
    assert ref.check_expectations(dict(good, c=2e-6), expected)


def test_ur10_acceptance_profile_is_the_protocol_noise_floor():
    from examples.ur10 import identification_truth as truth

    profile = json.loads((UR10 / "config/truth_reference_acceptance.json").read_text())
    sigma = truth.protocol()["noise"]["effort_sigma_nm"]["low"]
    assert list(profile) == [f"validation_rmse:{j}" for j in truth.JOINTS]
    for joint, value in sigma.items():
        spec = profile[f"validation_rmse:{joint}"]
        assert spec["comparison"] == "max"
        assert spec["threshold"] == pytest.approx(1.25 * value + 0.01, abs=1e-6)


def test_overlays_only_choose_the_stage():
    for path, stage in (
        (UR10 / "config/ur10_truth_reference.yaml", "physical_fit"),
        (TIAGO / "config/tiago_reference_physical_fit.yaml", "physical_fit"),
        (TIAGO / "config/tiago_reference_reject.yaml", "reconstruction"),
    ):
        overlay = yaml.safe_load(path.read_text())
        assert overlay["tasks"] == {"identification": {"select_stage": stage}}
        assert overlay["extends"].endswith("_unified_config.yaml")


@pytest.fixture(scope="module")
def ur10_truth(tmp_path_factory):
    root = tmp_path_factory.mktemp("ur10")
    proc, dirs = _run(UR10, ["--case", "truth"], root)
    assert len(dirs) == 1, proc.stdout[-2000:] + proc.stderr[-2000:]
    return dirs[0], proc


def test_ur10_truth_case_passes(ur10_truth):
    path, proc = ur10_truth
    assert proc.returncode == 0, proc.stdout[-2000:]
    for name in (*ARTIFACTS, "identified.urdf", "parameters.csv"):
        assert (path / name).is_file(), name
    report = _load(path, "reference.json")
    assert report["claim"] == "offline logs; no hardware deployment"
    assert report["physical"]["solver_status"] == "optimal"
    assert report["physical"]["status"] == "accepted"
    assert report["training_disjoint_from_heldout"]
    assert all(v["ok"] for v in report["physical"]["links"].values())
    assert max(report["heldout_vs_truth_rmse"].values()) <= 0.1
    assert _load(path, "export_check.json")["parity"] <= ref.PARITY_TOL
    verdict = _load(path, "verdict.json")
    assert verdict["selected_stage"] == "physical_fit"
    assert verdict["scope"] == "prediction"


def test_ur10_truth_report_lists_every_joint_with_unit(ur10_truth):
    report = _load(ur10_truth[0], "reference.json")
    assert len(report["per_joint"]) == 6
    for row in report["per_joint"]:
        assert row["unit"] == "N·m" and row["fitted"]
        assert row["train_rmse"] > 0 and row["heldout_rmse_selected"] > 0


def test_a_mismatch_with_the_expectations_exits_nonzero(tmp_path, monkeypatch):
    from examples.ur10 import identification_reference as ur10_ref

    monkeypatch.chdir(UR10)
    args = argparse.Namespace(
        root=str(tmp_path),
        asset_id=None,
        operator=None,
        strict_revisions=False,
        noise="low",
    )
    case = ur10_ref.case_truth(args)
    case.expected = dict(case.expected, selected_stage="reconstruction")
    assert ref.run_case(case, args) == 1


@pytest.fixture(scope="module")
def tiago_runs(tmp_path_factory):
    """Both cases, one after the other (parallel TIAGo runs can hang)."""
    out = {}
    for case in ("physical-fit", "reject"):
        root = tmp_path_factory.mktemp(case)
        proc, dirs = _run(TIAGO, ["--case", case], root)
        assert len(dirs) == 1, proc.stdout[-2000:] + proc.stderr[-2000:]
        out[case] = (dirs[0], proc)
    return out


def test_tiago_physical_fit_case_passes(tiago_runs):
    path, proc = tiago_runs["physical-fit"]
    assert proc.returncode == 0, proc.stdout[-2000:]
    for name in (*ARTIFACTS, "identified.urdf", "parameters.csv"):
        assert (path / name).is_file(), name
    report = _load(path, "reference.json")
    assert report["physical"]["solver_status"] == "optimal"
    assert report["physical"]["status"] == "accepted"
    assert report["scope"] == "execution"
    assert report["training_disjoint_from_heldout"]
    extra = report["heldout_extra"]
    assert extra["role"] == "validation" and extra["reported_not_gated"]
    assert extra["pooled_arm2_4"] > 0
    assert report["heldout_overall_rmse"]["gated"] is False
    assert _load(path, "export_check.json")["parity"] <= ref.PARITY_TOL
    fitted = {r["joint"] for r in report["per_joint"] if r["fitted"]}
    assert fitted == {f"arm_{i}_joint" for i in range(1, 5)}
    assert _load(path, "verdict.json")["selected_stage"] == "physical_fit"


def test_tiago_reject_case_exports_nothing(tiago_runs):
    path, proc = tiago_runs["reject"]
    assert proc.returncode == 0, proc.stdout[-2000:]
    assert not (path / "parameters.csv").exists()
    assert not (path / "identified.urdf").exists()
    assert (path / "fit_parameters.csv").is_file()
    verdict = _load(path, "verdict.json")
    assert verdict["selected_stage"] == "none"
    assert verdict["status"] != "pass"
    export = _load(path, "export_check.json")
    assert export["exported"] is False and "rejected" in export["error"]
    report = _load(path, "reference.json")
    assert report["physical"]["status"] == "rejected"
    assert "infeasible links" in report["physical"]["reason"]
    from examples.run_record import audit, missing

    assert set(missing(audit(path))) - {"revisions"} == {"export"}
