"""TIAGo mocap across hardware (#100): the real and square stages and their golden.

``golden/tiago_mocap_hardware.json`` holds the fits, the re-registered LOO
matrix, the comparisons C1-C5 and the square-motion RMSE of the study. The fast
tests check the verdict rules, that the golden is consistent with the frozen
bands and the shipped files, and that the note quotes it. The slow tests rerun
both stages on the shipped data (the real stage in reverse fit order, which is
the determinism check).
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import mocap_hardware_study as study  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_mocap_hardware.json"
NOTE = ROOT / "docs/development/tiago-mocap-hardware.md"
SQUARE_PROTOCOL = ROOT / "examples/tiago/data/validation/square_motion/protocol.yaml"
MM_TOL = 0.05
MRAD_TOL = 0.2


@pytest.fixture(scope="module")
def golden():
    return json.loads(GOLDEN.read_text())


def test_classification_boundaries():
    assert study.classify(1.36) == "beyond"
    assert study.classify(1.35) == "borderline"
    assert study.classify(0.65) == "borderline"
    assert study.classify(0.64) == "within"


def test_verdict_aggregation():
    assert study.verdict_text(["within", "beyond"]).startswith("not explained")
    assert study.verdict_text(["within", "borderline"]) == "borderline"
    assert study.verdict_text(["within"]).startswith("consistent")


def test_golden_comparisons_use_the_frozen_bands(golden):
    bands = study.frozen_bands()
    comp = golden["comparisons"]
    for key, row in comp["C2"]["rows"].items():
        assert row["band_mm"] == pytest.approx(bands["transfer_band_mm"][key]["band"])
        ratio = abs(row["gap_mm"]) / row["band_mm"]
        assert row["ratio"] == pytest.approx(ratio, rel=1e-6)
        assert row["class"] == study.classify(ratio)
    for key, row in comp["C3"]["parameters"].items():
        pair, joint, mode = key.split("|")
        frozen = bands["parameter_band_rad"][f"{pair}|{mode}"][joint]
        assert row["band_mrad"] == pytest.approx(1e3 * frozen)
        ratio = abs(row["difference_mrad"]) / row["band_mrad"]
        assert row["class"] == study.classify(ratio)
    if not comp["C5"]["pass"]:
        for key in ("C2", "C3"):
            assert "not interpreted" in comp[key]["verdict"]


def test_golden_steps_are_post_hoc_and_consistent(golden):
    for key, v in golden["steps"].items():
        assert 3 <= v["split"] <= v["n"] - 3, key
        assert v["rmse_split_arm5_mm"] <= v["rmse_mm"] + 1e-6, key
    assert "verdict" not in golden["steps"]


def test_archive_asset_is_tiago_48():
    assert (
        '"asset_id": "tiago-48"'
        in (ROOT / "examples/tiago/mocap_hardware.py").read_text()
    )


def test_golden_square_zero_model_matches_shipped_registration(golden):
    text = SQUARE_PROTOCOL.read_text()
    assert "nominal_registration_rmse_shipped_mm: 4.69" in text
    zero = golden["square"]["rmse_mm"]["2021-12-16-15-04-54_zero"]["zero"]
    assert zero == pytest.approx(4.69, abs=0.02)


def test_golden_fixture_recovery_z_is_split_by_mode(golden):
    z = golden["fixture_recovery_z"]
    assert z["3d"]["n"] + z["6d"]["n"] == z["all"]["n"]
    assert z["6d"]["rms"] > z["3d"]["rms"]


def test_note_quotes_the_golden_verdicts_and_values(golden):
    lines = NOTE.read_text().splitlines()

    def row(key):
        hits = [ln for ln in lines if f"| `{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    comp = golden["comparisons"]
    for key in ("C2", "C3"):
        assert comp[key]["verdict"] in row(key), key
    assert comp["C5"]["verdict"] in row("C5")
    sq = golden["square"]["mean_rmse_mm"]
    for model, value in sq.items():
        assert f"{value:.2f}" in row(f"C4 {model}"), model
    z = golden["fixture_recovery_z"]
    for mode in ("3d", "6d"):
        assert f"{z[mode]['rms']:.2f}" in row(f"recovery z {mode}")
    for key, v in golden["steps"].items():
        line = row(f"step {key}")
        assert f"{v['split']} / {v['n']}" in line
        for x in (v["before_mm"], v["after_mm"], v["rmse_mm"]):
            assert f"{x:.2f}" in line, key
        assert f"{v['arm_5_jump_mrad']:+.1f}" in line, key


def test_validate_runs_the_real_stage():
    text = (ROOT / "validate.py").read_text()
    assert (
        '("mocap_hardware.py", 300, False, ["real", "--archive", "--no-plot"])' in text
    )


@pytest.mark.slow
def test_real_stage_matches_golden_in_reverse_order(golden):
    real = study.real_stage(reverse=True)
    for key, fit in golden["fits"].items():
        got = real["fits"][key]
        assert got["status"] == fit["status"], key
        assert got["absorbed"] == fit["absorbed"], key
        for name, v in fit["params"].items():
            assert got["params"][name] * 1e3 == pytest.approx(v * 1e3, abs=MRAD_TOL), (
                key,
                name,
            )
    for key, metrics in golden["matrix"].items():
        for m, v in metrics.items():
            assert real["matrix"][key][m] == pytest.approx(v, abs=MM_TOL), (key, m)
    for key in ("C2", "C3", "C5"):
        assert (
            real["comparisons"][key]["verdict"] == golden["comparisons"][key]["verdict"]
        )


@pytest.mark.slow
def test_square_stage_matches_golden(golden):
    sq = study.square_stage()
    for run, models in golden["square"]["rmse_mm"].items():
        for model in ("zero", "mocap set"):
            assert sq["rmse_mm"][run][model] == pytest.approx(
                models[model], abs=MM_TOL
            ), (run, model)
