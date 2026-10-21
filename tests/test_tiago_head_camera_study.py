"""TIAGo head camera (#101): the study stages and their golden.

``golden/tiago_head_camera.json`` holds the training fit, the baselines, the
confirmation comparisons C0 to C2 with the not-usable rules, and the
diagnostics. The fast tests check the verdict rules, that the golden is
consistent with the frozen bands and that the note quotes it. The slow test
reruns the whole study on the shipped data.
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import head_camera_study as study  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_head_camera.json"
NOTE = ROOT / "docs/development/tiago-head-camera.md"
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
    assert "beyond" in study.verdict_text(["within", "beyond"])
    assert study.verdict_text(["within", "borderline"]) == "borderline"
    assert "consistent" in study.verdict_text(["within"])


def test_golden_c1_uses_the_frozen_bands(golden):
    bands = study.frozen()
    c1 = golden["confirmation"]["c1"]
    for rid, run in c1["runs"].items():
        for name, row in run["rows"].items():
            assert row["band_mrad"] == pytest.approx(c1["band_mrad"][name])
            ratio = abs(row["gap_mrad"]) / row["band_mrad"]
            assert row["ratio"] == pytest.approx(ratio, rel=1e-6)
            assert row["class"] == study.classify(ratio), (rid, name)
    assert bands is not None


def test_golden_c2_uses_the_frozen_bands(golden):
    c2 = golden["confirmation"]["c2"]
    assert len(c2["pairs"]) == 6
    for pair in c2["pairs"]:
        for direction in ("cam_to_mc", "mc_to_cam"):
            for unit in ("pos_mm", "rot_mrad"):
                g = pair[direction]
                ratio = abs(g[unit]) / g[unit + "_band"]
                assert g[unit + "_ratio"] == pytest.approx(ratio, rel=1e-6)
                assert g[unit + "_class"] == study.classify(ratio)
    for key, v in c2["summary"].items():
        assert v["beyond"] + v["borderline"] + v["within"] == 6, key


def test_golden_rules_and_verdict_are_consistent(golden):
    conf = golden["confirmation"]
    rules = conf["rules"]
    usable = (
        rules["rule1_c0"]
        and rules["rule2_fixture_se"]
        and all(rules["rule3_min_postures"].values())
    )
    assert rules["usable"] == usable
    v = conf["verdict"]
    assert v["final"] == (v["combined"] if usable else "not answerable with this data")
    assert v["combined"] == study.verdict_text(
        conf["c1"]["items"] + conf["c2"]["items"]
    )
    for rid, n in conf["runs"].items():
        assert rules["rule3_min_postures"][rid] == (n["n_paired"] >= 6)


def test_golden_c0_rule_1(golden):
    c0 = golden["confirmation"]["c0"]
    assert c0["rule1_pass"] == (c0["worst_residual_ratio"] <= 3 and c0["frames_ok"])
    assert c0["worst_residual_ratio"] == pytest.approx(
        max(max(v) for v in c0["residual_over_floor"].values()), rel=1e-6
    )


def test_golden_pal_sign_is_decided_on_training_only(golden):
    s = golden["baselines"]["pal_sign"]
    rows = s["rows"]
    best = min(rows, key=lambda k: rows[k]["B1"]["pos_mm"])
    assert float(best) == s["sign"]
    assert "training" in s["rule"]


def test_golden_training_fit_is_full_rank(golden):
    for tag in ("fit6d", "fit3d"):
        f = golden["training"][tag]
        assert f["rank"] == f["n_params"]
    assert golden["training"]["fit6d"]["n_params"] == 19  # 18 in G plus k


def test_note_quotes_the_golden(golden):
    lines = NOTE.read_text().splitlines()

    def row(key):
        hits = [ln for ln in lines if f"| `{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    train = golden["training"]
    for tag in ("fit6d", "fit3d"):
        f = train[tag]
        line = row(f"train {tag}")
        for x in (f["rmse_pos_mm"], f["rmse_rot_mrad"]):
            assert f"{x:.2f}" in line or f"{x:.1f}" in line
        for name, v, se in zip(
            study.ARM_NAMES, f["offsets_mrad"], f["offsets_se_mrad"]
        ):
            line = row(f"train {tag} d_{name}")
            assert f"{v:+.2f}" in line and f"{se:.2f}" in line
    assert f"{golden['baselines']['pal_sign']['sign']:+.0f}" in row("PAL sign")
    for m, v in golden["baselines"]["validation"]["scores"].items():
        line = row(f"val {m}")
        assert f"{v['pos_mm']:.2f}" in line and f"{v['rot_mrad']:.1f}" in line
    conf = golden["confirmation"]
    c0 = conf["c0"]
    assert f"{c0['worst_residual_ratio']:.2f}" in row("C0 rule 1")
    for rid, v in c0["runs"].items():
        line = row(f"C0 {rid}")
        for x in v["rms_pos_mm"]:
            assert f"{x:.2f}" in line
    for name, band in conf["c1"]["band_mrad"].items():
        line = row(f"C1 {name}")
        assert f"{band:.1f}" in line
        for run in conf["c1"]["runs"].values():
            assert f"{run['rows'][name]['gap_mrad']:+.1f}" in line
    for key, v in conf["c2"]["summary"].items():
        line = row("C2 " + key.replace("|", " "))
        for x in (v["band"], v["mean_gap"], v["max_abs_gap"]):
            assert f"{abs(x):.2f}" in line
    for k, ok in conf["rules"]["rule3_min_postures"].items():
        assert str(ok).lower() in row(f"rule 3 {k}").lower()
    assert conf["verdict"]["final"] in row("final")
    for sid, v in golden["diagnostics"].items():
        line = row(f"diag {sid}")
        assert f"{v['scores']['trained']['pos_mm']:.2f}" in line


def test_validate_runs_the_study():
    text = (ROOT / "validate.py").read_text()
    assert '("head_camera_study.py", 300, False, ["--archive", "--no-plot"])' in text


@pytest.mark.slow
def test_study_matches_golden(golden):
    res = study.golden(study.study(verbose=False))
    t, g = res["training"], golden["training"]
    for tag in ("fit6d", "fit3d"):
        assert t[tag]["status"] == g[tag]["status"]
        for a, b in zip(t[tag]["offsets_mrad"], g[tag]["offsets_mrad"]):
            assert a == pytest.approx(b, abs=MRAD_TOL)
        assert t[tag]["rmse_pos_mm"] == pytest.approx(g[tag]["rmse_pos_mm"], abs=MM_TOL)
    for m, v in golden["baselines"]["validation"]["scores"].items():
        got = res["baselines"]["validation"]["scores"][m]
        assert got["pos_mm"] == pytest.approx(v["pos_mm"], abs=MM_TOL)
        assert got["rot_mrad"] == pytest.approx(v["rot_mrad"], abs=MRAD_TOL)
    assert (
        res["baselines"]["pal_sign"]["sign"] == golden["baselines"]["pal_sign"]["sign"]
    )
    assert res["confirmation"]["rules"] == golden["confirmation"]["rules"]
    assert res["confirmation"]["verdict"] == golden["confirmation"]["verdict"]
    for key, v in golden["confirmation"]["c2"]["summary"].items():
        got = res["confirmation"]["c2"]["summary"][key]
        assert got["mean_gap"] == pytest.approx(v["mean_gap"], abs=MM_TOL)
        assert (got["beyond"], got["borderline"], got["within"]) == (
            v["beyond"],
            v["borderline"],
            v["within"],
        )
