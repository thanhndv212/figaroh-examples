"""TIAGo backlash surface (#71), part 2: the study stages and their golden.

``golden/tiago_backlash.json`` holds the training fit, the baselines B0 to B2
and M*, the verdicts V1 to V3 with the rules N1 to N6, and the joint-inspection
prediction checks. The tests check the verdict rules, that the golden agrees
with the frozen bands, that the note quotes it, and that the study reruns to
the same numbers (about 10 s).
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import backlash_study as study  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_backlash.json"
NOTE = ROOT / "docs/development/tiago-backlash.md"
TOL = 0.05


@pytest.fixture(scope="module")
def golden():
    return json.loads(GOLDEN.read_text())


def test_label_is_validation_level(golden):
    assert golden["label"] == "validation-level"
    assert golden["verdicts"]["label"] == "validation-level"


def test_classify_v1_boundaries():
    assert study.classify_v1(-1.01, 1.0)["class"] == "improves"
    assert study.classify_v1(-1.0, 1.0)["class"] == "no change"
    assert study.classify_v1(1.0, 1.0)["class"] == "no change"
    assert study.classify_v1(1.01, 1.0)["class"] == "worse"
    assert study.classify_v1(-1.2, 1.0)["borderline"]
    assert not study.classify_v1(-2.0, 1.0)["borderline"]


def test_classify_v2_and_v3():
    assert study.classify_v2(0.5, 0.5) == "W stable across sessions"
    assert study.classify_v2(-0.51, 0.5) == "W differs"
    none = "no load dependence detected"
    assert study.classify_v3(False, []) == none
    row = {"same_sign": True, "z_val": 2.5}
    assert study.classify_v3(True, [row]) == "load-dependent"
    assert study.classify_v3(True, [dict(row, same_sign=False)]) == none
    assert study.classify_v3(True, [dict(row, z_val=1.9)]) == none


def test_golden_uses_the_frozen_bands(golden):
    bands = study.band_tables()
    for j, v in golden["verdicts"]["joints"].items():
        assert v["V1"]["b_v1_mrad"] == pytest.approx(bands[j]["b_v1_mrad"])
        assert v["V2"]["band_w_mrad"] == pytest.approx(bands[j]["band_w_mrad"])
        assert v["flags"]["N6"] == (bands[j]["f4_ratio"] >= 1.2), j


def test_golden_flags_follow_the_rules(golden):
    joints = golden["verdicts"]["joints"]
    n2 = {j for j, v in joints.items() if v["flags"]["N2"]}
    n5 = {j for j, v in joints.items() if v["flags"]["N5"]}
    n6 = {j for j, v in joints.items() if v["flags"]["N6"]}
    assert n2 == {"arm_3_joint", "arm_7_joint"}
    assert n5 == {"arm_7_joint"}
    assert n6 == {"arm_5_joint", "arm_6_joint"}
    for j, tr in golden["training"]["joints"].items():
        assert joints[j]["flags"]["N2"] == (tr["stats"]["overlap"] < 0.5), j


def test_golden_v1_follows_the_rule(golden):
    for j, v in golden["verdicts"]["joints"].items():
        v1 = v["V1"]
        rm = golden["validation"]["joints"][j]["rmse_mrad"]
        assert v1["g_mrad"] == pytest.approx(rm["M*"] - rm["B1"], abs=1e-6), j
        want = study.classify_v1(v1["g_mrad"], v1["b_v1_mrad"])
        assert v1["class"] == want["class"], j
        assert v1["borderline"] == want["borderline"], j
        if v["flags"]["N6"]:
            assert v1["verdict"] == "not answerable", j
            assert not v1["answerable"]
        else:
            assert v1["verdict"] == v1["class"], j


def test_golden_v2_follows_the_rule(golden):
    for j, v in golden["verdicts"]["joints"].items():
        v2 = v["V2"]
        assert v2["dW_mrad"] == pytest.approx(
            v2["W_train_mrad"] - v2["W_val_mrad"], abs=1e-6
        )
        if v["flags"]["N2"]:
            assert not v2["answerable"], j
        else:
            assert v2["verdict"] == study.classify_v2(
                v2["dW_mrad"], v2["band_w_mrad"]
            ), j


def test_golden_v3_needs_an_admitted_load_group(golden):
    for j, v in golden["verdicts"]["joints"].items():
        v3 = v["V3"]
        if v3["answerable"] and not v3["load_admitted_train"]:
            assert v3["verdict"] == "no load dependence detected", j


def test_golden_selection_matches_v3(golden):
    for j, tr in golden["training"]["joints"].items():
        v3 = golden["verdicts"]["joints"][j]["V3"]
        assert v3["load_admitted_train"] == ("L" in tr["selection"]["groups"]), j


def test_primary_model_is_all_groups(golden):
    assert "all groups" in golden["verdicts"]["primary_model"].lower()
    for tr in golden["training"]["joints"].values():
        assert tr["models"]["M3"]["groups"] == "EWL"


def test_inspection_is_descriptive(golden):
    ins = golden["inspection"]
    assert "arm5_excluded" in ins
    assert len(ins["arm5"]) + len(ins["arm5_excluded"]) == 14
    for bag in ins["arm5"].values():
        assert (
            abs(bag["predict_M*_rmse_hey5_mrad"] - bag["predict_M*_rmse_schunk_mrad"])
            < 2.0
        )


def test_note_quotes_the_golden(golden):
    lines = NOTE.read_text().splitlines()

    def row(key):
        hits = [ln for ln in lines if f"| `{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    for j, tr in golden["training"]["joints"].items():
        m = tr["models"]["M3"]
        line = row(f"train {j}")
        assert f"{m['W_mrad']:.2f}" in line and f"{m['W_se_mrad']:.2f}" in line
        assert f"{m['rmse_mrad']:.2f}" in line
        assert tr["selection"]["groups"] in line
    for j, v in golden["validation"]["joints"].items():
        line = row(f"val {j}")
        for m, x in v["rmse_mrad"].items():
            assert f"{x:.2f}" in line, (j, m)
    for j, v in golden["verdicts"]["joints"].items():
        line = row(f"V {j}")
        assert f"{v['V1']['g_mrad']:.2f}" in line
        assert f"{v['V1']['b_v1_mrad']:.2f}" in line
        assert v["V1"]["verdict"] in line
        assert v["V2"]["verdict"] in line or "not answerable" in line
        for n, on in v["flags"].items():
            assert (n in line) == on, (j, n)
    ins = golden["inspection"]
    for name, p in ins["probes"].items():
        line = row(f"probe {name}")
        assert f"{p['abs_p2p_mrad']:.1f}" in line
        assert f"{p['W_over_probe']:.2f}" in line
    for name, b in ins["arm5"].items():
        line = row(f"bag {name}")
        assert f"{b['W_mrad']:.1f}" in line
        assert f"{b['arm5_angle_rad']:+.2f}" in line


def test_validate_runs_the_study():
    text = (ROOT / "validate.py").read_text()
    assert '("backlash_study.py", 300, False, ["--archive", "--no-plot"])' in text


def test_study_matches_golden(golden):
    res = study.golden(study.study(verbose=False))
    for j, tr in golden["training"]["joints"].items():
        a, b = res["training"]["joints"][j], tr
        assert a["selection"]["groups"] == b["selection"]["groups"]
        assert a["models"]["M3"]["W_mrad"] == pytest.approx(
            b["models"]["M3"]["W_mrad"], abs=TOL
        )
    for j, v in golden["validation"]["joints"].items():
        for m, x in v["rmse_mrad"].items():
            got = res["validation"]["joints"][j]["rmse_mrad"][m]
            assert got == pytest.approx(x, abs=TOL), (j, m)
    for j, v in golden["verdicts"]["joints"].items():
        got = res["verdicts"]["joints"][j]
        assert got["flags"] == v["flags"], j
        for k in ("V1", "V2", "V3"):
            assert got[k]["verdict"] == v[k]["verdict"], (j, k)
    for name, b in golden["inspection"]["arm5"].items():
        got = res["inspection"]["arm5"][name]
        assert got["W_mrad"] == pytest.approx(b["W_mrad"], abs=0.2), name
