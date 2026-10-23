"""TIAGo base suspension (#70): the simulated fixture and its golden.

``golden/tiago_suspension_truth.json`` holds the pre-registered summary (F1-F5)
and seed 0 in full. The fast tests check the fixture on seed 0 and that the
pre-registration note quotes the golden values; the slow test reruns the whole
grid. Simulated measurements only: no real-data fit of a suspension.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import suspension_truth as st  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_suspension_truth.json"
NOTE = ROOT / "docs/development/tiago-suspension.md"


@pytest.fixture(scope="module")
def fx():
    return st.Fixture()


@pytest.fixture(scope="module")
def gold():
    return json.loads(GOLDEN.read_text())


def _rows(note, prefix):
    """``{key: [cells]}`` of the note's table rows whose first cell is `key`."""
    out = {}
    for line in note.splitlines():
        if line.startswith(f"| `{prefix}"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            out[cells[0].strip("`")] = cells[1:]
    return out


def test_streams_are_independent_and_reproducible():
    for kind in st.STREAMS:
        assert np.array_equal(
            st.stream(kind, 0).normal(size=100), st.stream(kind, 0).normal(size=100)
        )
    a = st.stream("white", 0).normal(size=500)
    b = st.stream("clock", 0).normal(size=500)
    assert abs(np.corrcoef(a, b)[0, 1]) < 0.2
    assert not np.array_equal(
        st.stream("white", 0).normal(size=5), st.stream("white", 1).normal(size=5)
    )


def test_truth_is_reproducible_and_inside_the_spread():
    a, b = st.draw_truth(2), st.draw_truth(2)
    assert np.array_equal(a.k, b.k) and np.array_equal(a.c, b.c)
    k0 = np.asarray(st.FX["truth"]["k"])
    assert np.all(np.abs(a.k / k0 - 1.0) <= st.FX["truth"]["spread"] + 1e-12)
    assert not np.array_equal(st.draw_truth(3).k, a.k)


def test_f1_identifiability_matches_the_golden(gold):
    cells = st.f1_cells()
    for role, g in gold["f1"].items():
        assert cells[role]["pass"] == g["pass"]
        assert g["pass"] is True
        for m, v in g["cond"].items():
            assert cells[role]["cond"][m] == pytest.approx(v, rel=1e-3)


def test_f2_seed_0_matches_the_golden(fx, gold):
    g = gold["seed0"]["f2"]
    cell = st.f2_cell(fx, 0, st.USED)
    assert np.allclose(cell["z"], g["z"], atol=2e-3)
    assert cell["rel_err_max"] == pytest.approx(g["rel_err_max"], rel=1e-2)


def test_noise_free_recovery_is_exact(fx, gold):
    cell = st.f2_cell(fx, 0, 0.0)
    assert cell["rel_err_max"] <= st.FX["f2"]["exact_tol"]
    assert gold["criteria"]["F2_exact"] is True


def test_f4a_seed_0_matches_the_golden(fx, gold):
    cell = st.f4a_cell(fx, 0, st.USED, gold["context"])
    g = gold["seed0"]["f4a"]
    assert cell["gain"] == pytest.approx(g["gain"], abs=2e-3)
    assert cell["h_e_cv"] == pytest.approx(g["h_e_cv"])


def test_criteria_are_recorded_honestly(gold):
    """The two failed checks stay failed: thresholds are not relaxed."""
    c = gold["criteria"]
    assert c["F1"] and c["F2_exact"] and c["F2_gap"] and c["F2_bias"]
    assert c["F3_attribution"] and c["F3_full"] and c["F3b_signature"]
    assert c["F4a"] and c["F4b"] and c["F4c"] and c["F4c_q2"] and c["F4d"]
    assert c["F5"]
    assert not c["F2_z"] and not c["F3_signature"]
    assert gold["f3"]["signature_seeds"] == 0


def test_f2_bias_is_errors_in_variables(gold):
    """The bootstrap misses a common-sign bias; removing the regressor noise
    power centres the z (a sensitivity row, not the estimator)."""
    for lv in ("0.5", "1.0", "2.0"):
        e = gold["f2"][lv]
        assert e["z"]["mean"] < -0.25
        assert abs(e["eiv"]["z"]["mean"]) < 0.2
        assert 0.75 <= e["eiv"]["z"]["rms"] <= 1.33
        assert not any(e["c_unidentified"])
    b = gold["f2_bias"]
    assert b["k_max"] <= st.FX["f2"]["bias_max"]["k"]
    assert b["c_max"] <= st.FX["f2"]["bias_max"]["c"]
    ctx = gold["context"]
    assert all(
        cal >= se for cal, se in zip(ctx["c_rel_se_cal"], ctx["c_rel_se"])
    )


def test_f4a_pass_comes_from_selection_not_the_lever(gold):
    """At the base_link height the S2 gain is small; the pass is S2 at the
    ceiling with h_e found, at every lever of the sensitivity rows."""
    assert st.FX["truth"]["h_e"] == pytest.approx(0.0985)
    assert "s2_gain_min" not in st.FX["f4"]
    a = gold["f4"]["a"]
    assert a["pass"] and a["ok_seeds"] == 5 and a["gain"] < 0.1
    assert all(m == "S2" for m in a["models"])
    gains = []
    for h, e in sorted(gold["f4"]["a_sensitivity"].items(), key=lambda t: float(t[0])):
        assert e["pass"]
        assert all(abs(x - float(h)) <= st.FX["f4"]["h_e_tol"] for x in e["h_e_cv"])
        gains.append(e["gain"])
    assert gains[0] < a["gain"] < gains[1]  # the gain follows the lever


def test_f4c_truth_is_a_cube_and_s1_sits_on_a_bound(gold):
    c = gold["f4"]["c"]
    assert c["not_enough_seeds"] == 5 and c["pass"]
    assert c["mass_hat"] == pytest.approx(25.0, rel=0.1)
    assert len(c["o_hat"]) == 3 and c["s1_at_bound_dofs"] >= 1.0
    assert c["inertia_max_dr2"] < 0.01  # the assumed edge hardly matters
    assert min(c["r2_b2"]) > 0.85 and max(c["r2_s1"]) < 0.9
    assert c["q2_pass"] and c["q2_absent_seeds"] == 5
    assert max(c["q2_gain"]) < st.RULES["suspension_adds"]


def test_f4d_mixed_truth_shows_the_suspension_through_q2(gold):
    d = gold["f4"]["d"]
    assert d["pass"] and d["s1_not_enough_seeds"] == 5
    assert d["s1b2_gap_seeds"] == 5 and d["q2_present_seeds"] == 5
    assert abs(d["s1b2_gap"]) <= st.FX["f4"]["s2_gap_max"]
    assert max(d["q2_gain"]) >= st.RULES["suspension_adds"]
    assert max(d["s1b2_k_rel_err"]) > 0.1  # stiffnesses confounded: descriptive


def test_f3b_mixed_truth_reproduces_the_signature(gold):
    f = gold["f3b"]
    assert f["signature_pass"] and f["signature_seeds"] >= 4
    assert f["forces_r2"] < 0.2 and f["my_r2"] >= 0.75


def test_f5_selects_the_footprint_elastic_centre(gold):
    f5 = gold["f5"]
    assert f5["s1_correct"] == 5 and f5["s2_correct"] == 5
    assert all(
        abs(h - st.FX["truth"]["h_e"]) <= st.FX["f4"]["h_e_tol"]
        for h in f5["h_e_elastic"]
    )


def test_origin_sensitivity_is_descriptive_and_small_at_the_survey_scale(gold):
    o = gold["origin"]
    for ax in ("x", "y", "z"):
        for sign in "+-":
            # descriptive, not pre-registered: 6 % on x at 5 Hz (3 % at 10 Hz)
            assert o[f"{ax}{sign}"]["max_rel"] < 0.1
            assert abs(o[f"{ax}{sign}"]["dr2"]) < 0.01
    assert o["centroid"]["dr2"] < -0.1 and o["centroid"]["max_rel"] > 0.5


def test_note_quotes_the_golden_values(gold):
    note = NOTE.read_text()
    assert "Pre-registered thresholds" in note
    rec = _rows(note, "recovery")
    for col, lv in enumerate(("0.5", "1.0", "2.0")):
        e = gold["f2"][lv]
        assert float(rec["recovery z rms"][col]) == pytest.approx(
            e["z"]["rms"], abs=0.006
        )
        assert float(rec["recovery z mean"][col]) == pytest.approx(
            e["z"]["mean"], abs=0.006
        )
        assert float(rec["recovery rel err"][col]) == pytest.approx(
            e["rel_err_max"], abs=0.006
        )
        assert float(rec["recovery eiv z rms"][col]) == pytest.approx(
            e["eiv"]["z"]["rms"], abs=0.006
        )
        assert float(rec["recovery eiv z mean"][col]) == pytest.approx(
            e["eiv"]["z"]["mean"], abs=0.006
        )
    bias = _rows(note, "bias")
    ctx = gold["context"]
    for name, key in (("bias k", "k_bias"), ("bias c", "c_bias")):
        for cell, v in zip(bias[name], ctx[key]):
            assert float(cell.rstrip(" %")) == pytest.approx(100 * v, abs=0.06)
    cal = _rows(note, "rel se c calibrated")["rel se c calibrated"]
    for cell, v in zip(cal, ctx["c_rel_se_cal"]):
        assert float(cell.rstrip(" %")) == pytest.approx(100 * v, abs=0.06)
    f3 = gold["f3"]
    assert f"`status quo forces r2` | {f3['forces_r2']:.2f}" in note
    assert f"`status quo my r2` | {f3['my_r2']:.2f}" in note
    assert f"`ablation origin` | +{f3['gain']['origin']:.2f}" in note
    assert f"**{gold['f4']['a']['gain']:.2f}** (`elastic gain`" in note
    assert f"`band` {gold['context']['band']:.4f}" in note
    for name in ("x+", "y+", "z+"):
        pct = 100 * gold["origin"][name]["max_rel"]
        assert f"| footprint +1 sd {name[0]} | {pct:.1f} %" in note
    assert "Decisions taken (maintainer, 2026-10-23)" in note
    assert "Revision (maintainer, 2026-10-23)" in note
    assert f"`signature seeds` | {f3['signature_seeds']} of 5" in note
    assert f"pass, {gold['f3b']['signature_seeds']} of 5" in note
    for m, v in gold["f1"]["training"]["cond"].items():
        assert f"`condition {m}` | {v:.2f}" in note


@pytest.mark.slow
def test_full_grid_matches_golden(gold):
    res = st.run(tuple(gold["seeds"]), verbose=False)
    assert st.grade(res) == gold["criteria"]
    for lv, e in gold["f2"].items():
        r = res["f2"][lv]
        # the golden rounds to 1e-6; level 0 is graded against exact_tol
        tol = st.FX["f2"]["exact_tol"] if lv == "0.0" else 1e-8
        assert r["rel_err_max"] == pytest.approx(e["rel_err_max"], rel=1e-2, abs=tol)
        if "z" in e:
            assert r["z"]["rms"] == pytest.approx(e["z"]["rms"], abs=2e-3)
            assert r["z"]["mean"] == pytest.approx(e["z"]["mean"], abs=2e-3)
    assert res["f3"]["signature_seeds"] == gold["f3"]["signature_seeds"]
    assert res["f4"]["a"]["gain"] == pytest.approx(gold["f4"]["a"]["gain"], abs=2e-3)
    assert res["f4"]["a"]["h_e_cv"] == gold["f4"]["a"]["h_e_cv"]
    assert res["f3b"]["signature_seeds"] == gold["f3b"]["signature_seeds"]
    assert res["f4"]["c"]["q2_absent_seeds"] == gold["f4"]["c"]["q2_absent_seeds"]
    assert res["f4"]["d"]["q2_present_seeds"] == gold["f4"]["d"]["q2_present_seeds"]
    assert res["f2_bias"]["c_max"] == pytest.approx(gold["f2_bias"]["c_max"], abs=2e-3)
    assert res["f5"]["pass"] == gold["f5"]["pass"]
    for name in ("x+", "y-", "z+", "centroid"):
        assert res["origin"][name]["dr2"] == pytest.approx(
            gold["origin"][name]["dr2"], abs=2e-3
        )
