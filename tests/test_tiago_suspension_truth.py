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
    """The three failed checks stay failed: thresholds are not relaxed."""
    c = gold["criteria"]
    assert c["F1"] and c["F2_exact"] and c["F2_gap"]
    assert c["F3_attribution"] and c["F3_full"]
    assert c["F4b"] and c["F4c"] and c["F5"]
    assert not c["F2_z"] and not c["F3_signature"] and not c["F4a"]


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
    f3 = gold["f3"]
    assert f"`status quo forces r2` | {f3['forces_r2']:.2f}" in note
    assert f"`status quo my r2` | {f3['my_r2']:.2f}" in note
    assert f"`ablation origin` | +{f3['gain']['origin']:.2f}" in note
    assert f"**{gold['f4']['a']['gain']:.2f}** (`elastic gain`" in note
    assert f"`band` {gold['context']['band']:.4f}" in note
    for m, v in gold["f1"]["training"]["cond"].items():
        assert f"`condition {m}` | {v:.2f}" in note


@pytest.mark.slow
def test_full_grid_matches_golden(gold):
    res = st.run(tuple(gold["seeds"]), verbose=False)
    assert st.grade(res) == gold["criteria"]
    for lv, e in gold["f2"].items():
        r = res["f2"][lv]
        assert r["rel_err_max"] == pytest.approx(e["rel_err_max"], rel=1e-2, abs=1e-8)
        if "z" in e:
            assert r["z"]["rms"] == pytest.approx(e["z"]["rms"], abs=2e-3)
            assert r["z"]["mean"] == pytest.approx(e["z"]["mean"], abs=2e-3)
    assert res["f3"]["signature_seeds"] == gold["f3"]["signature_seeds"]
    assert res["f4"]["a"]["gain"] == pytest.approx(gold["f4"]["a"]["gain"], abs=2e-3)
    assert res["f5"]["pass"] == gold["f5"]["pass"]
