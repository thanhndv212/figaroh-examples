"""TIAGo base suspension (#70) part 2: the real fits and their verdict.

``golden/tiago_suspension_study.json`` is the output of
``examples/tiago/suspension_study.py`` on the real runs. The fast tests check
that the stored verdict follows from the stored numbers under the frozen rules
of part 1, that the fixture context is the frozen one, and that the note quotes
the result; the slow test reruns the study. The golden is the read after the
plate ``Fy`` revision (note, 12.2).
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import suspension_model as sm  # noqa: E402
from examples.tiago import suspension_study as ss  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_suspension_study.json"
NOTE = ROOT / "docs/development/tiago-suspension.md"


@pytest.fixture(scope="module")
def gold():
    return json.loads(GOLDEN.read_text())


def _arr(x):
    return np.array([np.inf if v == "inf" else v for v in x], dtype=float)


@pytest.mark.parametrize("conv", ss.CONVENTIONS)
def test_stored_verdict_follows_from_the_stored_numbers(gold, conv):
    r = gold[conv]
    rules = sm.RULES
    r2 = r["r2_validation"]
    rep = _arr(r["k_rel_diff"]) <= gold["context"]["band"]
    nonlin = np.abs(r["nonlinear_corr"]) > rules["nonlinear_corr"]
    assert rep.tolist() == r["repeatable"] and nonlin.tolist() == r["nonlinear"]
    v = sm.verdict(
        np.array(r2[r["model"]]), np.array(r2["B2"]), np.array(r["at_bound"]),
        rep, nonlin, gold["context"]["answerable"], r["b2"]["at_bound"],
    )  # fmt: skip
    assert v == r["verdict"]
    q2 = sm.suspension_adds(r2["S1B2"], r2["B2"], r["s1b2"]["k_at_bound"])
    assert q2["adds"] == r["q2"]["adds"] and q2["present"] == r["q2"]["present"]


def test_the_study_uses_the_frozen_fixture_context(gold):
    ctx = ss.fixture_context()
    for k in ("answerable", "band", "k_bias", "c_bias"):
        assert gold["context"][k] == ctx[k]


def test_the_verdict_is_not_enough_and_the_confirmation_runs_stay_unread(gold):
    f = gold["final"]
    assert f["enough"] is False and f["q2_present"] is False
    assert f["mz_same_verdict"] is True and all(f["answerable"])
    assert isinstance(gold["confirmation"], str) and "not read" in gold["confirmation"]
    # corrected read (Fy revision, note 12.2): S3, only rx explained, B2 not beaten
    r = gold["centre"]
    assert r["model"] == "S3" and f["explained_dofs"] == ["rx"]
    assert not any(r["verdict"]["beats_b2"])


def test_corrected_b2_mass_matches_the_plate_force_regressions(gold):
    """Note 12.2: the B2 cube mass falls in the -m of Fx, Fy on the gripper acceleration."""
    assert 5.6 <= gold["centre"]["b2"]["mass"] <= 6.7


def test_b1_reproduces_the_status_quo_signature_on_the_real_runs(gold):
    sig = sm.SETTINGS["fixture"]["f3"]["signature"]
    b1 = gold["b1"]
    assert np.mean(b1["r2"][:3]) <= sig["forces_r2_max"]
    assert b1["r2"][4] >= sig["my_r2_min"]
    assert b1["neg_damping"] >= sig["neg_damping_min"]


def test_combine_drops_mz_when_the_conventions_disagree():
    ok = [True] * 6
    a = {"verdict": {"dof_ok": ok, "explained": ok}, "b2": {"at_bound": False}, "q2": {"adds": [False] * 6}}
    b = {"verdict": {"dof_ok": ok[:5] + [False]}}
    ctx = {"answerable": [True] * 6}
    out = ss.combine({"centre": a, "cop": b}, ctx)
    assert out["mz_same_verdict"] is False and out["answerable"][ss.MZ] is False
    assert out["enough"] is True  # the five other DOF pass
    a["verdict"]["dof_ok"] = [False] + ok[1:]
    assert ss.combine({"centre": a, "cop": b}, ctx)["enough"] is False


def test_note_quotes_the_part_2_result(gold):
    note = NOTE.read_text()
    assert "## 12. Part 2: the real fits" in note
    r = gold["centre"]
    sel = r["selection"]
    assert f"the selected model is {r['model']}" in note
    assert f"`h_e` = {sel['h_e_cv']:.2f} m" in note
    assert "### 12.2 Revision: the plate `Fy` sign" in note
    for name in ("S1", "S2", "S3", "B2", "S1B2"):
        cells = " | ".join(f"{x:.2f}" for x in r["r2_validation"][name])
        assert f"| {name} | {cells} |" in note, name
    b1 = " | ".join(f"{x:.2f}" for x in gold["b1"]["r2"])
    assert f"| B1 | {b1} |" in note
    assert "**not enough**" in note


@pytest.mark.slow
@pytest.mark.real_config
def test_study_rerun_matches_the_golden(gold):
    res = ss._clean(ss.run(verbose=False))
    assert res["final"] == gold["final"]
    for conv in ss.CONVENTIONS:
        a, b = res[conv], gold[conv]
        assert a["model"] == b["model"] and a["h_e"] == b["h_e"]
        assert a["verdict"] == b["verdict"]
        for name, v in b["r2_validation"].items():
            assert np.allclose(a["r2_validation"][name], v, atol=1e-5), (conv, name)
        assert np.allclose(a["selected"]["k"], b["selected"]["k"], rtol=1e-6)


def test_amendment_a1_is_frozen_before_the_confirmation_read(gold):
    a1 = sm.SETTINGS["amendment_a1"]
    assert a1 == {
        "model": "S1", "transfer_r2_min": 0.5, "arm_load_r2_min": 0.5,
        "k_rel_band": 0.25, "base_motion_snr_min": 10.0, "rigid_b2_r2_min": 0.5,
    }  # fmt: skip
    assert "## 13. Amendment A1: the series reading" in NOTE.read_text()
    assert "not read" in gold["confirmation"]
