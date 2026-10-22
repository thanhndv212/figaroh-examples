"""TIAGo backlash surface (#71): the simulated fixture and its golden.

``golden/tiago_backlash_truth.json`` holds the pre-registered summary (F1-F6 at
every noise level) and seed 0 in full. The fast tests check the fixture on
seed 0 and that the pre-registration note quotes the golden values; the slow
test reruns the whole grid. Simulated measurements only: no real-data fit.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import backlash_truth as bt  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_backlash_truth.json"
NOTE = ROOT / "docs/development/tiago-backlash.md"


@pytest.fixture(scope="module")
def fx():
    return bt.Fixture()


def test_streams_are_independent_and_reproducible():
    draws = {k: bt.stream(k, 0).normal(size=200) for k in bt.STREAMS}
    kinds = list(bt.STREAMS)
    for i, a in enumerate(kinds):
        assert np.array_equal(draws[a], bt.stream(a, 0).normal(size=200))
        for b in kinds[i + 1 :]:
            assert abs(np.corrcoef(draws[a], draws[b])[0, 1]) < 0.3
    assert not np.array_equal(
        bt.stream("ar1", 0).normal(size=5), bt.stream("ar1", 1).normal(size=5)
    )


def test_truth_is_reproducible_and_options_only_overwrite():
    a, b = bt.draw_truth(2), bt.draw_truth(2)
    assert np.array_equal(a.W, b.W) and np.array_equal(a.G, b.G)
    c = bt.draw_truth(2, w=0.012, load=False)
    assert np.allclose(c.W, 0.012) and not c.G.any() and not c.kappa.any()
    assert np.array_equal(a.e_a2, c.e_a2) and a.step_centre == c.step_centre
    d = bt.draw_truth(2, g_arm6=0.010)
    assert d.G[5] == 0.010 and not np.delete(d.G, 5).any()


def test_synthesized_q_abs_solves_the_truth_equation(fx):
    truth = bt.draw_truth(0)
    i = 4  # arm_5: the step
    m = fx.measure(truth, "training", i, 1.0)
    sk = fx.skel["training"]["arm_5_joint"]
    b = bt.play(sk.q_rel, truth.W[i], sk.h[np.argmax(sk.valid)])
    lhs = m["q_true"] + truth.E(i, m["q_true"])
    rhs = (
        sk.q_rel
        - truth.c["training"][i]
        - b
        - truth.G[i] * np.tanh(sk.tau / 0.5)
        - truth.kappa[i] * sk.tau
    )
    assert np.abs(lhs - rhs).max() < 1e-9


def test_f1_identifiability_on_real_regressor_columns(fx):
    g = json.loads(GOLDEN.read_text())["f1"]
    cells = bt.f1_cells(fx)
    for role, joints in cells.items():
        for j, r in joints.items():
            assert r["pass"], (role, j)
            assert r["cond"] == pytest.approx(g[role][j]["cond"], rel=1e-3)


def test_f2_f6_on_seed_0_match_the_golden(fx):
    g = json.loads(GOLDEN.read_text())["seed0"]
    z = bt.f2_cell(fx, 0, bt.USED)["z"]
    for k, v in g["f2"]["z"].items():
        assert z[k] == pytest.approx(v, abs=1e-3)
    f6 = bt.f6_cell(fx, 0, bt.USED)
    assert f6 == g["f6"]
    assert f6["found"] == f6["injected"] and f6["false"] == 0


def test_f3_step_does_not_leak_into_the_width(fx):
    cell = bt.f3_cell(fx, 0, bt.USED)
    z = cell["zero_q_abs"]
    assert abs(z["W_hat_mrad"]) < bt.FX["f3"]["w_zero_mrad"]
    w = cell["width_q_abs"]
    assert abs(w["W_hat_mrad"] - 12.0) < 0.25 * 12.0


def test_note_quotes_the_golden_values():
    g = json.loads(GOLDEN.read_text())
    text = NOTE.read_text()
    assert "Pre-registered thresholds" in text
    lines = text.splitlines()

    def row(key):
        hits = [ln for ln in lines if f"`{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    for j, v in g["noise"]["sigma_d_mrad"].items():
        assert f"{v:.2f}" in row(f"noise {j}"), j
        assert f"{g['noise']['rho'][j]:.2f}" in row(f"noise {j}"), j
    used = g["levels"][str(g["used"])]
    assert f"{used['f2']['rms']:.2f}" in row("recovery z rms")
    assert f"{used['f2']['mean']:.2f}" in row("recovery z mean")
    for j, v in used["f4"]["joints"].items():
        assert f"{v['band_v1_mrad']:.2f}" in row(f"band {j}"), j
        assert f"{v['band_w_mrad']:.2f}" in row(f"band {j}"), j
        assert f"{v['ratio']:.1f}" in row(f"transfer {j}"), j
        assert f"{v['ratio_m3']:.2f}" in row(f"transfer {j}"), j
    assert str(used["f5"]["true_admit"]) in row("selection true admit")
    assert str(used["f6"]["found"]) in row("jumps found")


@pytest.mark.slow
def test_full_grid_matches_golden():
    g = json.loads(GOLDEN.read_text())
    res = bt.run(tuple(g["seeds"]), verbose=False)
    assert bt.grade(res) == g["criteria"]
    for lv, c in g["levels"].items():
        r = res["grid"][lv]
        assert r["f2"]["rms"] == pytest.approx(c["f2"]["rms"], abs=1e-3)
        assert r["f2"]["mean"] == pytest.approx(c["f2"]["mean"], abs=1e-3)
        assert r["f3"]["pass"] == c["f3"]["pass"]
        assert r["f5"]["true_admit"] == c["f5"]["true_admit"]
        assert r["f6"] == c["f6"]
        for j, v in c["f4"]["joints"].items():
            assert r["f4"]["joints"][j]["ratio"] == pytest.approx(v["ratio"], rel=1e-3)
            assert r["f4"]["joints"][j]["ratio_m3"] == pytest.approx(
                v["ratio_m3"], rel=1e-3
            )
            assert r["f4"]["joints"][j]["band_v1_mrad"] == pytest.approx(
                v["band_v1_mrad"], rel=1e-3, abs=1e-4
            )
