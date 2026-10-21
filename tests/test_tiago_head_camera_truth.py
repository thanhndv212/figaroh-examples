"""TIAGo head camera (#101): the simulated fixture and its golden.

``golden/tiago_head_camera_truth.json`` holds the pre-registered summary (F1-F6
at every grid level) and seed 0 in full. The fast tests check the fixture's
gauge (F1) on seed 0, the noise streams and that the pre-registration note
quotes the golden values; the slow test reruns the whole grid. Simulated
measurements only: no real-data joint fit.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import head_camera_truth as ht  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_head_camera_truth.json"
NOTE = ROOT / "docs/development/tiago-head-camera.md"


@pytest.fixture(scope="module")
def fx():
    return ht.Fixture()


def test_streams_are_independent_and_reproducible():
    draws = {k: ht.stream(k, 0).normal(size=200) for k in ht.STREAMS}
    kinds = list(ht.STREAMS)
    for i, a in enumerate(kinds):
        assert np.array_equal(draws[a], ht.stream(a, 0).normal(size=200))
        for b in kinds[i + 1 :]:
            assert abs(np.corrcoef(draws[a], draws[b])[0, 1]) < 0.3
    assert not np.array_equal(
        ht.stream("camera", 0).normal(size=5), ht.stream("camera", 1).normal(size=5)
    )


def test_truth_is_reproducible(fx):
    a, b = fx.draw_truth(2), fx.draw_truth(2)
    assert np.array_equal(a.off, b.off) and np.array_equal(a.X, b.X)
    c = fx.draw_truth(2, head=[0.0112, 0.0104])
    assert np.allclose(c.off[1:3], [0.0112, 0.0104])
    assert np.array_equal(a.off[3:], c.off[3:])
    assert np.array_equal(a.Y[fx.train.id], c.Y[fx.train.id])


def test_f1_gauge_on_seed_0(fx):
    """Rank 19 of 22 on every session, the analytic null space, the near-null."""
    g = json.loads(GOLDEN.read_text())["seed0"]["f1"]
    cell = ht.f1_cell(fx, 0, 2.0)
    crit = ht.FX["f1"]
    assert set(cell["ranks"].values()) == {crit["rank"]}
    assert cell["null_angle"] < crit["null_angle_max"]
    assert cell["near_null"] < crit["near_null_max"]
    assert cell["near_null_concentration"] > crit["near_null_concentration_min"]
    assert cell["near_null_rot_fraction"] < crit["near_null_rot_fraction_max"]
    assert cell["near_null"] == pytest.approx(g["near_null"], rel=1e-3)


def test_note_quotes_the_golden_values():
    g = json.loads(GOLDEN.read_text())
    text = NOTE.read_text()
    assert "Pre-registered thresholds" in text
    lines = text.splitlines()

    def row(key):
        hits = [ln for ln in lines if f"`{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    n = g["noise"]["camera"]
    assert f"{n['sigma_z_mm']:.2f}" in row("noise camera")
    assert f"{n['level_mm']:.1f}" in row("noise camera")
    assert f"{g['noise']['mocap']['pose_std_mm']:.2f}" in row("noise mocap")
    used = g["levels"][str(g["used_mm"])]
    for name, v in used["f5"]["parameter_band_rad"].items():
        assert f"{v * 1e3:.1f}" in row(f"band {name}"), name
    for d, v in used["f5"]["transfer"].items():
        assert f"{v['pos_mm']['band']:.2f}" in row(f"transfer {d}"), d
        assert f"{v['rot_mrad']['band']:.1f}" in row(f"transfer {d}"), d
    assert f"{used['f2']['rms']:.2f}" in row("recovery z rms")
    f4 = used["f4"]
    assert ("in" if f4["k_in"] else "out") in row(f"depth scale k @ {g['used_mm']} mm")
    assert f"{f4['cond_nok']:.1f}" in row(f"depth scale k @ {g['used_mm']} mm")


@pytest.mark.slow
def test_full_grid_matches_golden():
    g = json.loads(GOLDEN.read_text())
    res = ht.run(tuple(g["seeds"]), verbose=False)
    assert ht.grade(res) == g["criteria"]
    assert all(g["criteria"][k] for k in ("F1", "F2", "F3"))
    for lv, c in g["levels"].items():
        r = res["grid"][lv]
        assert r["f1"]["pass"] == c["f1"]["pass"]
        assert r["f2"]["rms"] == pytest.approx(c["f2"]["rms"], abs=1e-3)
        assert r["f3"]["rms"] == pytest.approx(c["f3"]["rms"], abs=1e-3)
        assert r["f4"]["k_in"] == c["f4"]["k_in"]
        for name, v in c["f5"]["parameter_band_rad"].items():
            assert r["f5"]["parameter_band_rad"][name] == pytest.approx(
                v, rel=1e-3, abs=1e-5
            )
        for d, v in c["f5"]["transfer"].items():
            for m in ("pos_mm", "rot_mrad"):
                assert r["f5"]["transfer"][d][m]["band"] == pytest.approx(
                    v[m]["band"], rel=1e-3, abs=5e-3
                )
    s0 = g["seed0"]
    cell = res["grid"][str(g["used_mm"])]["cells"]
    for k, v in s0["f2"]["z"].items():
        assert cell["f2"][0]["z"][k] == pytest.approx(v, abs=1e-3)
    for k, v in s0["f5"]["par_diff_rad"].items():
        assert np.allclose(cell["f5"][0]["par_diff_rad"][k], v, atol=2e-4)
