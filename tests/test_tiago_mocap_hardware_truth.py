"""TIAGo mocap across hardware (#100): the simulated fixture and its golden.

``golden/tiago_mocap_hardware_truth.json`` holds the first seed at the
pre-registered noise levels and the pre-registered summary (bands, effective
noise, recovery). The slow test reruns that seed; the fast tests check that
the systems' noise streams are independent and that the pre-registration note
quotes the golden values. Simulated measurements only: no real-data fit.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import mocap_hardware_truth as mt  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_mocap_hardware_truth.json"
NOTE = ROOT / "docs/development/tiago-mocap-hardware.md"


def test_streams_are_independent_and_reproducible():
    draws = {s: mt.stream(s, 0).normal(size=200) for s in mt.SYSTEMS}
    for i, a in enumerate(mt.SYSTEMS):
        assert np.array_equal(draws[a], mt.stream(a, 0).normal(size=200))
        for b in mt.SYSTEMS[i + 1 :]:
            assert abs(np.corrcoef(draws[a], draws[b])[0, 1]) < 0.3
    assert not np.array_equal(
        mt.stream("optitrack", 0).normal(size=5),
        mt.stream("optitrack", 1).normal(size=5),
    )


def test_note_quotes_the_golden_values():
    g = json.loads(GOLDEN.read_text())
    text = NOTE.read_text()
    assert "Pre-registered thresholds" in text
    lines = text.splitlines()

    def row(key):
        hits = [ln for ln in lines if f"`{key}`" in ln]
        assert hits, f"{key} missing from the note"
        return hits[0]

    for system, level in g["noise_estimates_mm"].items():
        assert f"{level:.1f}" in row(f"noise {system}")
    pre = g["preregistered"]
    for key, v in pre["transfer_band_mm"].items():
        assert f"{v['band']:.2f}" in row(key), key
    for key, v in pre["parameter_band_rad"].items():
        for name, x in v.items():
            assert f"{x * 1e3:.2f}" in row(f"{key}"), (key, name)
    assert f"{pre['recovery_z']['rms']:.2f}" in row("recovery z rms")


@pytest.mark.slow
def test_seed0_matches_golden(tmp_path):
    g = json.loads(GOLDEN.read_text())["seed0"]
    fx = mt.Fixture()
    cell = mt.run_cell(fx, g["seed"], g["levels"], tmp_path)
    assert cell["absorbed"] == [tuple(a) for a in g["absorbed"]]
    assert cell["status"] == g["status"]
    assert np.allclose(cell["z"], g["z"], atol=1e-3)
    for k, params in g["params"].items():
        for n, v in params.items():
            assert cell["params"][k][n] == pytest.approx(v, abs=2e-4)
    for k, metrics in g["matrix"].items():
        for m, v in metrics.items():
            assert cell["matrix"][k][m] == pytest.approx(v, abs=0.05, rel=1e-3), (k, m)
