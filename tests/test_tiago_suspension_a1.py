"""TIAGo base suspension (#70) amendment A1: the series reading on the confirmation runs.

The fast tests check the A1 items and the reading on numbers, and, once
``golden/tiago_suspension_a1.json`` exists, that its reading follows from its
stored numbers under the frozen ``amendment_a1`` thresholds.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from examples.tiago import suspension_a1 as a1  # noqa: E402
from examples.tiago import suspension_model as sm  # noqa: E402

GOLDEN = ROOT / "tests/golden/tiago_suspension_a1.json"
ZERO6 = [False] * 6


def _items(**kw):
    base = dict(
        r2_s1=[0.9] * 6, r2_arm=[0.9] * 6, k_train=[1e4] * 6, k_run=[1.1e4] * 6,
        train_bound=ZERO6, run_bound=ZERO6, snr=[50.0] * 6, r2_b2=[0.9] * 6,
    )  # fmt: skip
    base.update(kw)
    return a1.items(**base)


def test_a1_items_apply_the_frozen_thresholds():
    it = _items(
        r2_s1=[0.5, 0.49, 0.9, 0.9, 0.9, 0.9],
        k_run=[1.25e4, 1.26e4, 1e4, 1e4, 1e4, 1e4],
        train_bound=[False, False, True, False, False, False],
        snr=[10.0, 9.9, 50, 50, 50, 50],
    )
    assert it["transfers"][:2] == [True, False]
    assert it["k_holds"][:3] == [True, False, False]  # band 0.25; bound fails
    assert it["base_moves"][:2] == [True, False]
    assert _items(k_train=[0.0] * 6)["k_holds"] == ZERO6  # k = 0 at training: no band


def test_a1_reading_needs_both_runs():
    good = _items()
    rigid = _items(r2_s1=[0.1] * 6)
    other = _items(r2_s1=[0.1] * 6, r2_b2=[0.1] * 6)
    assert a1.reading([good, good]) == ["series compliance"] * 6
    assert a1.reading([good, rigid]) == ["rigid base"] * 6  # S1 fails one run, B2 holds
    assert a1.reading([good, other]) == ["no answer"] * 6
    assert a1.reading([good, _items(r2_arm=[0.1] * 6)]) == ["no answer"] * 6


def test_a1_uses_the_frozen_settings():
    assert a1.A1 is sm.SETTINGS["amendment_a1"] and a1.MZ_ABOUT == "centre"


@pytest.mark.skipif(not GOLDEN.exists(), reason="A1 not read yet")
def test_stored_a1_reading_follows_from_the_stored_numbers():
    g = json.loads(GOLDEN.read_text())
    assert g["a1"] == sm.SETTINGS["amendment_a1"]
    runs = []
    for sid in sm.SETTINGS["sessions"]["confirmation"]:
        r = g["confirmation"][sid]
        it = a1.items(
            r["r2_s1"], r["r2_arm"], g["training"]["k"], r["k_run"],
            g["training"]["at_bound"], r["run_bound"], r["snr"], r["r2_b2"],
        )  # fmt: skip
        assert it == r["items"], sid
        runs.append(it)
    assert a1.reading(runs) == g["reading"]


@pytest.mark.slow
@pytest.mark.real_config
@pytest.mark.skipif(not GOLDEN.exists(), reason="A1 not read yet")
def test_a1_rerun_matches_the_golden():
    g = json.loads(GOLDEN.read_text())
    res = a1.ss._clean(a1.run(verbose=False))
    assert res["reading"] == g["reading"]
    for sid, r in g["confirmation"].items():
        for key in ("r2_s1", "r2_arm", "r2_b2", "snr"):
            assert np.allclose(res["confirmation"][sid][key], r[key], atol=1e-6), (sid, key)
