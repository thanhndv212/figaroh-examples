"""TALOS table-contact recovers a known truth (figaroh-examples#98).

Synthetic touches at the real training postures from a drawn truth
(examples/talos_table_contact/calibration_truth.py), encoder noise on the
chain joints, scored by the height error of the fitted gap against the true
gap at the real validation postures. The seed-0 grid is pinned against the
``synthetic`` block of ``golden/talos_table_contact.json`` (3 decimals, so
rel 1e-4 plus abs 1e-3). The structural and cv_subset rows break ties by
platform (figaroh-plus#113): their values are compared only when the
platform keeps the golden's parameter count.

Mechanism check: the planned noise-free bound of 0.05 mm is not met. The
structural fit (ridge 1e-3) leaves 2.3 mm on left and 0.14 mm on right even
without noise, and without the ridge the held-out error grows (9.7 / 1.8 mm)
while the training gap goes to 1e-11 mm: 21 / 29 postures do not pin down
the 56 / 63 parameters, so held-out postures extrapolate. The test pins what
does hold (the pipeline fits the training data, the true gap is the sign
convention of the fit) and marks the 0.05 mm bound xfail.

Regenerate the synthetic block from examples/talos_table_contact with::

    python calibration_truth.py --seeds 0 --noise-deg 0.05 --mismatch --jobs 4 --json

and put it under the golden's ``synthetic`` key (the real block comes from
heldout_protocol.py --json, which does not write this key).
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "talos_table_contact"
GOLDEN = json.loads((ROOT / "tests/golden/talos_table_contact.json").read_text())
SYN = GOLDEN["synthetic"]["chains"]
for p in (str(ROOT), str(EXAMPLE)):
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("pinocchio")


@pytest.fixture(scope="module")
def ct():
    import calibration_truth

    return calibration_truth


def test_fits_are_the_protocol_models(ct):
    import heldout_protocol as hp

    assert list(ct.FITS) == list(hp.MODELS)
    for label, (_, est) in ct.FITS.items():
        if est:
            assert est["noise_std"] == hp.NOISE_STD, label
    # map gets the truth's own scales; the others keep core's defaults
    pri = ct.FITS["map"][1]["priors"]
    assert pri["translation"] == pytest.approx(1.5e-3)
    assert pri["rotation"] == pytest.approx(np.deg2rad(0.3))
    assert "priors" not in ct.FITS["map_cv"][1]


def test_seed_is_recovered_from_the_generator(ct):
    for seed in (0, 3, 12345):
        assert ct._seed_of(np.random.default_rng(seed)) == seed


def test_task_grid_counts(ct):
    full = ct.tasks(["left", "right"], range(5), ct.NOISE_DEG, list(ct.FITS), True)
    cases = sum(len(u[4]) for u in full)
    # 180 planar + 60 curved + 2 noise-free structural
    assert cases == 180 + 60 + 2


@pytest.fixture(scope="module")
def left(ct, tmp_path_factory):
    return ct.make_study("left"), tmp_path_factory.mktemp("truth")


def test_simulation_is_seeded_and_noise_free_is_clean(ct, left):
    study, _ = left
    truth = study.sample_truth("planar", np.random.default_rng(0))
    m0, q0 = study.simulate(study, truth, "planar", None, 0.05, 0)
    m1, q1 = study.simulate(study, truth, "planar", None, 0.05, 0)
    assert m0.shape == (21, 3) and not m0.any()
    assert q0.equals(q1)
    _, q_clean = study.simulate(study, truth, "planar", None, 0.0, 0)
    joints = ct.CHAINS["left"]["joints"]
    other = [c for c in q_clean.columns if c not in joints]
    assert q0[other].equals(q_clean[other])  # only the chain joints move
    dev = np.rad2deg(np.abs(q0[joints].to_numpy() - q_clean[joints].to_numpy()))
    assert 0.0 < dev.mean() < 0.2
    # the true gap at the synthesized postures is zero
    gap = study.predict(truth, "planar", q_clean)
    assert np.abs(gap).max() < 1e-6


@pytest.mark.slow
def test_noise_free_mechanism_left(ct, left):
    """The fit reproduces the training data; held-out is above the planned bound."""
    study, tmp = left
    row = ct.run_case(study, "left", "planar", "structural", 0, 0.0, tmp)
    assert row["train_rms_mm"] < 1.0
    assert row["heldout_rmse_mm"] < 3.0  # measured 2.31


@pytest.mark.slow
@pytest.mark.xfail(
    strict=True, reason="noise-free structural held-out is 2.3 mm on left, not < 0.05"
)
def test_noise_free_mechanism_bound_left(ct, left):
    study, tmp = left
    row = ct.run_case(study, "left", "planar", "structural", 0, 0.0, tmp)
    assert row["heldout_rmse_mm"] < ct.MECHANISM_TOL_MM


@pytest.fixture(scope="module")
def seed0_left(ct, left):
    study, tmp = left
    return {
        label: ct.run_case(study, "left", "planar", label, 0, 0.05, tmp)
        for label in ("table+contact only", "structural", "map")
    }


@pytest.mark.slow
def test_structural_beats_table_only_held_out(seed0_left):
    assert (
        seed0_left["structural"]["heldout_rmse_mm"]
        < seed0_left["table+contact only"]["heldout_rmse_mm"]
    )


@pytest.mark.slow
def test_seed0_matches_golden(ct, seed0_left):
    import heldout_protocol as hp

    gold = SYN["left"]["planar@0.05"]
    for label, row in seed0_left.items():
        g = gold[label]
        if label in hp.PLATFORM_DEPENDENT and row["n_params"] != g["n_params"]:
            continue
        assert row["n_params"] == g["n_params"], label
        for k in ("heldout_rmse_mm", "train_rms_mm"):
            assert row[k] == pytest.approx(g[k], rel=1e-4, abs=1e-3), (label, k)


if __name__ == "__main__":
    raise SystemExit(
        "regenerate with: python calibration_truth.py --json (see docstring)"
    )
