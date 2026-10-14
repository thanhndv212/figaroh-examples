"""Simulation harness (#96) on a toy linear model with a known answer."""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples.calibration_harness.simulation import (  # noqa: E402
    FitResult,
    SimulationStudy,
    anisotropic,
    gaussian_mm,
    run_case,
    run_grid,
    summarize,
)

NAMES = ["a", "b", "c"]
SIGMA_MM = 0.5


def _design(q):
    x = q["x"].to_numpy()
    return np.column_stack([np.ones_like(x), x, x**2])


def _study(noise=gaussian_mm, outputs=1):
    rng = np.random.default_rng(99)
    train = pd.DataFrame({"x": rng.uniform(-1, 1, 40)})
    held = {"h": pd.DataFrame({"x": rng.uniform(-1, 1, 15)})}

    def predict(params, kind, q, handle=None):
        y = _design(q) @ np.array([params[n] for n in NAMES])
        return np.column_stack([y] * outputs)

    def fit(df, level, estimation, workdir):
        A = _design(df)
        y = df[[f"y{k}" for k in range(outputs)]].to_numpy()[:, 0]
        theta, *_ = np.linalg.lstsq(A, y, rcond=None)
        cov = np.linalg.inv(A.T @ A) * (SIGMA_MM * 1e-3) ** 2
        return FitResult(
            dict(zip(NAMES, theta)),
            dict(zip(NAMES, np.sqrt(np.diag(cov)))),
            "lin",
            0,
            1,
        )

    def to_dataset(meas, q):
        cols = {f"y{k}": meas[:, k] for k in range(meas.shape[1])}
        return pd.concat([pd.DataFrame(cols), q.reset_index(drop=True)], axis=1)

    return SimulationStudy(
        sample_truth=lambda kind, rng: dict(zip(NAMES, rng.normal(0, 1, 3))),
        predict=predict,
        fit=fit,
        to_dataset=to_dataset,
        train_postures=train,
        heldout_postures=held,
        noise=noise,
        z_params={"lin": lambda n: True},
    )


def test_noise_free_recovery_is_exact(tmp_path):
    r = run_case(_study(), "lin", "lin", 0, 0.0, tmp_path)
    assert r["heldout_rmse_mm"] < 1e-9 and r["train_rms_mm"] < 1e-9
    assert set(r) >= {
        "truth",
        "fit",
        "seed",
        "noise_mm",
        "n_params",
        "solver_status",
        "nfev",
        "absorbed",
        "train_rms_mm",
        "heldout_rmse_mm",
        "heldout_max_mm",
        "strata_rmse_mm",
        "z_scores",
    }


def test_z_scores_have_unit_rms(tmp_path):
    z = []
    for seed in range(5):
        r = run_case(_study(), "lin", "lin", seed, SIGMA_MM, tmp_path)
        z += list(r["z_scores"].values())
    rms = float(np.sqrt(np.mean(np.square(z))))
    assert len(z) == 15 and 0.5 < rms < 1.6


def test_seeded_repeatability_and_seed_dependence(tmp_path):
    a = run_case(_study(), "lin", "lin", 3, 1.0, tmp_path)
    b = run_case(_study(), "lin", "lin", 3, 1.0, tmp_path)
    c = run_case(_study(), "lin", "lin", 4, 1.0, tmp_path)
    assert a == b and a != c


def test_no_z_when_fit_level_differs(tmp_path):
    r = run_case(_study(), "lin", "other", 0, 1.0, tmp_path)
    assert "z_scores" not in r


def test_anisotropic_per_column_std():
    sig = [1.0, 4.0, 0.25]
    clean = np.zeros((20000, 3))
    out = anisotropic(sig)(clean, 2.0, np.random.default_rng(0))
    np.testing.assert_allclose(out.std(axis=0), 2.0 * np.array(sig), rtol=0.03)
    with pytest.raises(ValueError):
        anisotropic([1.0])(clean, 1.0, np.random.default_rng(0))


def test_gaussian_mm_is_order_preserving():
    clean = np.ones((5, 3))
    got = gaussian_mm(clean, 0.7, np.random.default_rng(10_000))
    want = clean + np.random.default_rng(10_000).normal(0.0, 0.7e-3, clean.shape)
    np.testing.assert_array_equal(got, want)


def test_grid_and_summary(tmp_path):
    df = run_grid(
        _study(),
        ["lin"],
        {"lin": ("lin", None)},
        seeds=range(2),
        noise=(0.5,),
        workdir=tmp_path,
    )
    assert len(df) == 2
    table, z = summarize(df)
    assert list(table.index.names) == ["truth", "noise_mm", "fit"]
    assert z["n"] == 6
