"""Synthetic-truth studies for calibration (#96).

Draw a truth, simulate measurements at given postures, fit them, and judge
the fit against the truth in identifiable coordinates: the prediction on
held-out postures against the noise-free truth, and, when the fit has the
truth's model class, the z-score of the parameters the fit keeps.

A robot supplies a :class:`SimulationStudy` (how to draw, predict, fit and
write a dataset); everything else is shared. The random streams are fixed:
the truth comes from ``default_rng(seed)``, the noise from
``default_rng(10_000 + seed)``, and the noise model draws in one call, so a
study reproduces its numbers for a given seed.

Gauge: some truth parameters are absorbed by estimated frames and cannot be
recovered. ``z_params`` names the parameters a fit keeps per model class;
nothing outside it is scored.
"""

from __future__ import annotations

import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

# (clean (n, m), level, rng) -> noisy (n, m)
NoiseModel = Callable[[np.ndarray, float, np.random.Generator], np.ndarray]


def gaussian_mm(
    clean: np.ndarray, level: float, rng: np.random.Generator
) -> np.ndarray:
    """Isotropic Gaussian noise of ``level`` millimetres per component."""
    return clean + rng.normal(0.0, level * 1e-3, clean.shape)


def anisotropic(sigmas: Sequence[float]) -> NoiseModel:
    """Noise with a standard deviation ``level * sigmas[j]`` on column ``j``.

    ``sigmas`` is in the units of the measurement (so ``level`` is a
    multiplier); it must have one entry per column.
    """
    sig = np.asarray(sigmas, dtype=float)

    def model(clean: np.ndarray, level: float, rng: np.random.Generator):
        if clean.shape[1] != sig.size:
            raise ValueError(f"{sig.size} sigmas for {clean.shape[1]} columns")
        return clean + rng.normal(0.0, 1.0, clean.shape) * (level * sig)

    return model


def _row_error(e: np.ndarray) -> np.ndarray:
    return 1000 * np.linalg.norm(e, axis=1)


@dataclass
class FitResult:
    """What a fit returns.

    ``params`` and ``std`` map parameter name to value / standard deviation
    (same order as the fit's parameter vector); ``level`` is the model class
    used to predict (``"joint_offset"`` for a registration-only fit);
    ``handle`` is an opaque fitted object that ``predict`` may need.
    """

    params: Dict[str, float]
    std: Dict[str, float]
    level: str
    status: int
    nfev: int
    absorbed: List[str] = field(default_factory=list)
    handle: Any = None


@dataclass
class SimulationStudy:
    """Everything robot-specific about a synthetic-truth study.

    - ``sample_truth(kind, rng) -> truth`` (dict of name -> value);
    - ``predict(params, kind, postures, handle=None) -> (n, m)`` noise-free
      measurements; ``handle`` is the fit's handle (``None`` for the truth);
    - ``fit(train_df, level, estimation, workdir) -> FitResult``;
    - ``to_dataset(measurements, postures) -> DataFrame`` in the study's
      file layout;
    - ``simulate(study, truth, kind, postures, level, seed) ->
      (measurements, postures)`` replaces the default for studies whose
      noise is not additive on the measurement (encoder noise through IK);
    - ``strata(name) -> {stratum: bool mask}`` for a held-out set;
    - ``z_params[level]`` is the predicate on names the fit keeps;
    - ``train_residual(pred - meas)`` picks the components of the training
      RMS (default: all), e.g. positions only when the other columns are
      angles that wrap.
    """

    sample_truth: Callable[[str, np.random.Generator], Mapping[str, float]]
    predict: Callable[..., np.ndarray]
    fit: Callable[[pd.DataFrame, str, Optional[dict], Path], FitResult]
    to_dataset: Callable[[np.ndarray, pd.DataFrame], pd.DataFrame]
    train_postures: pd.DataFrame
    heldout_postures: Dict[str, pd.DataFrame]
    noise: NoiseModel = gaussian_mm
    simulate: Optional[Callable[..., Tuple[np.ndarray, pd.DataFrame]]] = None
    strata: Optional[Callable[[str], Dict[str, np.ndarray]]] = None
    z_params: Dict[str, Callable[[str], bool]] = field(default_factory=dict)
    row_error: Callable[[np.ndarray], np.ndarray] = _row_error
    train_residual: Callable[[np.ndarray], np.ndarray] = lambda e: e


def _simulate(study, truth, kind, postures, level, seed):
    if study.simulate is not None:
        return study.simulate(study, truth, kind, postures, level, seed)
    rng = np.random.default_rng(10_000 + seed)
    clean = study.predict(truth, kind, postures)
    return study.noise(clean, level, rng), postures


def run_case(
    study: SimulationStudy,
    truth_kind: str,
    fit_level: str,
    seed: int,
    noise: float,
    workdir,
    estimation: Optional[dict] = None,
    label: Optional[str] = None,
) -> dict:
    """Draw, simulate, fit, and judge one case."""
    truth = study.sample_truth(truth_kind, np.random.default_rng(seed))
    meas, post = _simulate(study, truth, truth_kind, study.train_postures, noise, seed)
    train = study.to_dataset(meas, post)
    case_dir = Path(workdir) / f"truth_{truth_kind}_{seed}_{noise}"
    case_dir.mkdir(parents=True, exist_ok=True)
    fit = study.fit(train, fit_level, estimation, case_dir)
    names = list(fit.params)

    result = {
        "truth": truth_kind,
        "fit": label or fit_level,
        "seed": seed,
        "noise_mm": noise,
        "n_params": len(names),
        "solver_status": int(fit.status),
        "nfev": int(fit.nfev),
        "absorbed": list(fit.absorbed),
    }
    pred = study.predict(fit.params, fit.level, post, fit.handle)
    res = study.train_residual(pred - meas)
    result["train_rms_mm"] = 1000 * float(np.sqrt(np.mean(res**2)))

    errors = []
    strata: Dict[str, list] = {}
    for name, q in study.heldout_postures.items():
        e = study.predict(fit.params, fit.level, q, fit.handle) - study.predict(
            truth, truth_kind, q
        )
        norm = study.row_error(e)
        errors.append(norm)
        if study.strata is not None:
            for k, m in study.strata(name).items():
                strata.setdefault(k, []).append(norm[m])
    errors = np.concatenate(errors)
    result["heldout_rmse_mm"] = float(np.sqrt(np.mean(errors**2)))
    result["heldout_max_mm"] = float(errors.max())
    result["strata_rmse_mm"] = {
        k: float(np.sqrt(np.mean(np.concatenate(v) ** 2))) for k, v in strata.items()
    }

    keep = study.z_params.get(truth_kind)
    if fit_level == truth_kind and keep is not None:
        result["z_scores"] = {
            n: float((fit.params[n] - truth[n]) / fit.std[n])
            for n in names
            if keep(n) and fit.std[n] > 0
        }
    return result


def run_grid(
    study: SimulationStudy,
    truths: Sequence[str],
    fits: Mapping[str, Tuple[str, Optional[dict]]],
    seeds: Sequence[int] = range(5),
    noise: Sequence[float] = (0.5, 2.0),
    workdir=None,
) -> pd.DataFrame:
    """Every truth x noise x fit x seed; ``fits`` is label -> (level, est)."""

    def go(tmp):
        return [
            run_case(study, t, level, s, n, tmp, estimation=est, label=label)
            for t in truths
            for n in noise
            for label, (level, est) in fits.items()
            for s in seeds
        ]

    if workdir is not None:
        return pd.DataFrame(go(workdir))
    with tempfile.TemporaryDirectory() as tmp:
        return pd.DataFrame(go(tmp))


def summarize(
    df: pd.DataFrame, strata: Optional[Mapping[str, str]] = None
) -> Tuple[pd.DataFrame, Optional[dict]]:
    """Per (truth, noise, fit) means, and the pooled z-score summary.

    ``strata`` maps an output column to a stratum key (default: every
    stratum, as ``<key>_mm``). The z summary is ``None`` without z-scores.
    """
    df = df.copy()
    if strata is None:
        keys = sorted({k for r in df.strata_rmse_mm for k in r})
        strata = {f"{k}_mm": k for k in keys}
    for col, key in strata.items():
        df[col] = [r[key] for r in df.strata_rmse_mm]
    table = df.groupby(["truth", "noise_mm", "fit"], sort=False).agg(
        n_params=("n_params", "median"),
        train_rms_mm=("train_rms_mm", "mean"),
        heldout_rmse_mm=("heldout_rmse_mm", "mean"),
        **{c: (c, "mean") for c in strata},
        heldout_max_mm=("heldout_max_mm", "max"),
    )
    z = [
        v
        for zs in df.get("z_scores", pd.Series(dtype=object)).dropna()
        for v in zs.values()
    ]
    if not z:
        return table, None
    z = np.array(z)
    return table, {
        "n": len(z),
        "mean": float(z.mean()),
        "rms": float(np.sqrt(np.mean(z**2))),
        "max_abs": float(np.abs(z).max()),
    }
