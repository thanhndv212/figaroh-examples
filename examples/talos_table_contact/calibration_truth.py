#!/usr/bin/env python3
"""TALOS table-contact: synthetic-truth study (figaroh-examples#98).

Draw a truth around the table anchored at the real table-only fit (15 chain
joints with 6D placement errors, plus table and contact draws), put the
real training postures onto the true table by IK, add encoder noise to the
joint angles, and fit with each method of ``heldout_protocol.MODELS``. This
is a ``calibration_harness.simulation.SimulationStudy`` per chain.

Judged in prediction space only. A flush-contact gap is gauge-bound (the leg
translations along z trade against the plane height, the contact against the
plane), so per-parameter z-scores are not meaningful and ``z_params`` is
empty. The score is the error of the fitted gap against the true gap at the
9 real validation postures (fitted gap minus true gap, gauge-invariant),
reported as the height error in mm. Those postures are not exactly on the
true table, which is why the *difference* of the two predictions is scored,
not the gap itself.

Surfaces: ``planar`` (the model class) and ``curved`` (a bowl, 1 mm sag at
the 90th-percentile contact radius). A constant table tilt is not a
mismatch, the plane roll/pitch parameters absorb it.

``train_rms_mm`` is the harness's: RMS over the gap residual components
(height in m and roll/pitch in rad, all times 1000), at the noisy postures.

Run from this directory::

    python calibration_truth.py [--chains left right] [--seeds 0 1 2 3 4]
        [--noise-deg 0.01 0.05 0.1] [--methods ...] [--mismatch]
        [--jobs 4] [--json] [--out summary.txt]
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
project_root = HERE.parents[1]
for p in (str(project_root), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

from examples.calibration_harness import simulation as sim  # noqa: E402
from generate_synthetic_data import (  # noqa: E402
    CHAINS,
    _load_robot,
    build_anchored_truth,
    true_gap,
)
from heldout_protocol import MODELS  # noqa: E402
from run_calibration_real_data import (  # noqa: E402
    _CANONICAL_JOINT_ORDER,
    _build_chain,
    _load_real_touches,
    _load_split,
)

#: encoder noise grid (deg, one-sigma per active joint)
NOISE_DEG = (0.01, 0.05, 0.1)
SEEDS = tuple(range(5))
#: the curved mismatch runs at this noise only
MISMATCH_NOISE_DEG = 0.05
#: noise-free planar check of the machinery (structural held-out < 0.05 mm)
MECHANISM_TOL_MM = 0.05

# truth scales of ``build_ground_truth`` (1.5 mm / 0.3 deg joint placement
# errors) as the priors of ``map``; the other methods keep core's defaults
_TRUTH_PRIORS = {
    "translation": 1.5e-3,
    "rotation": float(np.deg2rad(0.3)),
    "joint_offset": float(np.deg2rad(0.3)),
    "prismatic_offset": 1.5e-3,
}
#: fit-count cap of cv_subset (about 6 sizes instead of every subset size)
_CV_SIZES = [0, 10, 20, 30, 40, 50, 57]

#: label -> (fit level or "structural" for the config's, estimation)
FITS = {}
for _label, (_level, _est) in MODELS.items():
    _est = dict(_est) if _est else None
    if _label == "map":
        _est["priors"] = dict(_TRUTH_PRIORS)
    if _label == "cv_subset":
        _est["cv_sizes"] = list(_CV_SIZES)
    FITS[_label] = (_level or "structural", _est)

HELDOUT = "validation"
_SEED_LABEL = "noise_deg"


def _quiet():
    return contextlib.redirect_stdout(io.StringIO())


def _seed_of(rng: np.random.Generator) -> int:
    """The integer ``default_rng(seed)`` was built from (the harness hands
    ``sample_truth`` only the generator)."""
    return int(rng.bit_generator.seed_seq.entropy)


def _frame(q: np.ndarray, model) -> pd.DataFrame:
    """Canonical joint columns of full posture vectors ``q`` (n, nq)."""
    out = {}
    for name in _CANONICAL_JOINT_ORDER:
        out[name] = q[:, model.joints[model.getJointId(name)].idx_q]
    return pd.DataFrame(out)


def _full(df: pd.DataFrame, model, neutral) -> np.ndarray:
    q = np.tile(neutral, (len(df), 1))
    for name in _CANONICAL_JOINT_ORDER:
        q[:, model.joints[model.getJointId(name)].idx_q] = df[name].to_numpy()
    return q


def _write_raw(df: pd.DataFrame, path: Path) -> Path:
    """``df`` in the raw row layout the real files use (5 metadata fields,
    32 joints in canonical order), for ``_load_real_touches``."""
    rows = pd.DataFrame(
        {
            "side": "synthetic",
            "frame": "synthetic",
            "kind": "handle",
            "marker": [f"m{i}" for i in range(len(df))],
            "src": "joint_states",
        }
    )
    rows = pd.concat([rows, df[_CANONICAL_JOINT_ORDER].reset_index(drop=True)], axis=1)
    rows.to_csv(path, index=False)
    return path


def make_study(side: str, robot=None) -> sim.SimulationStudy:
    """The study of one chain (``left`` or ``right``)."""
    import pinocchio as pin

    c = CHAINS[side]
    robot = robot or _load_robot()
    model = robot.model
    neutral = pin.neutral(model)
    active = list(c["joints"])
    real_val = _load_real_touches(c["val"])[_CANONICAL_JOINT_ORDER]
    real_train = _load_real_touches(c["train"])[_CANONICAL_JOINT_ORDER]
    cache = {}  # (kind, seed) -> anchored truth and clean IK postures

    def sample_truth(kind, rng):
        seed = _seed_of(rng)
        key = (kind, seed)
        if key not in cache:
            with _quiet():
                cache[key] = build_anchored_truth(robot, side, seed, surface=kind)
        return cache[key]

    def simulate(study, truth, kind, postures, level, seed):
        clean = _frame(truth["q_train"], model)
        rng = np.random.default_rng(10_000 + seed)
        noise = rng.normal(0.0, np.deg2rad(level), (len(clean), len(active)))
        noisy = clean.copy()
        noisy[active] = clean[active].to_numpy() + noise
        return np.zeros((len(clean), 3)), noisy

    def predict(params, kind, postures, handle=None):
        q = postures if isinstance(postures, pd.DataFrame) else None
        if handle is None:  # the truth
            return true_gap(
                params["truth"],
                _full(q, model, neutral),
                params["surface"],
                c["base"],
                c["wrist"],
            )
        calib, x = handle
        _load_split(calib, q)
        n = len(q)
        return np.asarray(calib.cost_function(x)[: 3 * n]).reshape(3, n).T

    def to_dataset(measurements, postures):
        return postures

    def fit(train_df, level, estimation, workdir):
        csv = _write_raw(train_df, Path(workdir) / f"train_{side}.csv")
        with _quiet():
            calib = _build_chain(
                robot,
                c["config"],
                csv,
                asset_id=f"talos-{side}-sim",
                estimation=estimation,
                fit_level=None if level == "structural" else level,
            )
            result = calib.solve(
                method="lm",
                max_iterations=1 if level == "table_contact" else 3,
                enable_logging=False,
                html_report=False,
            )
        names = list(calib.calib_config["param_name"])
        x = np.asarray(result.x, dtype=float)
        lm = getattr(calib, "LM_result", None)
        return sim.FitResult(
            params=dict(zip(names, x.tolist())),
            std={n: 0.0 for n in names},
            level=level,
            status=int(getattr(lm, "status", 0)),
            nfev=int(getattr(lm, "nfev", 0)),
            handle=(calib, x),
        )

    return sim.SimulationStudy(
        sample_truth=sample_truth,
        predict=predict,
        fit=fit,
        to_dataset=to_dataset,
        train_postures=real_train,
        heldout_postures={HELDOUT: real_val},
        simulate=simulate,
        # per-parameter z-scores are not meaningful: the plane_z / contact
        # and leg-z gauge makes individual parameters unrecoverable, so the
        # fit is judged by its (gauge-invariant) gap prediction only
        z_params={},
        row_error=lambda e: 1000.0 * np.abs(e[:, 0]),
    )


def run_case(study, side, kind, label, seed, noise_deg, workdir) -> dict:
    """One case as a row; ``noise_deg`` replaces the harness's ``noise_mm``."""
    level, est = FITS[label]
    row = sim.run_case(
        study, kind, level, seed, noise_deg, workdir, estimation=est, label=label
    )
    row[_SEED_LABEL] = row.pop("noise_mm")
    row["chain"] = side
    row.pop("strata_rmse_mm", None)
    return row


# --------------------------------------------------------------------------
# Process pool: one robot load per worker
# --------------------------------------------------------------------------
_W = {}


def _init_worker():
    _W["robot"] = _load_robot()
    _W["studies"] = {}


def _task(args):
    import tempfile

    side, kind, seed, noise_deg, labels = args
    studies = _W["studies"]
    if side not in studies:
        studies[side] = make_study(side, _W["robot"])
    out = []
    with tempfile.TemporaryDirectory() as tmp:
        for label in labels:
            t0 = time.time()
            row = run_case(studies[side], side, kind, label, seed, noise_deg, tmp)
            row["seconds"] = time.time() - t0
            out.append(row)
    return out


def tasks(chains, seeds, noise_deg, labels, mismatch=False, mechanism=True):
    """Work units ``(chain, surface, seed, noise_deg, labels)``."""
    out = [
        (c, "planar", s, n, labels) for c in chains for s in seeds for n in noise_deg
    ]
    if mismatch:
        out += [
            (c, "curved", s, MISMATCH_NOISE_DEG, labels) for c in chains for s in seeds
        ]
    if mechanism:
        out += [(c, "planar", 0, 0.0, ["structural"]) for c in chains]
    return out


def run_tasks(units, jobs=1) -> pd.DataFrame:
    if jobs <= 1:
        _init_worker()
        rows = [r for u in units for r in _task(u)]
    else:
        with ProcessPoolExecutor(jobs, initializer=_init_worker) as pool:
            rows = [r for part in pool.map(_task, units) for r in part]
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------
def summary(df: pd.DataFrame) -> pd.DataFrame:
    """Median and min-max over seeds per chain, surface, noise and method."""
    g = df.groupby(["chain", "truth", _SEED_LABEL, "fit"], sort=False)
    out = g.agg(
        n=("seed", "count"),
        n_params=("n_params", "median"),
        heldout_med=("heldout_rmse_mm", "median"),
        heldout_min=("heldout_rmse_mm", "min"),
        heldout_max=("heldout_rmse_mm", "max"),
        train_rms_med=("train_rms_mm", "median"),
        train_rms_min=("train_rms_mm", "min"),
        train_rms_max=("train_rms_mm", "max"),
    )
    return out.reset_index()


def format_summary(df: pd.DataFrame) -> str:
    s = summary(df)
    lines = [
        "held-out height error (mm, median [min-max] over seeds) at the real "
        "validation postures; train RMS is the harness's",
        f"{'chain':5s} {'truth':7s} {'noise_deg':>9s} {'method':20s} {'n':>2s} "
        f"{'par':>4s} {'held-out mm':>22s} {'train RMS':>22s}",
    ]
    for r in s.itertuples():
        lines.append(
            f"{r.chain:5s} {r.truth:7s} {getattr(r, _SEED_LABEL):9.2f} "
            f"{r.fit:20s} {r.n:2d} {r.n_params:4.0f} "
            f"{r.heldout_med:7.3f} [{r.heldout_min:6.3f}-{r.heldout_max:6.3f}] "
            f"{r.train_rms_med:7.3f} [{r.train_rms_min:6.3f}-{r.train_rms_max:6.3f}]"
        )
    return "\n".join(lines)


def golden_block(df: pd.DataFrame) -> dict:
    """Seed 0 rows: the golden's ``synthetic`` block."""
    out = {"noise_deg": [], "chains": {}}
    for r in df[df.seed == 0].itertuples():
        key = f"{r.truth}@{getattr(r, _SEED_LABEL)}"
        out["chains"].setdefault(r.chain, {}).setdefault(key, {})[r.fit] = {
            "n_params": int(r.n_params),
            "heldout_rmse_mm": round(float(r.heldout_rmse_mm), 3),
            "train_rms_mm": round(float(r.train_rms_mm), 3),
        }
    out["noise_deg"] = sorted(set(df[_SEED_LABEL]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--chains", nargs="+", choices=list(CHAINS), default=list(CHAINS))
    ap.add_argument("--seeds", nargs="+", type=int, default=list(SEEDS))
    ap.add_argument("--noise-deg", nargs="+", type=float, default=list(NOISE_DEG))
    ap.add_argument("--methods", nargs="+", choices=list(FITS), default=list(FITS))
    ap.add_argument(
        "--mismatch", action="store_true", help="add the curved-table cases"
    )
    ap.add_argument(
        "--no-mechanism", action="store_true", help="skip the noise-free check"
    )
    ap.add_argument("--jobs", type=int, default=1, help="worker processes")
    ap.add_argument("--json", action="store_true", help="print the golden block")
    ap.add_argument("--out", type=Path, default=None, help="summary text path")
    args = ap.parse_args()

    t0 = time.time()
    units = tasks(
        args.chains,
        args.seeds,
        args.noise_deg,
        args.methods,
        args.mismatch,
        not args.no_mechanism,
    )
    df = run_tasks(units, args.jobs)
    text = format_summary(df)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
        df.to_json(args.out.with_suffix(".rows.json"), orient="records", indent=1)
    if args.json:
        print(json.dumps(golden_block(df), indent=2, sort_keys=True))
        return
    print(text)
    mech = df[df[_SEED_LABEL] == 0.0]
    for r in mech.itertuples():
        print(
            f"mechanism check {r.chain} {r.fit}: held-out {r.heldout_rmse_mm:.4f} mm "
            f"(tol {MECHANISM_TOL_MM})"
        )
    print(f"\nTotal runtime {time.time() - t0:.0f} s, {len(df)} cases")


if __name__ == "__main__":
    main()
