"""TIAGo Pro mocap: diagnostics of the train / held-out gap (#97).

DIAGNOSTICS ONLY. Nothing here selects a model or feeds the frozen
validation / confirmation selection of heldout_protocol.py; the numbers
explain why the held-out error (about 17-20 mm re-registered) is much larger
than the training RMS (about 6.7 mm).

- ``kfold``: k-fold within the training session 2026-08-05 (data/
  protocol_0805_kfold.yaml, k=5, seed 0; weaker than held-out sessions). Each
  model is fitted on k-1 folds and scored on the left-out fold with the
  training frames carried over (frames do not change within a session), in
  position and orientation. Decision rule, fixed before computing: if the
  selected model's (map_cv) k-fold position RMSE is at most 1.5 x its
  training RMS, overfitting is not the main cause of the gap, otherwise it
  is material.
- ``crossday``: fit on 07-01 (position only), evaluate on 07-02, one day
  apart. Separates marker / frame moves between days from robot changes
  across the month.
- ``splithalf``: re-registration on one random half of a held-out session
  (seed 0), scored on the other half and swapped, next to the in-sample
  re-registered RMSE: the in-sample bias of the primary metric.
- ``encoder``: which side of the series elastic transmission each session
  recorded. ``encoder_fit`` compares the recorded joints with the commanded
  plan postures (``optimal_configs.yaml``) and, on 2026-08-05, fits
  commanded - recorded = c + s * tau_gravity per joint (joint data and the
  nominal URDF only, no mocap). ``encoder_side`` then maps the July
  (motor-side) joints to the link side with that fit, and the 08-05 joints to
  the motor side, and scores the held-out sessions with the mocap.

Run from examples/tiago_pro:
python diagnostics.py [kfold|crossday|splithalf|encoder|all]
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.tiago_pro import heldout_protocol as hp  # noqa: E402

KFOLD_PROTOCOL = hp.DATA / "protocol_0805_kfold.yaml"
OVERFIT_RATIO = 1.5  # k-fold / training RMS above which overfitting is material
SELECTED = "full map_cv"
SEED = 0
SHARED_TOL = 2e-3  # rad: the 14 postures 07-01 and 07-02 share agree to 2e-4
PLAN_TOL = 1e-3  # rad: a recorded row "is" a plan posture below this
MIN_TORQUE = 0.5  # Nm: below this peak gravity torque only an offset is fitted
ENCODER_MODELS = ["registration only", "full_params", "full map_cv"]


def _rows_csv(path: Path, rows, tmp: Path, tag: str) -> Path:
    out = tmp / f"{tag}.csv"
    pd.read_csv(path).iloc[list(rows)].to_csv(out, index=False)
    return out


# ---------------------------------------------------------------- k-fold
def kfold(models=None, tmp: Path | None = None) -> dict:
    """``{model: {fold: {pos_rmse_mm, orient_rmse_deg, train_pos_rmse_mm}}}``."""
    plan = harness.load_plan(KFOLD_PROTOCOL)
    labels = list(hp.MODELS) if models is None else list(models)
    tmp = tmp or Path(tempfile.mkdtemp(prefix="tp_kfold_"))

    def build(label):
        def go(fold):
            (tr,) = fold.training
            csv = _rows_csv(tr.path, tr.rows, tmp, f"train_{tr.name}")
            level, frames_only, est = hp.MODELS[label]
            return hp.fit(level, frames_only, data_file=str(csv), estimation=est)

        return go

    def evaluate(calib, held):
        csv = _rows_csv(held.path, held.rows, tmp, f"held_{held.name}")
        with contextlib.redirect_stdout(io.StringIO()):
            calib._load_validation_data(str(csv))
            m = calib._compute_validation_metrics()
        overall = calib.evaluation_metrics["per_dof_stats"]["overall"]
        return {
            "n": m["n_val_samples"],
            "pos_rmse_mm": float(m["pos_rmse_calibrated_mm"]),
            "orient_rmse_deg": float(m["orient_rmse_calibrated_deg"]),
            "train_pos_rmse_mm": float(overall["pos_rmse_mm"]),
            "train_orient_rmse_deg": float(overall["orient_rmse_deg"]),
        }

    res = harness.run(plan, {m: build(m) for m in labels}, evaluate)
    return {"label": res.label, "results": {m: dict(res[m]) for m in labels}}


def kfold_table(res: dict, full_train: dict) -> tuple[str, dict]:
    """Markdown table and per-model summary; ``full_train`` is the training
    RMS (mm) of each model fitted on all 48 rows."""
    lines = [
        "| model | training RMS, all rows (mm) | training RMS, k-1 folds (mm) | "
        "held-out fold position RMSE (mm) | ratio to all-rows training RMS | "
        "held-out fold orientation RMSE (deg) |",
        "|---|---|---|---|---|---|",
    ]
    summary = {}
    for model, folds in res["results"].items():
        sets = [s for f in folds.values() for s in f.values()]
        n = np.array([s["n"] for s in sets], float)
        pos = float(
            np.sqrt(np.sum(n * [s["pos_rmse_mm"] ** 2 for s in sets]) / n.sum())
        )
        ori = float(
            np.sqrt(np.sum(n * [s["orient_rmse_deg"] ** 2 for s in sets]) / n.sum())
        )
        tr = float(np.mean([s["train_pos_rmse_mm"] for s in sets]))
        ratio = pos / full_train[model]
        summary[model] = {"pos": pos, "ori": ori, "train_folds": tr, "ratio": ratio}
        lines.append(
            f"| {model} | {full_train[model]:.2f} | {tr:.2f} | {pos:.2f} | "
            f"{ratio:.2f} | {ori:.2f} |"
        )
    return "\n".join(lines), summary


# ------------------------------------------------------------ cross-day
def _norm_mm(calib, x, q, measured):
    cfg = hp.position_config(calib, len(q))
    return np.linalg.norm(
        (hp.predict_positions(calib, x, q, cfg) - measured) * 1000, axis=1
    )


def _rms(v):
    return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float("nan")


def crossday(models=None) -> dict:
    """Fit on 07-01 (position only), evaluate on 07-02 (and 08-05 is not used)."""
    fit_file = hp.DATA / "calibration_samples_20260701_0913.csv"
    ev_file = hp.DATA / "calibration_samples_20260702_0756.csv"
    q_fit = pd.read_csv(fit_file)[hp.JOINTS].to_numpy()
    q_ev_df = pd.read_csv(ev_file)
    q_ev = q_ev_df[hp.JOINTS].to_numpy()
    dist = np.abs(q_ev[:, None, :] - q_fit[None]).max(axis=2).min(axis=1)
    shared = dist < SHARED_TOL
    out = {"n_fit": len(q_fit), "n_eval": len(q_ev), "n_shared": int(shared.sum())}
    for label in list(hp.MODELS) if models is None else list(models):
        level, frames_only, est = hp.MODELS[label]
        calib = hp.fit(
            level,
            frames_only,
            data_file=str(fit_file),
            estimation=est,
            position_only=True,
        )
        df = q_ev_df
        q = hp.session_postures(calib, df)
        meas = df[["x1", "y1", "z1"]].to_numpy()
        x0 = np.array(calib.LM_result.x, float)
        x_re = hp.reregister(calib, q, meas)
        tr = _norm_mm(calib, x0, q, meas)
        re = _norm_mm(calib, x_re, q, meas)
        out[label] = {
            "n_params": len(calib.calib_config["param_name"]),
            "train_pos_rmse_mm": float(
                calib.evaluation_metrics["per_dof_stats"]["overall"]["pos_rmse_mm"]
            ),
            "rereg": _rms(re),
            "transfer": _rms(tr),
            "rereg_shared": _rms(re[shared]),
            "transfer_shared": _rms(tr[shared]),
            "rereg_other": _rms(re[~shared]),
            "transfer_other": _rms(tr[~shared]),
        }
    return out


# ------------------------------------------------------------ split-half
def splithalf(models=None, seed: int = SEED) -> dict:
    """Re-register on one half of a held-out session, score the other half."""
    out = {}
    for label in list(hp.MODELS) if models is None else list(models):
        calib = hp.fit_model(label)
        res = {}
        for role, name in hp.SETS[1:]:
            df = pd.read_csv(hp.DATA / name)
            q = hp.session_postures(calib, df)
            meas = df[["x1", "y1", "z1"]].to_numpy()
            perm = np.random.default_rng(seed).permutation(len(df))
            halves = [np.sort(h) for h in np.array_split(perm, 2)]
            full = _rms(_norm_mm(calib, hp.reregister(calib, q, meas), q, meas))
            cross, within = [], []
            for a, b in (halves, halves[::-1]):
                x = hp.reregister(calib, q[a], meas[a])
                within.append(_rms(_norm_mm(calib, x, q[a], meas[a])))
                cross.append(_rms(_norm_mm(calib, x, q[b], meas[b])))
            res[name] = {
                "role": role,
                "n": len(df),
                "in_sample_all": full,
                "in_sample_half": float(np.mean(within)),
                "other_half_AtoB": cross[0],
                "other_half_BtoA": cross[1],
                "other_half": _rms(
                    np.concatenate(
                        [[cross[0]] * len(halves[1]), [cross[1]] * len(halves[0])]
                    )
                ),
            }
        out[label] = res
    return out


# ---------------------------------------------------------- encoder side
def _plan() -> tuple[list, np.ndarray]:
    plan = yaml.safe_load((hp.DATA / "optimal_configs.yaml").read_text())
    return plan["calibration_joint_names"], np.array(
        plan["calibration_joint_configurations"]
    )


def gravity_torque(q: np.ndarray, joints: list) -> np.ndarray:
    """(n, len(joints)) gravity torque of the nominal URDF (no payload)."""
    model = pin.buildModelFromUrdf(str(hp.URDF))
    data = model.createData()
    ids = [model.getJointId(n) for n in joints]
    out = np.empty((len(q), len(joints)))
    for r, row in enumerate(q):
        x = pin.neutral(model)
        for j, v in zip(ids, row):
            x[model.joints[j].idx_q] = v
        g = pin.computeGeneralizedGravity(model, data, x)
        out[r] = [g[model.joints[j].idx_v] for j in ids]
    return out


def encoder_fit() -> dict:
    """Recorded against commanded joints; the 08-05 offset + compliance fit.

    ``sessions[file]``: rows, rows within PLAN_TOL of a plan posture and the
    median gap to the nearest one. ``joints[name]``: offset ``c`` (rad),
    compliance ``s`` (rad/Nm), correlation of commanded - recorded with the
    gravity torque, residual RMS (rad) and peak torque (Nm), on 08-05.
    """
    joints, plan = _plan()
    sessions = {}
    for _, name in hp.SETS:
        q = pd.read_csv(hp.DATA / name)[joints].to_numpy()
        gap = np.abs(q[:, None, :] - plan[None]).max(axis=2)
        best = gap.min(axis=1)
        sessions[name] = {
            "n": len(q),
            "on_plan": int((best < PLAN_TOL).sum()),
            "on_plan_max_gap": (
                float(best[best < PLAN_TOL].max())
                if (best < PLAN_TOL).any()
                else float("nan")
            ),
            "median_gap": float(np.median(best)),
        }
    q = pd.read_csv(hp.DATA / hp.TRAINING)[joints].to_numpy()
    cmd = plan[np.abs(q[:, None, :] - plan[None]).max(axis=2).argmin(axis=1)]
    d = cmd - q
    tau = gravity_torque(q, joints)
    fit = {}
    for i, n in enumerate(joints):
        if n == "torso_lift_joint":
            continue  # prismatic, not elastic
        if np.abs(tau[:, i]).max() < MIN_TORQUE:
            c, s, r = float(d[:, i].mean()), 0.0, float("nan")
        else:
            a = np.c_[np.ones(len(q)), tau[:, i]]
            c, s = np.linalg.lstsq(a, d[:, i], rcond=None)[0]
            r = float(np.corrcoef(d[:, i], tau[:, i])[0, 1])
        res = d[:, i] - c - s * tau[:, i]
        fit[n] = {
            "c": float(c),
            "s": float(s),
            "corr": r,
            "resid_rms": float(np.sqrt(np.mean(res**2))),
            "tau_max": float(np.abs(tau[:, i]).max()),
        }
    return {"sessions": sessions, "joints": fit}


def to_side(path: Path, fit: dict, side: str, out: Path, parts="both") -> Path:
    """Copy of a session with its joints moved to ``side`` ("link": motor -
    c - s*tau, "motor": link + c + s*tau). ``parts`` keeps "offset",
    "elastic" or "both" terms of the correction."""
    df = pd.read_csv(path)
    names = list(fit)
    q = df[names].to_numpy()
    tau = gravity_torque(q, names)
    c = np.array([fit[n]["c"] for n in names])
    s = np.array([fit[n]["s"] for n in names])
    corr = (c if parts != "elastic" else 0) + (s * tau if parts != "offset" else 0)
    df[names] = q + (corr if side == "motor" else -corr)
    dest = out / f"{path.stem}_{side}_{parts}.csv"
    df.to_csv(dest, index=False)
    return dest


def encoder_side(models=None, tmp: Path | None = None) -> dict:
    """Held-out re-registered RMSE (mm) with the encoder-side correction.

    ``mapped[model][file][parts]``: trained on 08-05 as recorded (link side),
    July joints as recorded ("raw") or mapped to the link side. ``motor``:
    trained on 08-05 mapped to the motor side, July as recorded.
    """
    fit = encoder_fit()["joints"]
    tmp = tmp or Path(tempfile.mkdtemp(prefix="tp_encoder_"))
    july = [hp.DATA / name for _, name in hp.SETS[1:]]
    out = {"mapped": {}, "motor": {}}
    for label in ENCODER_MODELS if models is None else list(models):
        calib = hp.fit_model(label)
        out["mapped"][label] = {
            f.name: {
                "raw": hp.component_errors(calib, f)["rmse_rereg"],
                **{
                    p: hp.component_errors(calib, to_side(f, fit, "link", tmp, p))[
                        "rmse_rereg"
                    ]
                    for p in ("offset", "elastic", "both")
                },
            }
            for f in july
        }
        level, frames_only, est = hp.MODELS[label]
        motor = to_side(hp.DATA / hp.TRAINING, fit, "motor", tmp)
        calib = hp.fit(level, frames_only, data_file=str(motor), estimation=est)
        out["motor"][label] = {
            "train_pos_rmse_mm": float(
                calib.evaluation_metrics["per_dof_stats"]["overall"]["pos_rmse_mm"]
            ),
            **{f.name: hp.component_errors(calib, f)["rmse_rereg"] for f in july},
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "what",
        nargs="?",
        default="all",
        choices=["kfold", "crossday", "splithalf", "encoder", "all"],
    )
    ap.add_argument("--models", nargs="*", default=None)
    ap.add_argument("--json", type=Path, default=None, help="write results here")
    a = ap.parse_args()
    out = {}
    if a.what in ("kfold", "all"):
        res = kfold(a.models)
        full = {
            m: hp.fit_model(m).evaluation_metrics["per_dof_stats"]["overall"][
                "pos_rmse_mm"
            ]
            for m in res["results"]
        }
        table, summ = kfold_table(res, full)
        print(f"\n{res['label']}\n{table}")
        if SELECTED in summ:
            r = summ[SELECTED]["ratio"]
            print(
                f"\nselected {SELECTED}: ratio {r:.2f} -> overfitting "
                + ("MATERIAL" if r > OVERFIT_RATIO else "not the main cause")
                + f" (threshold {OVERFIT_RATIO})"
            )
        out["kfold"] = {"raw": res, "summary": summ, "full_train": full}
    if a.what in ("crossday", "all"):
        out["crossday"] = crossday(a.models)
        print("\ncross-day 07-01 -> 07-02:", json.dumps(out["crossday"], indent=1))
    if a.what in ("splithalf", "all"):
        out["splithalf"] = splithalf(a.models)
        print("\nsplit-half:", json.dumps(out["splithalf"], indent=1))
    if a.what in ("encoder", "all"):
        out["encoder"] = {"fit": encoder_fit(), "side": encoder_side(a.models)}
        print("\nencoder side:", json.dumps(out["encoder"], indent=1))
    if a.json:
        a.json.write_text(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
