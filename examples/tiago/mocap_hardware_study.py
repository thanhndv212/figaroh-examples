"""TIAGo mocap across hardware (#100): the study stages (``real``, ``square``).

The model, the metrics and the thresholds are the pre-registered ones
(``mocap_hardware.py``, ``docs/development/tiago-mocap-hardware.md``, bands
frozen in ``tests/golden/tiago_mocap_hardware_truth.json``). Nothing here is
selected or tuned on any session; descriptive fits of single sessions are
allowed because the model is fixed.

``real``: fits M_Q (Qualisys 2021-11-30 training), M_OT (OptiTrack 1658) and
M_V (Vicon 1508), the descriptive per-session fits (Qualisys 2021-10-26 1051
and 1102, OptiTrack 1552, Vicon 1607) and the 6D secondary fits of M_OT and M_V;
scores every model on every session with at least ``MIN_POSTURES`` postures by
leave-one-out (LOO) re-registration (primary) and in-sample (secondary); and
reads C1, C2, C3 and C5, the arm_5-only diagnostic and the 6D secondary against
the frozen bands.

``square``: registered path RMSE of each model on the seven square-motion runs
(C4) with the +-0.05 s lag sweep.

Every fit starts from a fresh calibration object and the same deterministic
initial frames (a registration of the nominal joints on the session), so a fit
does not depend on the fits that ran before it in the process.
"""

from __future__ import annotations

import html
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml
from scipy.optimize import least_squares

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.data import Protocol  # noqa: E402

from examples.tiago import mocap_hardware as mh  # noqa: E402
from examples.tiago.utils import square_motion_extraction as sme  # noqa: E402

FROZEN = project_root / "tests/golden/tiago_mocap_hardware_truth.json"
SQUARE = mh.TIAGO / "data/validation/square_motion"
# the trained models: name -> session of the fit; the first three are the
# models of the study, the others are descriptive per-session fits
MODELS = {
    "M_Q": "qualisys/2021-11-30-1544",
    "M_OT": "optitrack/2023-09-12-1658",
    "M_V": "vicon/2023-09-20-1508",
}
DESCRIPTIVE = {
    "Q1051": "qualisys/2021-10-26-1051",
    "Q1102": "qualisys/2021-10-26-1102",
    "OT1552": "optitrack/2023-09-12-1552",
    "V1607": "vicon/2023-09-20-1607",
}
SYSTEM_OF = {"M_Q": "qualisys", "M_OT": "optitrack", "M_V": "vicon"}
ARM5 = mh.JOINTS.index("arm_5_joint")
BORDERLINE = 0.35  # five seeds: a band is uncertain by about 35 %
SQUARE_LAGS_S = (-0.05, 0.05)
# positive control (C5): encoder difference of arm_6, first minus second session
C5_ARM6_MRAD = 19.3

# Recovery z of the truth fixture at the used level (5 seeds, rms of
# (fit - truth) / standard error over the comparable parameters), split by
# fit mode: output of mocap_hardware_truth.py --noise 2.0. A z rms near 1
# means honest standard errors; 6D is understated by about 1.5.
FIXTURE_RECOVERY_Z = {
    "3d": {"n": 75, "rms": 1.007},
    "6d": {"n": 50, "rms": 1.538},
    "all": {"n": 125, "rms": 1.247},
}
SESSIONS_6D = ("optitrack", "vicon")


def classify(ratio: float, borderline: float = BORDERLINE) -> str:
    """``beyond`` / ``borderline`` / ``within`` for ``value / band``."""
    if ratio > 1.0 + borderline:
        return "beyond"
    if ratio >= 1.0 - borderline:
        return "borderline"
    return "within"


def verdict_text(classes: List[str]) -> str:
    """One verdict for a comparison from the class of each of its items.

    Any item beyond its band: not explained by hardware alone; else any
    borderline: borderline; else consistent with hardware alone (absence of
    evidence of another cause, not proof).
    """
    if "beyond" in classes:
        return "not explained by measurement hardware alone"
    if "borderline" in classes:
        return "borderline"
    return "consistent with measurement hardware alone"


def frozen_bands() -> dict:
    return json.loads(FROZEN.read_text())["preregistered"]


def _f(x) -> Optional[float]:
    x = float(x)
    return x if np.isfinite(x) else None


# --- fits ---------------------------------------------------------------------


def fit_record(session: mh.Session, six_d: bool = False):
    """Structural fit of the fixed model on one session: ``(record, calib)``."""
    t0 = time.time()
    params, std, calib = mh.fit_session(session.path, six_d=six_d)
    metrics = calib.evaluation_metrics
    names = list(calib.calib_config["param_name"])
    rec = {
        "session": f"{session.system}/{session.id}",
        "mode": "6d" if six_d else "3d",
        "names": names,
        "params": {n: float(params[n]) for n in names},
        "std": {n: _f(std[n]) for n in names},
        "offsets": mh.offsets_of(params).tolist(),
        "absorbed": list(calib.calib_config.get("absorbed_param_name", [])),
        "status": int(calib.LM_result.status),
        "condition": _f(metrics.get("condition_number", np.nan)),
        "core_rmse": _f(metrics.get("rmse", np.nan)),
        "n_outliers": int(metrics.get("n_outliers", 0) or 0),
        "n": int(calib.calib_config["NbSample"]),
        "seconds": round(time.time() - t0, 1),
    }
    return rec, calib


def fit_all(sessions: dict, reverse: bool = False):
    """Every fit of the real stage, each from a fresh calibration object.

    ``reverse`` runs them in the opposite order (determinism check). Returns
    ``(fits, calibs)``: records by name (``M_OT|6d`` for the secondary fits).
    """
    plan = [(n, k, False) for n, k in {**MODELS, **DESCRIPTIVE}.items()]
    plan += [("M_OT", MODELS["M_OT"], True), ("M_V", MODELS["M_V"], True)]
    if reverse:
        plan = plan[::-1]
    fits, calibs = {}, {}
    for name, key, six_d in plan:
        rec, calib = fit_record(sessions[key], six_d)
        label = f"{name}|{rec['mode']}"
        fits[label] = rec
        calibs[label] = calib
        print(f"  fit {label:10s} {key:28s} {rec['seconds']:5.1f} s", flush=True)
    names = {tuple(sorted(r["params"])) for r in fits.values() if r["mode"] == "3d"}
    if len(names) != 1:
        raise RuntimeError(
            "structural selection kept different parameter sets: "
            + str(
                {k: sorted(r["params"]) for k, r in fits.items() if r["mode"] == "3d"}
            )
        )
    bad = {k: r["status"] for k, r in fits.items() if r["status"] <= 0}
    if bad:
        raise RuntimeError(f"fit did not converge: {bad}")
    return fits, calibs


# --- scoring ------------------------------------------------------------------


def scored_sessions(sessions: dict) -> Dict[str, mh.Data]:
    """Data of every session with at least ``MIN_POSTURES`` postures."""
    out = {}
    for key, s in sessions.items():
        data = mh.read_data(s.path, six_d=s.six_d)
        if len(data) >= mh.MIN_POSTURES:
            out[key] = data
    return out


def score_matrix(chain: mh.Chain, fits: dict, data: Dict[str, mh.Data]) -> dict:
    """LOO and in-sample scores, ``{model|session|mode: score}``.

    Models are the three systems' models (``qualisys`` ... ``vicon``, with the
    6D fit of the 2023 systems in 6D mode and M_Q otherwise), ``registration
    only`` (zero offsets) and the descriptive per-session fits.
    """
    off3 = {s: np.array(fits[f"{m}|3d"]["offsets"]) for m, s in SYSTEM_OF.items()}
    off6 = {
        s: np.array(fits[f"{m}|6d"]["offsets"])
        for m, s in SYSTEM_OF.items()
        if f"{m}|6d" in fits
    }
    models3 = {**off3, "registration only": np.zeros(8)}
    models3.update({n: np.array(fits[f"{n}|3d"]["offsets"]) for n in DESCRIPTIVE})
    matrix = {}
    for key, d in data.items():
        d3 = mh.Data(d.q, d.pos)
        for name, off in models3.items():
            matrix[f"{name}|{key}|3d"] = mh.score(chain, d3, off)
        if d.rot is not None:
            for name in (*off3, "registration only"):
                off = off6.get(name, models3[name])
                matrix[f"{name}|{key}|6d"] = mh.score(chain, d, off)
    return matrix


def refit_arm5(chain, q, meas, offsets, x0=None):
    """Frames (9) and the arm_5 offset refitted on a set of postures.

    Everything else of the trained offsets is kept. Returns ``(x (9,), theta5)``.
    """
    offsets = np.array(offsets, dtype=float)
    if x0 is None:
        R, p = chain.tool_pose(q, offsets)
        x0 = np.r_[mh.reregister(R, p, meas), offsets[ARM5]]

    def pose(theta5):
        off = offsets.copy()
        off[ARM5] = theta5
        return chain.frame_pose(q, off, sme.TOOL_FRAME)

    def residual(x):
        R, p = pose(x[9])
        pos, _ = mh._predict(R, p, x[:9], False)
        return (pos - meas).ravel()

    x = least_squares(residual, x0, method="lm").x
    return x[:9], float(x[9]), x


def arm5_diagnostic(chain, fits, data) -> dict:
    """M_Q with only arm_5 (and the frames) refitted on each 2023 session.

    A known ~90 mrad shift of arm_5 separates the 2021 and 2023 fits; this
    asks how much of the transfer gap one offset closes. It does not separate
    date from hardware.
    """
    base = np.array(fits["M_Q|3d"]["offsets"])
    out = {}
    for key, d in data.items():
        if key.split("/")[0] not in SESSIONS_6D:
            continue
        q, meas = d.q, d.pos
        R, p = chain.tool_pose(q, base)
        _, theta5, x_full = refit_arm5(chain, q, meas, base)
        err = np.empty((len(q), 3))
        for i in range(len(q)):
            keep = np.arange(len(q)) != i
            xi, t5, _ = refit_arm5(chain, q[keep], meas[keep], base, x0=x_full)
            off = base.copy()
            off[ARM5] = t5
            Ri, pi = chain.frame_pose(q[i : i + 1], off, sme.TOOL_FRAME)
            pos, _ = mh._predict(Ri, pi, xi, False)
            err[i] = pos[0] - meas[i]
        out[key] = {
            "arm_5_offset_mrad": 1e3 * theta5,
            "arm_5_offset_M_Q_mrad": 1e3 * base[ARM5],
            "loo_mm": mh.rms_mm(err),
        }
    return out


# --- residual-step diagnostic (post hoc) ---------------------------------------

STEP_MIN_SEGMENT = 3  # postures on each side of a split


def best_split(r: np.ndarray, min_segment: int = STEP_MIN_SEGMENT) -> int:
    """Index ``k`` (rows before the split) minimising the within-segment SSE of ``r``."""
    n = len(r)
    ks = range(min_segment, n - min_segment + 1)
    sse = [
        np.sum((r[:k] - r[:k].mean()) ** 2) + np.sum((r[k:] - r[k:].mean()) ** 2)
        for k in ks
    ]
    return list(ks)[int(np.argmin(sse))]


def split_arm5_refit(chain, q, meas, offsets, k, x0):
    """Frames and one arm_5 offset per side of the split, others kept.

    Returns ``(rmse_mm, jump_mrad)`` with the jump after minus before.
    """
    offsets = np.array(offsets, dtype=float)

    def poses(x):
        out = []
        for sl, th in ((slice(0, k), x[9]), (slice(k, None), x[10])):
            off = offsets.copy()
            off[ARM5] = th
            out.append(chain.frame_pose(q[sl], off, sme.TOOL_FRAME))
        return out

    def residual(x):
        (Ra, pa), (Rb, pb) = poses(x)
        R, p = np.concatenate([Ra, Rb]), np.concatenate([pa, pb])
        pos, _ = mh._predict(R, p, x[:9], False)
        return (pos - meas).ravel()

    x = least_squares(residual, np.r_[x0, offsets[ARM5], offsets[ARM5]], method="lm").x
    err = residual(x).reshape(-1, 3)
    return mh.rms_mm(err), 1e3 * float(x[10] - x[9])


def step_diagnostic(chain, fits, data) -> dict:
    """Per-posture residual norms of each session's own fit, in recording order.

    Post hoc and descriptive: no pre-registration, no verdict, nothing is
    selected. Rows of the shipped CSVs are in recording time (``t_start_robot``
    increases). Reports the best single split of the residual norm and, for
    that split, an arm_5 offset refitted separately on each side.
    """
    out = {}
    for name, key in {**MODELS, **DESCRIPTIVE}.items():
        d = data[key]
        off = np.array(fits[f"{name}|3d"]["offsets"])
        R, p = chain.tool_pose(d.q, off)
        x = mh.reregister(R, p, d.pos)
        err, _ = mh.residuals(R, p, x, d.pos)
        r = 1000.0 * np.linalg.norm(err, axis=1)
        k = best_split(r)
        rmse, jump = split_arm5_refit(chain, d.q, d.pos, off, k, x)
        out[key] = {
            "model": name,
            "n": len(r),
            "split": k,
            "before_mm": float(r[:k].mean()),
            "after_mm": float(r[k:].mean()),
            "rmse_mm": mh.rms_mm(err),
            "rmse_split_arm5_mm": rmse,
            "arm_5_jump_mrad": jump,
        }
    return out


# --- comparisons -------------------------------------------------------------


def transfer_rows(matrix: dict, bands: dict, mode: str) -> dict:
    """Transfer gaps G = e(M_A, E) - e(M_own, E) against their frozen bands."""
    metric = "loo_mm" if mode == "3d" else "loo_6d_mm"
    rows = {}
    for key, b in bands["transfer_band_mm"].items():
        pair, m = key.split("|")
        if m != mode:
            continue
        model, session = pair.split("->")
        own = session.split("/")[0]
        a = matrix[f"{model}|{session}|{mode}"][metric]
        o = matrix[f"{own}|{session}|{mode}"][metric]
        ratio = abs(a - o) / b["band"]
        rows[key] = {
            "model_loo_mm": a,
            "own_loo_mm": o,
            "gap_mm": a - o,
            "band_mm": b["band"],
            "ratio": ratio,
            "class": classify(ratio),
        }
    return rows


def parameter_rows(fits: dict, bands: dict, mode: str) -> dict:
    """Parameter differences theta_A - theta_B (mrad) against their frozen bands."""
    rows = {}
    for key, per in bands["parameter_band_rad"].items():
        pair, m = key.split("|")
        if m != mode:
            continue
        a, b = pair.split("-")
        ka = f"M_{ {'qualisys': 'Q', 'optitrack': 'OT', 'vicon': 'V'}[a] }|{mode}"
        kb = f"M_{ {'qualisys': 'Q', 'optitrack': 'OT', 'vicon': 'V'}[b] }|{mode}"
        ka = ka if ka in fits else ka.replace("6d", "3d")
        kb = kb if kb in fits else kb.replace("6d", "3d")
        for j in mh.COMPARABLE:
            d = fits[ka]["params"][j] - fits[kb]["params"][j]
            se = float(np.hypot(fits[ka]["std"][j] or 0.0, fits[kb]["std"][j] or 0.0))
            ratio = abs(d) / per[j]
            rows[f"{pair}|{j}|{mode}"] = {
                "difference_mrad": 1e3 * d,
                "se_mrad": 1e3 * se,
                "band_mrad": 1e3 * per[j],
                "ratio": ratio,
                "class": classify(ratio),
            }
    return rows


def positive_control(fits: dict) -> dict:
    """C5: the 2021-10-26 sessions 1051 and 1102 against the recorded encoder difference."""
    a, b = fits["Q1051|3d"], fits["Q1102|3d"]
    rows, passed = {}, True
    for j in mh.COMPARABLE:
        d = 1e3 * (a["params"][j] - b["params"][j])
        se = 1e3 * float(np.hypot(a["std"][j], b["std"][j]))
        if j.endswith("arm_6_joint"):
            # sign by convention: the session pair is not signed in the data
            err = min(abs(d - C5_ARM6_MRAD), abs(d + C5_ARM6_MRAD))
            sign = "-" if abs(d + C5_ARM6_MRAD) < abs(d - C5_ARM6_MRAD) else "+"
            expect = f"{sign}{C5_ARM6_MRAD}"
        else:
            err, expect = abs(d), "0"
        ok = err <= 2.0 * se
        passed &= ok
        rows[j] = {
            "difference_mrad": d,
            "se_mrad": se,
            "expected_mrad": expect,
            "deviation_mrad": err,
            "limit_mrad": 2.0 * se,
            "pass": bool(ok),
        }
    # arm_1 is absorbed by the base, arm_7 is not identifiable with a 3D point
    unobservable = (
        "offsetRZ_arm_1_joint" in a["absorbed"]
        and "offsetRZ_arm_7_joint" not in a["params"]
    )
    # power of the control: the same limit also accepts "no difference" on arm_6
    arm6 = rows["offsetRZ_arm_6_joint"]
    return {
        "rows": rows,
        "arm_1_arm_7_unobservable": unobservable,
        "arm_6_null_also_within_limit": bool(
            abs(arm6["difference_mrad"]) <= arm6["limit_mrad"]
        ),
        "pass": bool(passed),
    }


def build_comparisons(fits, matrix, arm5, bands) -> dict:
    """C1 .. C5 and the 6D secondary, each with a verdict against the frozen bands."""
    control = positive_control(fits)
    t3, t6 = transfer_rows(matrix, bands, "3d"), transfer_rows(matrix, bands, "6d")
    p3, p6 = parameter_rows(fits, bands, "3d"), parameter_rows(fits, bands, "6d")
    c2 = {k: v for k, v in t3.items() if k.startswith("qualisys->")}
    c3t = {k: v for k, v in t3.items() if not k.startswith("qualisys->")}
    c3p = {k: v for k, v in p3.items() if k.startswith("optitrack-vicon")}
    own = {
        "M_OT->optitrack/2023-09-12-1552": ("M_OT", "optitrack/2023-09-12-1552"),
        "M_V->vicon/2023-09-20-1607": ("M_V", "vicon/2023-09-20-1607"),
    }
    c1 = {}
    for name in MODELS:
        f = fits[f"{name}|3d"]
        train = matrix[f"{SYSTEM_OF[name]}|{f['session']}|3d"]
        c1[name] = {
            "session": f["session"],
            "n": f["n"],
            "train_insample_mm": train["insample_mm"],
            "train_loo_mm": train["loo_mm"],
            "condition": f["condition"],
            "params_mrad": {n: 1e3 * f["params"][n] for n in mh.COMPARABLE},
            "se_mrad": {n: 1e3 * f["std"][n] for n in mh.COMPARABLE},
        }
    for label, (name, sess) in own.items():
        c1[name]["other_session"] = sess
        c1[name]["other_session_loo_mm"] = matrix[f"{SYSTEM_OF[name]}|{sess}|3d"][
            "loo_mm"
        ]
        c1[name]["other_session_insample_mm"] = matrix[f"{SYSTEM_OF[name]}|{sess}|3d"][
            "insample_mm"
        ]
    comparisons = {
        "C1": {
            "title": "per-system fit and own-system LOO",
            "rows": c1,
            "verdict": "descriptive (no pre-registered band)",
        },
        "C2": {
            "title": "2021 model on 2023 data: transfer gap vs band",
            "rows": c2,
            "verdict": verdict_text([r["class"] for r in c2.values()]),
        },
        "C3": {
            "title": "near-date hardware: OptiTrack model on Vicon and the reverse",
            "transfer": c3t,
            "parameters": c3p,
            "verdict": verdict_text(
                [r["class"] for r in c3t.values()] + [r["class"] for r in c3p.values()]
            ),
        },
        "C5": {
            "title": "positive control: 2021-10-26 1051 against 1102",
            **control,
            "verdict": "control passed" if control["pass"] else "control FAILED",
        },
        "arm5_diagnostic": {"title": "M_Q with only arm_5 refitted", "rows": arm5},
        "secondary_6d": {
            "title": "6D (W_ROT = 0.1 m/rad; not optimal until core #215; never decides)",
            "transfer": t6,
            "parameters": p6,
            "verdict": verdict_text(
                [r["class"] for r in t6.values()] + [r["class"] for r in p6.values()]
            ),
        },
        "supporting": {
            "parameters_vs_qualisys": {
                k: v for k, v in p3.items() if not k.startswith("optitrack-vicon")
            }
        },
    }
    if not control["pass"]:
        for key in ("C2", "C3"):
            comparisons[key]["verdict"] = "not interpreted: C5 control failed"
    return comparisons


def real_stage(sessions: Optional[dict] = None, reverse: bool = False) -> dict:
    """Fits, scores and the comparisons C1, C2, C3, C5 (+ diagnostic, 6D)."""
    t0 = time.time()
    sessions = sessions or mh.load_sessions()
    print("real stage: fits (fresh calibration object per fit)")
    fits, calibs = fit_all(sessions, reverse=reverse)
    data = scored_sessions(sessions)
    chain = mh.Chain(calibs["M_Q|3d"])
    print("scoring (LOO re-registration)", flush=True)
    matrix = score_matrix(chain, fits, data)
    print("arm_5-only diagnostic", flush=True)
    arm5 = arm5_diagnostic(chain, fits, data)
    comparisons = build_comparisons(fits, matrix, arm5, frozen_bands())
    print("residual-step diagnostic (post hoc)", flush=True)
    steps = step_diagnostic(chain, fits, data)
    return {
        "steps": steps,
        "fits": fits,
        "matrix": matrix,
        "comparisons": comparisons,
        "sessions": {k: len(d) for k, d in data.items()},
        "seconds": round(time.time() - t0, 1),
        "_calibs": calibs,
    }


# --- square motion (C4) ------------------------------------------------------


def load_square_runs() -> List[dict]:
    """The seven validation runs: joints, end-effector marker path, offset set."""
    protocol = Protocol.load(SQUARE / "protocol.yaml")
    protocol.verify(root=SQUARE)
    manifest = yaml.safe_load((SQUARE / "protocol.yaml").read_text())
    runs = []
    for entry in manifest["sessions"]:
        files = list(entry["files"])
        joints = pd.read_csv(
            SQUARE / next(f for f in files if f.endswith("_joints.csv"))
        )
        markers = pd.read_csv(
            SQUARE / next(f for f in files if f.endswith("_markers.csv"))
        )
        marker = markers[markers["body"] == entry["end_effector"]]
        runs.append(
            {
                "id": entry["id"],
                "offset_set": entry["offset_set"],
                "t": joints["t"].to_numpy(),
                "q": joints[mh.JOINTS].to_numpy(),
                "t_marker": marker["t"].to_numpy(),
                "marker": marker[["x", "y", "z"]].to_numpy(),
            }
        )
    return runs


def recorded_set(name: str = "mocap") -> dict:
    manifest = yaml.safe_load((SQUARE / "protocol.yaml").read_text())
    return manifest["offset_sets"][name]["values"]


def run_poses(model, q, t, t_marker, lag_shift: float = 0.0, offsets=None):
    """Tool poses of a model at the marker times (joints read at ``t + lag_shift``)."""
    tm = t_marker + lag_shift
    q_at = np.column_stack([np.interp(tm, t, q[:, k]) for k in range(q.shape[1])])
    if offsets is not None:
        q_at = mh.apply_joint_offsets(q_at, offsets)
    return sme.tool_poses(model, sme.TOOL_FRAME, q_at)


def square_stage(fits: Optional[dict] = None, sessions: Optional[dict] = None) -> dict:
    """Registered path RMSE (mm) of each model on the square-motion runs, with lags."""
    t0 = time.time()
    if fits is None:
        sessions = sessions or mh.load_sessions()
        fits = {}
        for name, key in {**MODELS, **DESCRIPTIVE}.items():
            if name in ("OT1552", "V1607"):
                continue
            fits[f"{name}|3d"] = fit_record(sessions[key])[0]
    nominal = pin.buildModelFromUrdf(str(mh.URDF))
    placed = mh.placement_model(nominal, recorded_set("mocap"))
    # the inverse set is the sign check of the recorded one (the sets are the
    # same offsets with opposite sign, protocol.yaml): a descriptive column
    placed_inv = mh.placement_model(nominal, recorded_set("mocap_inv"))
    models = {
        "zero": (nominal, None),
        "mocap set": (placed, None),
        "mocap_inv set": (placed_inv, None),
    }
    for name in ("M_Q", "Q1051", "Q1102", "M_OT", "M_V"):
        models[name] = (nominal, np.array(fits[f"{name}|3d"]["offsets"]))
    runs = load_square_runs()
    table, lags = {}, {}
    for run in runs:
        row = {}
        for name, (model, off) in models.items():
            R, p = run_poses(model, run["q"], run["t"], run["t_marker"], 0.0, off)
            row[name] = mh.path_rmse(R, p, run["marker"])
        table[run["id"]] = row
        lag = {}
        for name in ("zero", "M_Q"):
            model, off = models[name]
            lag[name] = {}
            for delta in (*SQUARE_LAGS_S, 0.0):
                R, p = run_poses(model, run["q"], run["t"], run["t_marker"], delta, off)
                lag[name][f"{delta:+.2f}"] = mh.path_rmse(R, p, run["marker"])
        lags[run["id"]] = lag
        print(
            f"  {run['id']:30s} " + " ".join(f"{k} {v:.2f}" for k, v in row.items()),
            flush=True,
        )
    mean = {n: float(np.mean([r[n] for r in table.values()])) for n in models}
    return {
        "rmse_mm": table,
        "mean_rmse_mm": mean,
        "lag_sweep_mm": lags,
        "offset_sets": {r["id"]: r["offset_set"] for r in runs},
        "seconds": round(time.time() - t0, 1),
    }


# --- output ------------------------------------------------------------------


def print_real(res: dict) -> None:
    c = res["comparisons"]
    print("\nC1  per-system fit (training in-sample / LOO mm, cond)")
    for name, r in c["C1"]["rows"].items():
        extra = (
            f"  other session {r['other_session'].split('/')[1]} LOO "
            f"{r['other_session_loo_mm']:.2f} mm"
            if "other_session" in r
            else ""
        )
        print(
            f"  {name:5s} n={r['n']:2d} {r['train_insample_mm']:.2f} / "
            f"{r['train_loo_mm']:.2f} mm cond {r['condition']:.1f}{extra}"
        )
        print(
            "        "
            + "  ".join(
                f"{n[8:13]} {r['params_mrad'][n]:+.1f}+-{r['se_mrad'][n]:.1f}"
                for n in mh.COMPARABLE
            )
            + " mrad"
        )
    for key in ("C2", "C3"):
        block = c[key]
        print(f"\n{key}  {block['title']}: {block['verdict']}")
        for k, v in (block.get("rows") or block["transfer"]).items():
            print(
                f"  {k:46s} G {v['gap_mm']:+.2f} mm  band {v['band_mm']:.2f}  "
                f"{v['ratio']:.2f}  {v['class']}"
            )
        for k, v in block.get("parameters", {}).items():
            print(
                f"  {k:46s} d {v['difference_mrad']:+7.2f} mrad "
                f"(se {v['se_mrad']:.2f}) band {v['band_mrad']:.2f}  "
                f"{v['ratio']:.2f}  {v['class']}"
            )
    c5 = c["C5"]
    print(f"\nC5  {c5['verdict']}")
    for j, r in c5["rows"].items():
        print(
            f"  {j:26s} d {r['difference_mrad']:+7.2f} mrad (se {r['se_mrad']:.2f}) "
            f"expected {r['expected_mrad']:>5s} dev {r['deviation_mrad']:.2f} "
            f"limit {r['limit_mrad']:.2f}  {'ok' if r['pass'] else 'FAIL'}"
        )
    print("\narm_5-only diagnostic (M_Q, arm_5 + frames refitted)")
    for k, v in c["arm5_diagnostic"]["rows"].items():
        print(
            f"  {k:30s} arm_5 {v['arm_5_offset_mrad']:+.1f} mrad "
            f"(M_Q {v['arm_5_offset_M_Q_mrad']:+.1f})  LOO {v['loo_mm']:.2f} mm"
        )
    print("\nresidual-step diagnostic (post hoc, no verdict): best single split")
    for k, v in res["steps"].items():
        print(
            f"  {k:28s} split {v['split']:2d}/{v['n']:2d}  before {v['before_mm']:.2f} "
            f"after {v['after_mm']:.2f} mm  rmse {v['rmse_mm']:.2f} -> "
            f"{v['rmse_split_arm5_mm']:.2f}  arm_5 jump {v['arm_5_jump_mrad']:+.1f} mrad"
        )
    s = c["secondary_6d"]
    print(f"\n6D secondary (never decides): {s['verdict']}")
    for k, v in s["transfer"].items():
        print(
            f"  {k:46s} G {v['gap_mm']:+.2f} mm  band {v['band_mm']:.2f}  "
            f"{v['ratio']:.2f}  {v['class']}"
        )
    for k, v in s["parameters"].items():
        print(
            f"  {k:46s} d {v['difference_mrad']:+7.2f} mrad band {v['band_mrad']:.2f}  "
            f"{v['ratio']:.2f}  {v['class']}"
        )


def print_square(sq: dict) -> None:
    names = list(next(iter(sq["rmse_mm"].values())))
    print("\nC4  registered path RMSE (mm)")
    print("  " + f"{'run':30s}" + "".join(f"{n:>10s}" for n in names))
    for run, row in sq["rmse_mm"].items():
        print("  " + f"{run:30s}" + "".join(f"{row[n]:10.2f}" for n in names))
    print(
        "  "
        + f"{'mean':30s}"
        + "".join(f"{sq['mean_rmse_mm'][n]:10.2f}" for n in names)
    )
    print("lag sweep (mean over runs)")
    for name in ("zero", "M_Q"):
        cols = sq["lag_sweep_mm"][next(iter(sq["lag_sweep_mm"]))][name]
        print(
            f"  {name:5s} "
            + "  ".join(
                f"{d} s {np.mean([r[name][d] for r in sq['lag_sweep_mm'].values()]):.2f}"
                for d in cols
            )
        )


def strip(res: dict) -> dict:
    return {k: v for k, v in res.items() if not k.startswith("_")}


def golden(real: dict, square: dict, fixture_z: Optional[dict] = None) -> dict:
    """The golden record: parameters, LOO matrix, comparisons and square RMSE."""
    out = {
        "fits": {
            k: {
                "params": v["params"],
                "std": v["std"],
                "condition": v["condition"],
                "absorbed": v["absorbed"],
                "status": v["status"],
                "n": v["n"],
            }
            for k, v in real["fits"].items()
        },
        "matrix": {
            k: {m: x for m, x in v.items() if m != "n"}
            for k, v in real["matrix"].items()
        },
        "comparisons": real["comparisons"],
        "steps": real["steps"],
        "square": {k: square[k] for k in ("rmse_mm", "mean_rmse_mm", "lag_sweep_mm")},
    }
    if fixture_z is not None:
        out["fixture_recovery_z"] = fixture_z
    return out


def report_html(real: Optional[dict], square: Optional[dict]) -> str:
    """A small self-contained report of the study run."""
    rows = []

    def table(title, header, body):
        rows.append(f"<h2>{html.escape(title)}</h2><table><tr>")
        rows.append("".join(f"<th>{html.escape(h)}</th>" for h in header) + "</tr>")
        for r in body:
            rows.append(
                "<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in r) + "</tr>"
            )
        rows.append("</table>")

    if real:
        c = real["comparisons"]
        table(
            "Verdicts against the frozen bands",
            ["comparison", "verdict"],
            [[k, c[k]["verdict"]] for k in ("C1", "C2", "C3", "C5", "secondary_6d")],
        )
        for key in ("C2",):
            table(
                c[key]["title"],
                ["pair", "gap mm", "band mm", "ratio", "class"],
                [
                    [
                        k,
                        f"{v['gap_mm']:+.2f}",
                        f"{v['band_mm']:.2f}",
                        f"{v['ratio']:.2f}",
                        v["class"],
                    ]
                    for k, v in c[key]["rows"].items()
                ],
            )
        table(
            c["C3"]["title"],
            ["item", "value", "band", "ratio", "class"],
            [
                [
                    k,
                    f"{v['gap_mm']:+.2f} mm",
                    f"{v['band_mm']:.2f}",
                    f"{v['ratio']:.2f}",
                    v["class"],
                ]
                for k, v in c["C3"]["transfer"].items()
            ]
            + [
                [
                    k,
                    f"{v['difference_mrad']:+.2f} mrad",
                    f"{v['band_mrad']:.2f}",
                    f"{v['ratio']:.2f}",
                    v["class"],
                ]
                for k, v in c["C3"]["parameters"].items()
            ],
        )
        table(
            "C5 positive control",
            ["joint", "difference mrad", "expected", "limit mrad", "pass"],
            [
                [
                    j,
                    f"{r['difference_mrad']:+.2f}",
                    r["expected_mrad"],
                    f"{r['limit_mrad']:.2f}",
                    r["pass"],
                ]
                for j, r in c["C5"]["rows"].items()
            ],
        )
    if square:
        names = list(next(iter(square["rmse_mm"].values())))
        table(
            "C4 registered path RMSE (mm)",
            ["run", *names],
            [
                [run, *[f"{row[n]:.2f}" for n in names]]
                for run, row in square["rmse_mm"].items()
            ],
        )
    return (
        "<!doctype html><meta charset='utf-8'><title>TIAGo mocap across hardware</title>"
        "<style>body{font:14px sans-serif;margin:2em}table{border-collapse:collapse}"
        "td,th{border:1px solid #999;padding:2px 8px;text-align:right}</style>"
        "<h1>TIAGo mocap across hardware (#100)</h1>" + "".join(rows)
    )


def archive(real, square, args) -> int:
    """Run record: report, heldout, corrections, verdict, provenance, reproduction."""
    from figaroh.tools.run_archive import archive_run, compute_run_dir

    from examples.run_record import (
        audit,
        describe_file,
        missing,
        write_reproduction_record,
    )

    calib = real["_calibs"]["M_Q|3d"]
    run_dir = compute_run_dir(calib, root=args.root)
    (run_dir / "report.html").write_text(report_html(real, square))
    c = real["comparisons"]
    heldout = {
        "primary": "leave-one-out re-registered RMSE (mm)",
        "secondary": "in-sample re-registered RMSE (mm)",
        "matrix": {
            k: {m: x for m, x in v.items() if m != "n"}
            for k, v in real["matrix"].items()
        },
        "sessions": real["sessions"],
        "residual_steps_post_hoc": real["steps"],
        "square": None if square is None else strip(square),
    }
    corrections = {
        "gauge": {
            "estimated_frames": mh.FRAME_NAMES + mh.TOOL_NAMES,
            "absorbed_by_frames": real["fits"]["M_Q|3d"]["absorbed"],
            "comparable": mh.COMPARABLE,
        },
        "fits": {
            k: {
                key: v[key]
                for key in (
                    "session",
                    "mode",
                    "params",
                    "std",
                    "condition",
                    "status",
                    "n",
                )
            }
            for k, v in real["fits"].items()
        },
    }
    verdict = {
        "status": "complete" if c["C5"]["pass"] else "control failed",
        "selected_stage": "fit",
        "splits": {
            "validation_source": "none: held-out sessions are scored by leave-one-out "
            "re-registration; nothing is selected on any session",
            "training": [v for v in MODELS.values()],
            "evaluated": sorted(real["sessions"]),
        },
        "verdicts": {
            k: c[k]["verdict"] for k in ("C1", "C2", "C3", "C5", "secondary_6d")
        },
        "bands": "tests/golden/tiago_mocap_hardware_truth.json (frozen)",
    }
    for name, data in (
        ("heldout.json", heldout),
        ("corrections.json", corrections),
        ("verdict.json", verdict),
    ):
        (run_dir / name).write_text(json.dumps(data, indent=2, default=str))
    archive_run(calib, run_dir)
    sessions = mh.load_sessions()
    inputs = {key: describe_file(s.path) for key, s in sessions.items()}
    for system, path in mh.PROTOCOLS.items():
        inputs[f"{system}/protocol.yaml"] = describe_file(path)
    inputs["square_motion/protocol.yaml"] = describe_file(SQUARE / "protocol.yaml")
    inputs["frozen bands"] = describe_file(FROZEN)
    artifacts = {
        name: describe_file(run_dir / name)
        for name in ("heldout.json", "corrections.json", "verdict.json")
    }
    write_reproduction_record(
        run_dir,
        processing={
            "command": "mocap_hardware.py " + " ".join(sys.argv[1:]),
            "model": "joint_offset, structural, position-only xyz, base 6D + tool 3D",
            "estimation": "fresh calibration object and Kabsch+LM registration start per fit",
            "primary_metric": "leave-one-out re-registered RMSE",
            "w_rot_m_per_rad": mh.W_ROT,
            "min_postures": mh.MIN_POSTURES,
            "protocols": {
                s: str(p.relative_to(mh.TIAGO)) for s, p in mh.PROTOCOLS.items()
            },
        },
        inputs=inputs,
        artifacts=artifacts,
    )
    checks = audit(run_dir)
    print(f"\nRun archived to {run_dir}")
    return 1 if missing(checks) else 0


def run(stage: str, args) -> int:
    real = square = None
    code = 0
    if stage in ("real", "all"):
        real = real_stage()
        print_real(real)
    if stage in ("square", "all"):
        print("\nsquare stage")
        square = square_stage(None if real is None else real["fits"])
        print_square(square)
    if real is not None and not real["comparisons"]["C5"]["pass"]:
        print("\nC5 positive control FAILED: C2 and C3 are not interpreted")
        code = 1
    if args.archive and real is not None:
        code |= archive(real, square, args)
    elif args.archive:
        print("--archive needs the real stage (real or all)")
        code = 1
    if args.out:
        args.out.write_text(
            json.dumps(
                {k: strip(v) for k, v in (("real", real), ("square", square)) if v},
                indent=1,
                default=str,
            )
        )
    if args.golden and real is not None and square is not None:
        args.golden.write_text(
            json.dumps(
                golden(real, square, FIXTURE_RECOVERY_Z),
                indent=1,
                sort_keys=True,
                default=str,
            )
            + "\n"
        )
    return code
