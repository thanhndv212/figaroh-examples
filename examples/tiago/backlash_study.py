"""TIAGo backlash surface (#71): the study (part 2).

The model, the selection rule, the bands, the verdict rules and the
not-answerable rules are the pre-registered ones (``backlash_model.py``,
``config/tiago_backlash_study_config.yaml``, the bands in
``tests/golden/tiago_backlash_truth.json``,
``docs/development/tiago-backlash.md`` sections 3 to 9). Nothing here is tuned
on any session. The verdicts are validation-level: there is no confirmation
set (D2). The primary model M* is the all-groups model (E, W and the load
group); the selected-groups model is secondary and descriptive.

The order is fixed, and the validation absolute encoder is unlocked only after
the training stage:

(a) ``training``: the fit on 2023-07-24 (E frozen, an offset per session and
    jump segment, W, G, kappa with block-bootstrap SE read as upper bounds,
    the frozen selection rule);
(b) ``validation``: B0, B1, B2 (degree 5), M* and the selected model on the
    2023-03-09 bottom run, offsets re-anchored on it;
(c) ``verdicts``: V1, V2, V3 per joint with the rules N1 to N6;
(d) ``inspection``: prediction checks on the joint-inspection recordings,
    descriptive, nothing scored.

    python backlash_study.py --archive --no-plot
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional

import numpy as np
import yaml

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import backlash_model as bm  # noqa: E402

FROZEN = project_root / "tests/golden/tiago_backlash_truth.json"
JI = bm.TIAGO / "data/joint_inspection"
SETTINGS = bm.SETTINGS
JOINTS = bm.JOINTS
RULES = SETTINGS["rules"]
URDF = SETTINGS["urdf"]["backlash_days"]
HEY5 = SETTINGS["urdf"]["joint_inspection"]
SCHUNK = SETTINGS["urdf"]["backlash_days"]
LEVEL = str(SETTINGS["noise"]["used_level"])
LABEL = "validation-level"
PROBES = {
    "arm_6_backlash_abs_inspect_2024-01-26-18-07-23": 38.3,
    "arm_6_backlash_abs_inspect_2024-01-26-18-07-54": 42.2,
}
MRAD = 1e3


# --- verdict rules -----------------------------------------------------------


def frozen() -> dict:
    """The fixture results the rules are read from (bands, ratios, SE)."""
    return json.loads(FROZEN.read_text())


def band_tables(g: Optional[dict] = None) -> dict:
    """Per joint: ``b_V1``, ``band_W`` (mrad), the F4 ratio of the primary model
    (rule N6) and the fixture SE of W (rule N3, seed 0)."""
    g = g or frozen()
    lv = g["levels"][LEVEL]["f4"]["joints"]
    se = g["seed0"]["f2"]["joints"]
    return {
        j: {
            "b_v1_mrad": lv[j]["band_v1_mrad"],
            "band_w_mrad": lv[j]["band_w_mrad"],
            "f4_ratio": lv[j]["ratio_m3"],
            "fixture_w_se_mrad": se[j]["W_se_mrad"],
        }
        for j in JOINTS
    }


def classify_v1(gap_mrad: float, band_mrad: float, borderline: float = None) -> dict:
    """V1: ``g = RMSE(M*) - RMSE(B1)`` against ``b_V1``; borderline within
    ``borderline`` (35 %) of the band from an edge."""
    borderline = RULES["v1_borderline"] if borderline is None else borderline
    if gap_mrad < -band_mrad:
        cls = "improves"
    elif gap_mrad > band_mrad:
        cls = "worse"
    else:
        cls = "no change"
    edge = abs(abs(gap_mrad) - band_mrad) <= borderline * band_mrad
    return {"class": cls, "borderline": bool(edge)}


def classify_v2(dw_mrad: float, band_mrad: float) -> str:
    return "W stable across sessions" if abs(dw_mrad) <= band_mrad else "W differs"


def classify_v3(selected_load: bool, rows: List[dict]) -> str:
    """V3: the load group admitted on training, and a coefficient with the same
    sign and ``|z| > v3_z_validation_min`` on the validation refit."""
    if not selected_load:
        return "no load dependence detected"
    ok = any(r["same_sign"] and r["z_val"] > RULES["v3_z_validation_min"] for r in rows)
    return "load-dependent" if ok else "no load dependence detected"


def _f(x) -> Optional[float]:
    return None if x is None or not np.isfinite(x) else float(x)


# --- data --------------------------------------------------------------------


def joint_data(run, targets, urdf, role) -> Dict[str, bm.JointData]:
    return {j: bm.joint_data(run, j, targets[j], urdf) for j in JOINTS}


def segment_stats(jd: bm.JointData) -> dict:
    """Jump count and shortest segment (s), for rule N1."""
    n_seg = int(jd.seg.max()) + 1
    dur = [float(jd.t[jd.seg == k][-1] - jd.t[jd.seg == k][0]) for k in range(n_seg)]
    return {"jumps": n_seg - 1, "min_segment_s": min(dur)}


def run_stats(jd: bm.JointData, run) -> dict:
    r = jd.rows()
    sg = bm.soft_sign(r.tau)
    return {
        "n": int(r.t.size),
        "valid_fraction": float(np.mean(jd.valid)),
        "reversals": bm.reversal_count(jd.h),
        "overlap": bm.direction_overlap(r),
        "corr_h_sg": bm._corr(r.h, sg),
        "tau_sd": float(np.std(r.tau)),
        "n5_sign_fraction": bm.gravity_sign_fraction(run, jd.joint),
        **segment_stats(jd),
    }


def offsets_mrad(fit: bm.Fit, jd: bm.JointData) -> List[float]:
    """Offset per jump segment on the gauge of the fit (first E hat dropped)."""
    p = fit.terms(jd)
    d = jd.d
    return [
        float(np.mean((d - p)[(jd.seg == k) & jd.valid]) * MRAD)
        for k in range(int(jd.seg.max()) + 1)
    ]


# --- (a) training --------------------------------------------------------------


def fit_record(fit: bm.Fit) -> dict:
    rec = {
        "groups": "".join(fit.groups),
        "rank": fit.rank,
        "n_columns": int(fit.keep.sum()),
        "cond": _f(fit.cond),
        "n": fit.n,
        "rmse_mrad": fit.rmse * MRAD,
        "load_use": [bool(u) for u in fit.load_use],
    }
    if "W" in fit.groups:
        rec["W_mrad"] = fit.W * MRAD
        if fit.se is not None:
            rec["W_se_mrad"] = float(fit.se[fit.index["W"][0]] * MRAD)
    if "L" in fit.groups:
        rec["G_mrad"] = fit.G * MRAD
        rec["kappa_mrad_per_nm"] = fit.kappa * MRAD
        if fit.se is not None:
            i = fit.index["L"]
            rec["G_se_mrad"] = float(fit.se[i[0]] * MRAD)
            rec["kappa_se_mrad_per_nm"] = float(fit.se[i[1]] * MRAD)
    return rec


def training_stage(jds: Dict[str, bm.JointData], run) -> dict:
    out = {"session": run.session.id, "joints": {}}
    for j in JOINTS:
        jd = jds[j]
        knots = bm.make_knots(jd.q_abs)
        rec = {
            "stats": run_stats(jd, run),
            "knots": [float(knots[0]), float(knots[-1]), int(knots.size)],
            "q_abs_range": [float(jd.q_abs.min()), float(jd.q_abs.max())],
        }
        fits = {
            "M0": bm.fit_groups(jd, knots, ()),
            "M1": bm.fit_groups(jd, knots, ("E",)),
            "M2": bm.fit_groups(jd, knots, ("E", "W"), bootstrap=True),
            "M3": bm.fit_groups(jd, knots, ("E", "W", "L"), bootstrap=True),
        }
        rec["models"] = {k: fit_record(f) for k, f in fits.items()}
        rec["offsets_mrad"] = offsets_mrad(fits["M3"], jd)
        sel = bm.select_groups(jd, knots)
        fsel = bm.fit_groups(jd, knots, sel["groups"], bootstrap=True)
        rec["selection"] = {
            "groups": "".join(sel["groups"]),
            "cv_rmse_mrad": {k: v * MRAD for k, v in sel["cv"].items()},
            "z": sel["z"],
            "fit": fit_record(fsel),
        }
        rec["_knots"] = knots
        rec["_fits"] = fits
        rec["_selected"] = fsel
        out["joints"][j] = rec
    return out


# --- (b) validation baselines --------------------------------------------------


def _rmse_pred(jd: bm.JointData, pred: np.ndarray, mask=None) -> float:
    m = jd.valid if mask is None else (jd.valid & mask)
    return float(np.sqrt(np.mean((jd.d - pred)[m] ** 2)) * MRAD)


def fit_b2_backoff(jd: bm.JointData, max_degree: int = 5):
    """B2 at degree 5; where the design is rank deficient, the highest lower
    degree that is full rank, as ``backlash_empirical_surface.py`` does."""
    for degree in range(max_degree, -1, -1):
        try:
            return bm.fit_b2(jd, degree)
        except ValueError:
            continue
    raise ValueError("no full-rank degree for B2")


def validation_stage(train: dict, jds_t, jds_v, run_v) -> dict:
    out = {"session": run_v.session.id, "joints": {}}
    for j in JOINTS:
        jt, jv = jds_t[j], jds_v[j]
        tr = train["joints"][j]
        knots = tr["_knots"]
        inside = (jv.q_abs >= knots[0]) & (jv.q_abs <= knots[-1])
        b2 = fit_b2_backoff(jt)
        preds = {
            "B0": bm.predict(bm.fit_b0(jt), jv),
            "B1": bm.predict(bm.fit_b1(jt), jv),
            "B2": bm.predict_b2(b2, jv),
            "M*": bm.predict(tr["_fits"]["M3"], jv),
            "selected": bm.predict(tr["_selected"], jv),
        }
        rmse = {k: _rmse_pred(jv, p) for k, p in preds.items()}
        rmse_in = {k: _rmse_pred(jv, p, inside) for k, p in preds.items()}
        # train RMSE of the same models (not re-anchored differences: own session)
        train_rmse = {
            "B0": bm.rmse(bm.fit_b0(jt), jt) * MRAD,
            "B1": bm.rmse(bm.fit_b1(jt), jt) * MRAD,
            "B2": _rmse_pred(jt, bm.predict_b2(b2, jt)),
            "M*": bm.rmse(tr["_fits"]["M3"], jt) * MRAD,
            "selected": bm.rmse(tr["_selected"], jt) * MRAD,
        }
        out["joints"][j] = {
            "stats": run_stats(jv, run_v),
            "rmse_mrad": rmse,
            "rmse_inside_knots_mrad": rmse_in,
            "train_rmse_mrad": train_rmse,
            "b2_degree": int(b2.degree),
            # descriptive: what each term adds (not part of any verdict)
            "decomposition_mrad": {
                "M0 offset": rmse["B0"],
                "M1 E": _rmse_pred(jv, bm.predict(tr["_fits"]["M1"], jv)),
                "M2 E+W": _rmse_pred(jv, bm.predict(tr["_fits"]["M2"], jv)),
                "M3 E+W+L": rmse["M*"],
            },
            "outside_fraction": float(
                np.mean(~inside & jv.valid) / max(np.mean(jv.valid), 1e-9)
            ),
            "offset_mrad": {
                "training": tr["offsets_mrad"],
                "validation": offsets_mrad(tr["_fits"]["M3"], jv),
            },
        }
    return out


# --- (c) verdicts --------------------------------------------------------------


def verdict_stage(train: dict, val: dict, jds_t, jds_v) -> dict:
    bands = band_tables()
    out = {"label": LABEL, "primary_model": "all groups (E, W, L)", "joints": {}}
    for j in JOINTS:
        tr, va, b = train["joints"][j], val["joints"][j], bands[j]
        jt, jv = jds_t[j], jds_v[j]
        flags = {}
        # N1: jumps and short segments, either run
        flags["N1"] = any(
            s["jumps"] > RULES["n1_max_jumps"]
            or s["min_segment_s"] < SETTINGS["jumps"]["min_segment_s"]
            for s in (tr["stats"], va["stats"])
        )
        # N2 (arm extension): direction overlap in training
        flags["N2"] = tr["stats"]["overlap"] < RULES["n2_min_overlap"]
        w_tr = tr["models"]["M2"]["W_mrad"]
        flags["N3"] = b["fixture_w_se_mrad"] > max(
            RULES["n3_se_floor_mrad"], abs(w_tr) / 2
        )
        flags["N4"] = any(
            abs(s["corr_h_sg"]) > RULES["n4_corr_max"]
            for s in (tr["stats"], va["stats"])
        )
        flags["N5"] = any(
            s["n5_sign_fraction"] > RULES["n5_sign_fraction"]
            for s in (tr["stats"], va["stats"])
        )
        flags["N6"] = b["f4_ratio"] >= RULES["n6_ratio_min"]
        reasons = {k: bool(v) for k, v in flags.items()}

        # V1
        g = va["rmse_mrad"]["M*"] - va["rmse_mrad"]["B1"]
        gs = va["rmse_mrad"]["selected"] - va["rmse_mrad"]["B1"]
        v1 = {
            "g_mrad": g,
            "g_selected_mrad": gs,
            "b_v1_mrad": b["b_v1_mrad"],
            **classify_v1(g, b["b_v1_mrad"]),
        }
        v1["answerable"] = not (flags["N1"] or flags["N6"])
        v1["verdict"] = (
            f"{v1['class']}{' (borderline)' if v1['borderline'] else ''}"
            if v1["answerable"]
            else "not answerable"
        )
        v1["sd_selected_class"] = classify_v1(gs, b["b_v1_mrad"])["class"]

        # V2: E+W fits of both sessions, W in mrad
        fv = bm.fit_groups(jv, bm.make_knots(jv.q_abs), ("E", "W"), bootstrap=True)
        w_val = fv.W * MRAD
        dw = w_tr - w_val
        v2 = {
            "W_train_mrad": w_tr,
            "W_val_mrad": w_val,
            "W_train_se_mrad": tr["models"]["M2"].get("W_se_mrad"),
            "W_val_se_mrad": float(fv.se[fv.index["W"][0]] * MRAD),
            "dW_mrad": dw,
            "band_w_mrad": b["band_w_mrad"],
            "ratio": abs(dw) / b["band_w_mrad"],
            "W_all_groups_train_mrad": tr["models"]["M3"]["W_mrad"],
        }
        v2["answerable"] = not (flags["N1"] or flags["N2"] or flags["N3"])
        v2["verdict"] = (
            classify_v2(dw, b["band_w_mrad"])
            if v2["answerable"]
            else ("not answerable (W descriptive)")
        )

        # V3: selection on training, same-sign refit on validation
        sel = tr["selection"]
        fsel = tr["_selected"]
        selected_load = "L" in sel["groups"]
        rows = []
        degenerate = not any(fsel.load_use)
        if selected_load:
            fval = bm.fit_groups(
                jv, bm.make_knots(jv.q_abs), fsel.groups, bootstrap=True
            )
            for i, name in enumerate(("G", "kappa")):
                if not (fsel.load_use[i] and fval.load_use[i]):
                    continue
                zt, zv = fsel.z("L", i), fval.z("L", i)
                ct, cv = fsel.coef[fsel.index["L"][i]], fval.coef[fval.index["L"][i]]
                if zt > SETTINGS["selection"]["z_min"]:
                    rows.append(
                        {
                            "coef": name,
                            "train": float(ct * MRAD),
                            "val": float(cv * MRAD),
                            "z_train": float(zt),
                            "z_val": float(zv),
                            "same_sign": bool(np.sign(ct) == np.sign(cv)),
                        }
                    )
        v3 = {
            "selected_groups": sel["groups"],
            "load_admitted_train": bool(selected_load),
            "coefficients": rows,
            "answerable": not (flags["N1"] or flags["N4"] or flags["N5"] or degenerate),
            "degenerate_load": bool(degenerate),
        }
        v3["verdict"] = (
            classify_v3(selected_load, rows) if v3["answerable"] else "not answerable"
        )
        out["joints"][j] = {
            "flags": reasons,
            "V1": v1,
            "V2": v2,
            "V3": v3,
        }
    return out


# --- (d) joint inspection ------------------------------------------------------


def read_bag(path: Path):
    import pandas as pd

    return pd.read_csv(path)


def inspection_files() -> dict:
    """``{id: path}`` of the diagnostic bags, sha256-verified."""
    from figaroh.data.protocol import Protocol

    proto = Protocol.load(JI / "protocol.yaml")
    proto.verify(root=JI)
    meta = yaml.safe_load((JI / "protocol.yaml").read_text())["sessions"]
    return {s["id"]: JI / next(iter(s["files"])) for s in meta}


def nominal_config(run_t) -> np.ndarray:
    """Configuration of the joints the inspection files do not record: the
    median of the training run (torso, arm_1 to arm_4, arm_7)."""
    return np.median(run_t.q, axis=0)


def bag_gravity(urdf: str, nominal: np.ndarray, q5, q6) -> np.ndarray:
    """arm_6 generalized gravity torque with arm_5 and arm_6 from the bag and
    the other joints at ``nominal`` (the files record only arm_5 and arm_6)."""
    import pinocchio as pin

    m = bm._model(urdf)
    data = m.createData()
    q = pin.neutral(m)
    iq = [m.joints[m.getJointId(j)].idx_q for j in bm.CONFIG_JOINTS]
    i6 = m.joints[m.getJointId("arm_6_joint")].idx_v
    base = np.array(nominal, float)
    out = np.empty(q5.size)
    for i in range(q5.size):
        c = base.copy()
        c[bm.CONFIG_JOINTS.index("arm_5_joint")] = q5[i]
        c[bm.CONFIG_JOINTS.index("arm_6_joint")] = q6[i]
        q[iq] = c
        out[i] = pin.computeGeneralizedGravity(m, data, q)[i6]
    return out


def bag_joint_data(df, tau=None) -> bm.JointData:
    t = df["t"].to_numpy()
    q_rel = df["arm_6_position"].to_numpy()
    q_abs = df["arm_6_absolute"].to_numpy()
    h, valid = bm.direction_state(t, np.gradient(q_rel, t))
    d = q_rel - q_abs
    seg = bm.segment_ids(t.size, bm.jump_indices(d, q_rel))
    tau = np.zeros(t.size) if tau is None else tau
    return bm.JointData("arm_6_joint", t, q_rel, q_abs, h, tau, seg, valid)


def inspection_stage(train: dict, w_val_mrad: float, run_t) -> dict:
    files = inspection_files()
    fit = train["joints"]["arm_6_joint"]["_fits"]["M3"]
    w_all = fit.W * MRAD
    w_ew = train["joints"]["arm_6_joint"]["models"]["M2"]["W_mrad"]
    nominal = nominal_config(run_t)
    out = {
        "note": "descriptive prediction checks; no score, nothing fitted for a verdict",
        "hand": {"primary": HEY5, "sensitivity": SCHUNK, "verified": False},
        "nominal_configuration": "training median of torso, arm_1..arm_4, arm_7 (not recorded in these files)",
        "model_W_train_mrad": {"E+W": w_ew, "all_groups": w_all},
        "model_W_val_mrad": w_val_mrad,
        "probes": {},
        "arm5": {},
    }
    for sid, width in PROBES.items():
        df = read_bag(files[sid])
        p2p_abs = float(np.ptp(df["arm_6_absolute"]) * MRAD)
        p2p_rel = float(np.ptp(df["arm_6_position"]) * MRAD)
        out["probes"][sid] = {
            "probe_mrad": width,
            "abs_p2p_mrad": p2p_abs,
            "rel_p2p_mrad": p2p_rel,
            "W_over_probe": w_ew / p2p_abs,
            "W_minus_probe_mrad": w_ew - p2p_abs,
            "W_all_over_probe": w_all / p2p_abs,
        }
    knots_t = train["joints"]["arm_6_joint"]["_knots"]
    bags, excluded = {}, {}
    for sid, path in files.items():
        if not sid.startswith("arm5_"):
            continue
        df = read_bag(path)
        jd = bag_joint_data(df)
        if not jd.valid.any():
            excluded[sid] = "arm_6 does not move: no usable sample"
            continue
        kn = bm.make_knots(jd.q_abs)
        f = bm.fit_groups(jd, kn, ("E", "W"), bootstrap=True)
        # trained E+W model (frozen from 2023-07-24), offset re-anchored on the bag
        ft = train["joints"]["arm_6_joint"]["_fits"]["M2"]
        inside = (jd.q_abs >= knots_t[0]) & (jd.q_abs <= knots_t[-1])
        rec = {
            "arm5_angle_rad": float(np.median(df["arm_5_absolute"])),
            "arm5_range_rad": float(np.ptp(df["arm_5_absolute"])),
            "n": int(jd.valid.sum()),
            "jumps": int(jd.seg.max()),
            "W_mrad": f.W * MRAD,
            "W_se_mrad": float(f.se[f.index["W"][0]] * MRAD),
            "rmse_E_W_own_mrad": f.rmse * MRAD,
            "predict_EW_rmse_mrad": _rmse_pred(jd, bm.predict(ft, jd)),
            "outside_fraction": float(
                np.mean(~inside & jd.valid) / max(np.mean(jd.valid), 1e-9)
            ),
        }
        # load terms of M* with the nominal configuration, Hey5 and Schunk
        for tag, urdf in (("hey5", HEY5), ("schunk", SCHUNK)):
            tau = bag_gravity(
                urdf,
                nominal,
                df["arm_5_position"].to_numpy(),
                df["arm_6_position"].to_numpy(),
            )
            jt = bag_joint_data(df, tau)
            rec[f"predict_M*_rmse_{tag}_mrad"] = _rmse_pred(jt, bm.predict(fit, jt))
            rec[f"tau_sd_{tag}"] = float(np.std(tau[jd.valid]))
        bags[sid] = rec
    out["arm5"] = bags
    out["arm5_excluded"] = excluded
    ang = np.array([b["arm5_angle_rad"] for b in bags.values()])
    w = np.array([b["W_mrad"] for b in bags.values()])
    order = np.argsort(ang)
    out["arm5_summary"] = {
        "bags": len(bags),
        "W_mean_mrad": float(w.mean()),
        "W_sd_mrad": float(w.std(ddof=1)),
        "W_min_mrad": float(w.min()),
        "W_max_mrad": float(w.max()),
        "corr_W_angle": bm._corr(ang, w),
        # post hoc, after seeing the per-bag widths: descriptive only
        "corr_W_abs_angle_posthoc": bm._corr(np.abs(ang), w),
        "slope_mrad_per_rad": float(np.polyfit(ang, w, 1)[0]),
        "order": [list(bags)[i] for i in order],
    }
    return out


# --- study, report --------------------------------------------------------------


def study(verbose: bool = True) -> dict:
    """All stages, in order. The validation absolute encoder is read after (a)."""
    t0 = time.time()
    sessions = bm.load_sessions()
    roles = bm.role_sets(sessions)
    s_t, s_v = roles["training"][0], roles["validation"][0]
    run_t, run_v = bm.read_regressors(s_t), bm.read_regressors(s_v)
    jds_t = joint_data(run_t, bm.read_targets(s_t, JOINTS), URDF, "training")
    train = training_stage(jds_t, run_t)
    if verbose:
        print_training(train)
    targets_v = bm.read_targets(s_v, JOINTS, unlock_validation=True)
    jds_v = joint_data(run_v, targets_v, URDF, "validation")
    val = validation_stage(train, jds_t, jds_v, run_v)
    ver = verdict_stage(train, val, jds_t, jds_v)
    insp = inspection_stage(
        train, ver["joints"]["arm_6_joint"]["V2"]["W_val_mrad"], run_t
    )
    if verbose:
        print_validation(val)
        print_verdicts(ver)
        print_inspection(insp)
    return {
        "label": LABEL,
        "training": train,
        "validation": val,
        "verdicts": ver,
        "inspection": insp,
        "seconds": round(time.time() - t0, 1),
    }


def strip(x):
    """JSON-ready copy: private keys (live objects) dropped, arrays to lists."""
    if isinstance(x, dict):
        return {
            str(k): strip(v)
            for k, v in x.items()
            if not (isinstance(k, str) and k.startswith("_"))
        }
    if isinstance(x, (list, tuple)):
        return [strip(v) for v in x]
    if isinstance(x, np.ndarray):
        return strip(x.tolist())
    if isinstance(x, (np.floating, float)):
        return _f(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def print_training(train) -> None:
    print(f"\ntraining {train['session']} (M*: E, W, L; SE upper bounds)")
    for j, r in train["joints"].items():
        m = r["models"]["M3"]
        print(
            f"  {j:12s} W {m['W_mrad']:+7.2f} ({m['W_se_mrad']:.2f})"
            f"  G {m.get('G_mrad', 0):+7.2f} ({m.get('G_se_mrad', 0):.2f})"
            f"  kappa {m.get('kappa_mrad_per_nm', 0):+7.2f} ({m.get('kappa_se_mrad_per_nm', 0):.2f})"
            f"  rmse {m['rmse_mrad']:.2f}  selected {r['selection']['groups']}"
        )


def print_validation(val) -> None:
    print(f"\nvalidation {val['session']} RMSE mrad (offset re-anchored)")
    print("  joint        B0     B1     B2     M*   selected  outside")
    for j, r in val["joints"].items():
        e = r["rmse_mrad"]
        print(
            f"  {j:12s} {e['B0']:6.2f} {e['B1']:6.2f} {e['B2']:6.2f} {e['M*']:6.2f}"
            f" {e['selected']:7.2f}   {r['outside_fraction']:.2f}"
        )


def print_verdicts(ver) -> None:
    print(f"\nverdicts ({ver['label']})")
    for j, r in ver["joints"].items():
        fl = ",".join(k for k, v in r["flags"].items() if v) or "-"
        print(
            f"  {j:12s} flags {fl:12s} V1 {r['V1']['verdict']} (g {r['V1']['g_mrad']:+.2f}, b {r['V1']['b_v1_mrad']:.2f});"
            f" V2 {r['V2']['verdict']} (dW {r['V2']['dW_mrad']:+.2f}, band {r['V2']['band_w_mrad']:.2f});"
            f" V3 {r['V3']['verdict']}"
        )


def print_inspection(i) -> None:
    print("\njoint inspection (descriptive, Hey5 not verified)")
    print(
        f"  arm_6 W train {i['model_W_train_mrad']['E+W']:.1f} (E+W), "
        f"{i['model_W_train_mrad']['all_groups']:.1f} (M*), val {i['model_W_val_mrad']:.1f} mrad"
    )
    for sid, p in i["probes"].items():
        print(
            f"  probe {p['probe_mrad']} (abs p2p {p['abs_p2p_mrad']:.1f}): W/probe {p['W_over_probe']:.2f}"
        )
    s = i["arm5_summary"]
    print(
        f"  arm_5 bags {s['bags']}: W {s['W_mean_mrad']:.1f} +- {s['W_sd_mrad']:.1f} mrad,"
        f" corr with angle {s['corr_W_angle']:+.2f} (|angle|, post hoc {s['corr_W_abs_angle_posthoc']:+.2f}), slope {s['slope_mrad_per_rad']:+.1f} mrad/rad"
    )


def golden(res: dict) -> dict:
    """What the tests and the note quote."""
    r = strip(res)
    r.pop("seconds", None)
    return r


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--archive", action="store_true", help="write the run record")
    parser.add_argument("--root", default="results/runs", help="archive root")
    parser.add_argument("--out", type=Path, help="write the results as JSON")
    parser.add_argument("--golden", type=Path, help="write the golden file")
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    res = study()
    code = 0
    if args.archive:
        code |= archive(res, args)
    if args.out:
        args.out.write_text(json.dumps(strip(res), indent=1) + "\n")
    if args.golden:
        args.golden.write_text(json.dumps(golden(res), indent=1, sort_keys=True) + "\n")
    return code


def report_html(r: dict) -> str:
    rows = "".join(
        f"<tr><td>{j}</td><td>{v['V1']['verdict']}</td><td>{v['V2']['verdict']}</td>"
        f"<td>{v['V3']['verdict']}</td></tr>"
        for j, v in r["verdicts"]["joints"].items()
    )
    return (
        "<html><body><h1>TIAGo backlash surface (#71)</h1>"
        f"<p>Verdicts are {r['label']}: no confirmation set.</p>"
        "<table border=1><tr><th>joint</th><th>V1</th><th>V2</th><th>V3</th></tr>"
        f"{rows}</table></body></html>"
    )


class _Carrier:
    """What the run archive reads from a calibration object (no solver is run)."""

    def __init__(self, res: dict, started: str):
        names = [f"W_{j}" for j in JOINTS]
        vals = [res["training"]["joints"][j]["models"]["M3"]["W_mrad"] for j in JOINTS]
        self.robot_name = SETTINGS["robot"]
        self.calib_config = {"robot_name": SETTINGS["robot"], "param_name": names}
        self.results_data = {"calibrated parameters values": vals}
        self._config_file_path = str(bm.CONFIG)
        self.robot = SimpleNamespace(robot_urdf=str(bm.URDF_DIR / URDF))
        from figaroh.tools.stages import StageResult

        m = res["training"]["joints"]["arm_6_joint"]["models"]["M3"]
        self.stages = [
            StageResult("data", "ok", "protocol sha256 verified"),
            StageResult(
                "fit",
                "ok",
                "linear least squares per joint, block-bootstrap SE",
                metrics={"rmse_arm_6": {"value": m["rmse_mrad"], "unit": "mrad"}},
            ),
            StageResult("export", "ok"),
        ]
        self._run_started_at = started


def archive(res, args) -> int:
    """Run record: heldout, corrections, verdict, provenance, reproduction."""
    from datetime import datetime, timezone

    from figaroh.tools.provenance import collect_run_provenance
    from figaroh.tools.run_archive import archive_run, compute_run_dir

    from examples.run_record import (
        audit,
        describe_file,
        missing,
        write_reproduction_record,
    )

    r = strip(res)
    carrier = _Carrier(res, datetime.now(timezone.utc).isoformat())
    carrier._run_provenance = collect_run_provenance(carrier, "calibration")
    run_dir = compute_run_dir(carrier, root=args.root)
    heldout = {
        "primary": "RMSE of d on 2023-03-09 bottom, offset re-anchored (mrad)",
        "validation": r["validation"],
        "inspection": r["inspection"],
    }
    corrections = {
        "model": "d = c_s + E(q_abs) + (W/2) h + G tanh(tau_g/tau0) + kappa tau_g",
        "training": r["training"],
    }
    verdict = {
        "status": "complete",
        "selected_stage": "fit",
        "label": r["label"],
        "splits": {
            "validation_source": "none: nothing is selected on the validation run",
            "training": [r["training"]["session"]],
            "validation": [r["validation"]["session"]],
        },
        "verdicts": r["verdicts"],
        "bands": "tests/golden/tiago_backlash_truth.json (frozen)",
    }
    (run_dir / "report.html").write_text(report_html(r))
    for name, data in (
        ("heldout.json", heldout),
        ("corrections.json", corrections),
        ("verdict.json", verdict),
    ):
        (run_dir / name).write_text(json.dumps(data, indent=2, default=str))
    archive_run(carrier, run_dir)
    inputs = {}
    for sid, s in bm.load_sessions().items():
        for kind in ("names", "values"):
            path = getattr(s, kind)
            if path is not None:
                inputs[f"{sid}/{kind}"] = describe_file(path)
    for sid, path in inspection_files().items():
        inputs[f"inspection/{sid}"] = describe_file(path)
    inputs["protocol.yaml"] = describe_file(bm.PROTOCOL)
    inputs["inspection protocol.yaml"] = describe_file(JI / "protocol.yaml")
    inputs["config"] = describe_file(bm.CONFIG)
    inputs["frozen bands"] = describe_file(FROZEN)
    artifacts = {
        name: describe_file(run_dir / name)
        for name in ("heldout.json", "corrections.json", "verdict.json")
    }
    write_reproduction_record(
        run_dir,
        processing={
            "command": "backlash_study.py " + " ".join(sys.argv[1:]),
            "model": "d = c_s + E(q_abs) + (W/2) h + G tanh(tau_g/tau0) + kappa tau_g",
            "estimation": "linear least squares, unit-norm columns, block bootstrap",
            "primary_model": "all groups (E, W, L)",
            "primary_metric": "validation RMSE of d, offset re-anchored",
        },
        inputs=inputs,
        artifacts=artifacts,
    )
    checks = audit(run_dir)
    print(f"\nRun archived to {run_dir}")
    return 1 if missing(checks) else 0


if __name__ == "__main__":
    sys.exit(main())
