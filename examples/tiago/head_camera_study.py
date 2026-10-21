"""TIAGo head camera vs motion capture (#101): the study (part 2).

The model, the metrics, the bands and the verdict rules are the pre-registered
ones (``head_camera.py``, ``head_camera_truth.py``, the bands in
``tests/golden/tiago_head_camera_truth.json``, ``docs/development/tiago-head-camera.md``
sections 3 to 8). Nothing here is tuned on any session.

The order is fixed, and the confirmation values (2023-10-27) are read only in
the last stage:

(a) ``training``: the gauge-fixed fit ``G = {d_arm_1 .. d_arm_6, X, Y}`` with a
    depth scale ``k`` (F4: ``k`` is in) on the 2023-11-27 ``center`` session,
    6D whitened (primary) and position only (3D, secondary);
(b) ``baselines``: B0 (nominal offsets, ``X`` and ``Y`` refitted), B1 (PAL
    offsets, ``X`` and ``Y`` refitted), B1' (PAL as published, ``Y`` refitted)
    and the trained model, scored on the validation session (2023-11-23
    ``distortion_original``) by leave-one-out (LOO) re-registration. The PAL
    sign convention (``q + d`` or ``q - d``) is decided on the training
    session only;
(c) ``confirmation``: unlocked after (a) and (b). C0 sensor agreement and the
    not-usable rule 1, C1 parameters per run and sensor against the F5 bands,
    C2 cross-run transfer gaps against the transfer bands, the secondary G of
    the frozen 11-27 model, the baselines on the confirmation runs, the
    fixture standard errors (rule 2) and rule 3; then the verdicts;
(d) ``diagnostics``: 2023-11-07 camera postures and 2023-11-27 ``topleft``,
    descriptive only.

    python head_camera_study.py --archive --no-plot
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
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import head_camera as hc  # noqa: E402

FROZEN = project_root / "tests/golden/tiago_head_camera_truth.json"
SETTINGS = hc.SETTINGS
LEVEL = float(SETTINGS["noise_level_mm"])
SIG_C = hc.sigma_vector(LEVEL)
SIG_M = hc.mocap_sigma_vector(LEVEL)
SIG_CM = np.sqrt(SIG_C**2 + SIG_M**2)  # C0: the two sensors' noise together
I4 = np.eye(4)
BORDERLINE = 0.35  # five seeds: a band is uncertain by about 35 %
ARM_NAMES = [f"arm_{i}" for i in range(1, 7)]
C0_RESIDUAL_FACTOR = 3.0  # rule 1: residual above 3x the simulated floor
C0_FRAME_FACTOR = 2.0  # rule 1: frames off their mean by more than 2x recovery
DIAGNOSTICS = ("2023-11-07-camera", "2023-11-27-topleft")
MODELS = ("B0", "B1", "B1p", "trained")


# --- verdict rules ------------------------------------------------------------


def classify(ratio: float, borderline: float = BORDERLINE) -> str:
    """``beyond`` / ``borderline`` / ``within`` for ``abs(gap) / band``."""
    if ratio > 1.0 + borderline:
        return "beyond"
    if ratio >= 1.0 - borderline:
        return "borderline"
    return "within"


def verdict_text(classes: List[str]) -> str:
    """One verdict from the class of each item.

    Any item beyond its band: the camera is not shown to agree with mocap; else
    any borderline: borderline; else the camera is consistent with mocap.
    """
    if "beyond" in classes:
        return "camera not consistent with mocap for arm calibration (beyond band)"
    if "borderline" in classes:
        return "borderline"
    return "camera consistent with mocap for arm calibration"


def frozen() -> dict:
    """The frozen bands of the used level and the C0 floor (simulated)."""
    g = json.loads(FROZEN.read_text())
    return g["levels"][str(LEVEL)]["f5"]


def _f(x) -> Optional[float]:
    x = float(x)
    return x if np.isfinite(x) else None


def _lst(a) -> List[float]:
    return [float(v) for v in np.asarray(a).ravel()]


# --- PAL ----------------------------------------------------------------------


def pal_offsets10(sign: float = 1.0) -> np.ndarray:
    """PAL joint offsets in ``hc.JOINTS`` order (torso 0: PAL gives none), times ``sign``."""
    o = hc.pal_reference()["offsets"]
    return sign * np.array([o.get(j, 0.0) for j in hc.JOINTS])


def pal_camera_x(kin: hc.Kinematics) -> np.ndarray:
    """PAL's camera pose as the frame ``X`` of the optical frame (B1').

    PAL's ``camera_position`` / ``camera_orientation`` replace the origin of
    ``xtion_joint`` (``head_2_link`` to ``xtion_link``). ``X`` is the correction
    of the optical frame: ``X = T_opt^-1 T_xj^-1 T_pal T_opt``.
    """
    cam = hc.pal_reference()["camera"]
    r, p, y = cam["rpy"]
    T_pal = I4.copy()
    T_pal[:3, :3] = Rotation.from_euler("xyz", [r, p, y]).as_matrix()
    T_pal[:3, 3] = cam["xyz"]
    _, F_h2 = kin.frame_in_joint("head_2_link")
    _, F_x = kin.frame_in_joint("xtion_link")
    _, F_o = kin.frame_in_joint(hc.CAMERA_FRAME)
    T_xj = np.linalg.inv(F_h2) @ F_x
    T_opt = np.linalg.inv(F_x) @ F_o
    return np.linalg.inv(T_opt) @ np.linalg.inv(T_xj) @ T_pal @ T_opt


# --- scoring ------------------------------------------------------------------


def fixed_x_loo(
    kin: hc.Kinematics,
    obs: hc.Obs,
    off: np.ndarray,
    sigma: np.ndarray,
    X: np.ndarray,
    scale: bool = True,
) -> np.ndarray:
    """B1': LOO errors (n, 6) with ``X`` fixed; ``Y`` (and ``k``) refitted.

    The offsets and ``X`` are kept; ``Y = Y0 hom(y)`` and the depth scale are
    refitted on the other postures and the held-out posture is scored.
    """
    Tc, Ta = kin.poses(obs.Q, off)
    Y0 = hc.register_board(kin, obs, X, off)
    n = len(obs)

    def solve(keep):
        sub = obs.take(keep)

        def fun(t):
            M = hc.predict(
                Tc[keep],
                Ta[keep],
                X,
                Y0 @ hc.hom(t[:6]),
                1.0 + (t[6] if scale else 0.0),
            )
            return (hc.raw_residuals(M, sub) / sigma).ravel()

        t0 = np.zeros(7 if scale else 6)
        return least_squares(fun, t0, method="lm", xtol=1e-12, ftol=1e-12).x

    out = np.empty((n, 6))
    for i in range(n):
        t = solve(np.arange(n) != i)
        M = hc.predict(
            Tc[i : i + 1],
            Ta[i : i + 1],
            X,
            Y0 @ hc.hom(t[:6]),
            1.0 + (t[6] if scale else 0.0),
        )
        out[i] = hc.raw_residuals(M, obs.take([i]))[0]
    return out


def score(kin, obs, models: dict, sigma, X0=I4, scale=True) -> Dict[str, dict]:
    """LOO re-registered RMSE of each model on ``obs``: position mm, rotation mrad."""
    out = {}
    for name, m in models.items():
        if m.get("X") is not None:
            e = fixed_x_loo(kin, obs, m["off"], sigma, m["X"], scale)
        else:
            e = hc.loo_errors(kin, obs, m["off"], sigma, X0, scale=scale)
        mm, mrad = hc.rmse(e)
        out[name] = {"pos_mm": mm, "rot_mrad": mrad}
    return out


# --- (a) training -------------------------------------------------------------


def fit_record(kin, f: hc.Fit, obs: hc.Obs) -> dict:
    """Parameters, standard errors, conditioning and residual RMSE of a fit."""
    se = f.se()
    res = hc.rmse(hc.raw_residuals(hc.predict_fit(kin, f, obs.Q), obs))
    return {
        "names": list(f.names),
        "theta": _lst(f.theta),
        "se": _lst(se),
        "offsets_mrad": _lst(1e3 * f.offsets[list(hc.FIT_OFFSETS)]),
        "offsets_se_mrad": _lst(1e3 * se[: len(hc.FIT_OFFSETS)]),
        "k": float(f.k),
        "cond": _f(f.cond),
        "rank": f.rank(),
        "n_params": len(f.theta),
        "n_rows": int(f.n_rows),
        "status": int(f.status),
        "rmse_pos_mm": res[0],
        "rmse_rot_mrad": res[1],
        "chi2": float(f.cost),
    }


def training_stage(kin, sessions) -> dict:
    """(a): the gauge-fixed fit with ``k`` on the training session, 6D and 3D."""
    s = hc.role_sets(sessions)["training"][0]
    obs = hc.read_camera(s)
    f6 = hc.fit(kin, obs, hc.Spec(scale=True), SIG_C)
    f3 = hc.fit(kin, obs, hc.Spec(rows="3d", scale=True), SIG_C)
    rec6, rec3 = fit_record(kin, f6, obs), fit_record(kin, f3, obs)
    loo = hc.loo_errors(kin, obs, f6.offsets, SIG_C, I4, scale=True)
    rec6["loo_pos_mm"], rec6["loo_rot_mrad"] = hc.rmse(loo)
    return {
        "session": s.id,
        "n": len(obs),
        "level_mm": LEVEL,
        "fit6d": rec6,
        "fit3d": rec3,
        "_fit": f6,
        "_obs": obs,
    }


# --- (b) baselines ------------------------------------------------------------


def decide_pal_sign(kin, obs_train) -> dict:
    """PAL's sign (``q + sign d``) on the training session only.

    Rule, fixed before the run: the sign whose B1 (PAL offsets, ``X`` and ``Y``
    refitted) has the lower LOO position RMSE on the training session; B1' is
    reported for both signs and does not decide.
    """
    rows = {}
    for sign in (1.0, -1.0):
        off = pal_offsets10(sign)
        b1 = score(kin, obs_train, {"B1": {"off": off}}, SIG_C)["B1"]
        b1p = score(
            kin, obs_train, {"B1p": {"off": off, "X": pal_camera_x(kin)}}, SIG_C
        )["B1p"]
        rows[f"{sign:+.0f}"] = {"B1": b1, "B1p": b1p}
    chosen = min((1.0, -1.0), key=lambda s: rows[f"{s:+.0f}"]["B1"]["pos_mm"])
    return {
        "rule": "lower B1 LOO position RMSE on the training session",
        "rows": rows,
        "sign": chosen,
    }


def model_set(kin, trained_off, sign) -> Dict[str, dict]:
    """B0, B1, B1' and the trained model as ``{name: {off, X?}}``."""
    return {
        "B0": {"off": np.zeros(10)},
        "B1": {"off": pal_offsets10(sign)},
        "B1p": {"off": pal_offsets10(sign), "X": pal_camera_x(kin)},
        "trained": {"off": np.asarray(trained_off)},
    }


def baselines_stage(kin, sessions, train) -> dict:
    """(b): the PAL sign on training, then all four models on validation."""
    sign = decide_pal_sign(kin, train["_obs"])
    models = model_set(kin, train["_fit"].offsets, sign["sign"])
    val = hc.role_sets(sessions)["validation"][0]
    obs = hc.read_camera(val)
    return {
        "pal_sign": sign,
        "models": models,
        "validation": {
            "session": val.id,
            "n": len(obs),
            "scores": score(kin, obs, models, SIG_C),
        },
        "training_scores": score(kin, train["_obs"], models, SIG_C),
    }


# --- (c) confirmation ---------------------------------------------------------


def read_runs(sessions) -> Dict[str, dict]:
    """The confirmation runs: camera (all postures) and the paired camera / mocap rows."""
    runs = {}
    for s in sorted(hc.role_sets(sessions)["confirmation"], key=lambda s: s.id):
        cam = hc.read_camera(s, allow_confirmation=True)
        plan = pd.read_csv(s.joints, usecols=["posture"])["posture"].to_numpy()
        rows = hc.paired_rows(s)
        mc = pd.read_csv(s.mocap).sort_values("posture")
        if mc["posture"].tolist() != plan[rows].tolist():
            raise ValueError(f"{s.id}: mocap postures do not match the paired rows")
        mobs = hc.Obs(
            cam.Q[rows],
            hc.from_rpy(mc[["phix", "phiy", "phiz"]].to_numpy()),
            mc[["x", "y", "z"]].to_numpy(),
        )
        runs[s.id] = {"cam_all": cam, "cam": cam.take(rows), "mc": mobs, "rows": rows}
    return runs


def frame_dev(T: np.ndarray, Tmean: np.ndarray) -> np.ndarray:
    """6-vector (m, rad): translation difference and body-frame rotation vector to the mean."""
    dR = Rotation.from_matrix(Tmean[:3, :3].T @ T[:3, :3]).as_rotvec()
    return np.r_[T[:3, 3] - Tmean[:3, 3], dR]


def mean_frame(Ts: List[np.ndarray]) -> np.ndarray:
    """Mean of frames: mean translation, chordal mean rotation."""
    u, _, vt = np.linalg.svd(sum(T[:3, :3] for T in Ts))
    R = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    out = I4.copy()
    out[:3, :3], out[:3, 3] = R, np.mean([T[:3, 3] for T in Ts], axis=0)
    return out


def c0_stage(runs, bands) -> dict:
    """C0: ``M_cam = X' M_mc Y'`` per run (no kinematics) and the not-usable rule 1."""
    c0 = bands["c0"]
    out = {"runs": {}}
    fr = {}
    for rid, r in runs.items():
        a = hc.axb_agreement(r["cam"].T, r["mc"].T, SIG_CM)
        fr[rid] = a
        rms = a["rms_per_axis"]
        out["runs"][rid] = {
            "n": len(r["cam"]),
            "Xp": _lst(hc.vec(a["X"])),
            "Yp": _lst(hc.vec(a["Y"])),
            "rms_pos_mm": _lst(1e3 * rms[:3]),
            "rms_rot_mrad": _lst(1e3 * rms[3:]),
            "cost": a["cost"],
        }
    floor_pos, floor_rot = np.array(c0["floor_pos_mm"]), np.array(c0["floor_rot_mrad"])
    ids = list(runs)
    res_ratio = {
        rid: _lst(
            np.r_[
                np.array(out["runs"][rid]["rms_pos_mm"]) / floor_pos,
                np.array(out["runs"][rid]["rms_rot_mrad"]) / floor_rot,
            ]
        )
        for rid in ids
    }
    # frames: each component of X', Y' against the mean of the runs
    tol = {
        "X": C0_FRAME_FACTOR
        * np.array([c0["x_pos_mm"] * 1e-3] * 3 + [c0["x_rot_mrad"] * 1e-3] * 3),
        "Y": C0_FRAME_FACTOR
        * np.array([c0["y_pos_mm"] * 1e-3] * 3 + [c0["y_rot_mrad"] * 1e-3] * 3),
    }
    dev = {}
    for key in ("X", "Y"):
        mean = mean_frame([fr[i][key] for i in ids])
        dev[key] = {i: frame_dev(fr[i][key], mean) for i in ids}
    frame_ratio = {
        key: {i: _lst(np.abs(dev[key][i]) / tol[key]) for i in ids} for key in dev
    }
    worst_res = max(max(v) for v in res_ratio.values())
    worst_frame = max(max(max(v) for v in frame_ratio[k].values()) for k in frame_ratio)
    out.update(
        {
            "residual_over_floor": res_ratio,
            "frame_dev_over_tol": frame_ratio,
            "frame_dev": {
                k: {i: _lst(v) for i, v in d.items()} for k, d in dev.items()
            },
            "floor": {"pos_mm": _lst(floor_pos), "rot_mrad": _lst(floor_rot)},
            "worst_residual_ratio": float(worst_res),
            "worst_frame_ratio": float(worst_frame),
            "residual_ok": bool(worst_res <= C0_RESIDUAL_FACTOR),
            "frames_ok": bool(worst_frame <= 1.0),
            "rule1_pass": bool(worst_res <= C0_RESIDUAL_FACTOR and worst_frame <= 1.0),
            "_frames": fr,
        }
    )
    return out


def run_fits(kin, runs, c0, scale=True) -> dict:
    """G per run and sensor on the paired rows (camera with ``k``, mocap without).

    The mocap frame ``X`` of the body starts at ``X_cam X'`` (C0): only a start, the fit is free.
    """
    out = {}
    for rid, r in runs.items():
        fc = hc.fit(kin, r["cam"], hc.Spec(scale=scale), SIG_C)
        x0m = fc.X @ c0["_frames"][rid]["X"]
        fm = hc.fit(kin, r["mc"], hc.Spec(), SIG_M, X0=x0m)
        out[rid] = {"cam": fc, "mc": fm, "x0_mc": x0m}
    return out


def c1_stage(fits, bands) -> dict:
    """C1: ``d_arm_1 .. d_arm_6`` per run and sensor against the F5 parameter bands."""
    band = {k: 1e3 * v for k, v in bands["parameter_band_rad"].items()}
    out = {"band_mrad": band, "runs": {}, "items": []}
    for rid, f in fits.items():
        cam = 1e3 * f["cam"].offsets[list(hc.FIT_OFFSETS)]
        mc = 1e3 * f["mc"].offsets[list(hc.FIT_OFFSETS)]
        sc = 1e3 * f["cam"].se()[:6]
        sm = 1e3 * f["mc"].se()[:6]
        rows = {}
        for j, name in enumerate(ARM_NAMES):
            gap = cam[j] - mc[j]
            ratio = abs(gap) / band[name]
            rows[name] = {
                "camera_mrad": float(cam[j]),
                "mocap_mrad": float(mc[j]),
                "camera_se_mrad": float(sc[j]),
                "mocap_se_mrad": float(sm[j]),
                "gap_mrad": float(gap),
                "band_mrad": band[name],
                "ratio": float(ratio),
                "class": classify(ratio),
            }
            out["items"].append(rows[name]["class"])
        out["runs"][rid] = {
            "rows": rows,
            "camera_status": f["cam"].status,
            "mocap_status": f["mc"].status,
            "camera_cond": _f(f["cam"].cond),
            "mocap_cond": _f(f["mc"].cond),
            "camera_k": float(f["cam"].k),
        }
    ids = list(fits)
    spread = {}
    for sensor in ("cam", "mc"):
        arr = np.array(
            [1e3 * fits[i][sensor].offsets[list(hc.FIT_OFFSETS)] for i in ids]
        )
        spread[sensor] = {
            n: float(np.std(arr[:, j], ddof=1)) for j, n in enumerate(ARM_NAMES)
        }
    out["run_to_run_sd_mrad"] = {"camera": spread["cam"], "mocap": spread["mc"]}
    out["verdict"] = verdict_text(out["items"])
    return out


def c2_stage(kin, runs, fits, bands, train_off) -> dict:
    """C2: transfer gaps of the six ordered pairs, both directions, and the secondary G."""
    ids = list(runs)
    tr = bands["transfer"]
    cache = {}

    def e(sensor, rid, model_off):
        key = (sensor, rid, tuple(np.round(model_off, 12)))
        if key not in cache:
            if sensor == "mc":
                err = hc.loo_errors(
                    kin, runs[rid]["mc"], model_off, SIG_M, fits[rid]["x0_mc"]
                )
            else:
                err = hc.loo_errors(
                    kin, runs[rid]["cam"], model_off, SIG_C, I4, scale=True
                )
            cache[key] = hc.rmse(err)
        return cache[key]

    pairs = []
    for r in ids:
        for rp in ids:
            if r == rp:
                continue
            offc, offm = fits[rp]["cam"].offsets, fits[rp]["mc"].offsets
            a, b = e("mc", r, offc), e("mc", r, offm)
            c, d = e("cam", r, offm), e("cam", r, offc)
            sec_m = e("mc", r, train_off)
            sec_c = e("cam", r, train_off)
            pairs.append(
                {
                    "evaluated": r,
                    "trained": rp,
                    "cam_to_mc": {"pos_mm": a[0] - b[0], "rot_mrad": a[1] - b[1]},
                    "mc_to_cam": {"pos_mm": c[0] - d[0], "rot_mrad": c[1] - d[1]},
                    "e_mc": {"cam_model": a, "mc_model": b},
                    "e_cam": {"mc_model": c, "cam_model": d},
                    "secondary": {
                        # frozen 11-27 model against the 10-27 models of run r'
                        "cam_to_mc": {
                            "pos_mm": sec_m[0] - b[0],
                            "rot_mrad": sec_m[1] - b[1],
                        },
                        "cam_to_cam": {
                            "pos_mm": sec_c[0] - d[0],
                            "rot_mrad": sec_c[1] - d[1],
                        },
                    },
                }
            )
    items = []
    for p in pairs:
        for direction in ("cam_to_mc", "mc_to_cam"):
            for metric in ("pos_mm", "rot_mrad"):
                band = tr[direction][metric]["band"]
                gap = p[direction][metric]
                ratio = abs(gap) / band
                p[direction][f"{metric}_band"] = band
                p[direction][f"{metric}_ratio"] = ratio
                p[direction][f"{metric}_class"] = classify(ratio)
                items.append(p[direction][f"{metric}_class"])
    summary = {}
    for direction in ("cam_to_mc", "mc_to_cam"):
        for metric in ("pos_mm", "rot_mrad"):
            g = np.array([p[direction][metric] for p in pairs])
            cls = [p[direction][f"{metric}_class"] for p in pairs]
            summary[f"{direction}|{metric}"] = {
                "band": tr[direction][metric]["band"],
                "mean_gap": float(g.mean()),
                "max_abs_gap": float(np.abs(g).max()),
                "beyond": cls.count("beyond"),
                "borderline": cls.count("borderline"),
                "within": cls.count("within"),
            }
    sec = {}
    for key in ("cam_to_mc", "cam_to_cam"):
        for metric in ("pos_mm", "rot_mrad"):
            g = np.array([p["secondary"][key][metric] for p in pairs])
            sec[f"{key}|{metric}"] = {"mean": float(g.mean()), "values": _lst(g)}
    return {
        "pairs": pairs,
        "summary": summary,
        "secondary_11_27": sec,
        "items": items,
        "verdict": verdict_text(items),
    }


def confirmation_baselines(kin, runs, models, fits) -> dict:
    """B0, B1, B1' and the trained 11-27 model on the confirmation runs (camera, all rows)."""
    out = {}
    for rid, r in runs.items():
        out[rid] = {
            "camera": score(kin, r["cam_all"], models, SIG_C),
            "mocap": score(
                kin, r["mc"], models, SIG_M, X0=fits[rid]["x0_mc"], scale=False
            ),
        }
    return out


def fixture_se_stage(bands) -> dict:
    """Rule 2: standard errors of the fixture's fits at the 10-27 paired plan.

    The truth fixture of the pre-registration (seeds 0 to 4, the used level):
    camera (with ``k``) and mocap twin fitted on each run's paired postures;
    the mean standard error of ``d_arm_2 .. d_arm_6`` per run and sensor.
    """
    from examples.tiago import head_camera_truth as ht

    fx = ht.Fixture()
    band = {k: 1e3 * v for k, v in bands["parameter_band_rad"].items()}
    se = {"cam": {}, "mc": {}}
    for seed in ht.SEEDS:
        truth = fx.draw_truth(seed)
        cam = fx.camera_obs(seed, truth, LEVEL)
        mc = fx.mocap_obs(seed, truth, LEVEL)
        for s in fx.conf:
            fc = hc.fit(
                fx.kin, cam[s.id].take(fx.rows[s.id]), hc.Spec(scale=True), SIG_C
            )
            fm = hc.fit(fx.kin, mc[s.id], hc.Spec(), SIG_M)
            for key, f in (("cam", fc), ("mc", fm)):
                se[key].setdefault(s.id, []).append(1e3 * f.se()[1:6])
    out = {"se_mrad": {}, "triggered": {}}
    for key in se:
        for rid, v in se[key].items():
            m = np.mean(v, axis=0)
            out["se_mrad"][f"{key}|{rid}"] = _lst(m)
            bad = [
                bool(m[j] > 10.0 or m[j] > 0.5 * band[f"arm_{j + 2}"]) for j in range(5)
            ]
            out["triggered"][f"{key}|{rid}"] = {"over": bad, "count": int(sum(bad))}
    out["any_run_ge3"] = bool(any(v["count"] >= 3 for v in out["triggered"].values()))
    out["rule2_pass"] = not out["any_run_ge3"]
    return out


def confirmation_stage(kin, sessions, train, base, bands) -> dict:
    """(c): unlocks the confirmation values; C0, C1, C2, baselines, rules and verdicts."""
    runs = read_runs(sessions)
    min_n = hc.MIN_POSTURES
    rule3 = {rid: len(r["cam"]) >= min_n for rid, r in runs.items()}
    c0 = c0_stage(runs, bands)
    fits = run_fits(kin, runs, c0)
    c1 = c1_stage(fits, bands)
    c2 = c2_stage(kin, runs, fits, bands, train["_fit"].offsets)
    fx = fixture_se_stage(bands)
    nok = {}
    for rid, r in runs.items():
        f = hc.fit(kin, r["cam"], hc.Spec(), SIG_C)
        nok[rid] = {
            "arm_mrad": _lst(1e3 * f.offsets[list(hc.FIT_OFFSETS)]),
            "gap_to_mocap_mrad": _lst(
                1e3 * (f.offsets - fits[rid]["mc"].offsets)[list(hc.FIT_OFFSETS)]
            ),
        }
    usable = bool(c0["rule1_pass"] and fx["rule2_pass"] and all(rule3.values()))
    combined = verdict_text(c1["items"] + c2["items"])
    final = combined if usable else "not answerable with this data"
    return {
        "runs": {
            rid: {"n_camera": len(r["cam_all"]), "n_paired": len(r["cam"])}
            for rid, r in runs.items()
        },
        "rules": {
            "rule1_c0": c0["rule1_pass"],
            "rule2_fixture_se": fx["rule2_pass"],
            "rule3_min_postures": rule3,
            "usable": usable,
        },
        "c0": {k: v for k, v in c0.items() if not k.startswith("_")},
        "c1": c1,
        "c2": c2,
        "fixture_se": fx,
        "no_k_sensitivity": nok,
        "baselines": confirmation_baselines(kin, runs, base["models"], fits),
        "verdict": {
            "c1": c1["verdict"],
            "c2": c2["verdict"],
            "combined": combined,
            "final": final,
        },
        "_fits": fits,
        "_runs": runs,
        "_c0": c0,
    }


# --- (d) diagnostics ----------------------------------------------------------


def diagnostics_stage(kin, sessions, base) -> dict:
    """(d): the four models on 11-07 postures and on 11-27 topleft (descriptive)."""
    out = {}
    for sid in DIAGNOSTICS:
        s = sessions[sid]
        if s.role != "diagnostic":
            raise PermissionError(f"{sid} is {s.role}")
        obs = hc.read_camera(s)
        out[sid] = {"n": len(obs), "scores": score(kin, obs, base["models"], SIG_C)}
    return out


# --- stages and report --------------------------------------------------------


def study(verbose: bool = True) -> dict:
    """All stages, in order. The confirmation values are read in stage (c) only."""
    t0 = time.time()
    kin = hc.Kinematics()
    sessions = hc.load_sessions()
    bands = frozen()
    train = training_stage(kin, sessions)
    base = baselines_stage(kin, sessions, train)
    if verbose:
        print_training(train, base)
    conf = confirmation_stage(kin, sessions, train, base, bands)
    diag = diagnostics_stage(kin, sessions, base)
    res = {
        "level_mm": LEVEL,
        "training": train,
        "baselines": base,
        "confirmation": conf,
        "diagnostics": diag,
        "seconds": round(time.time() - t0, 1),
    }
    if verbose:
        print_confirmation(conf)
        print_diagnostics(diag)
    return res


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
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


def print_training(train, base) -> None:
    print(f"\ntraining {train['session']} ({train['n']} postures), level {LEVEL} mm")
    for tag in ("fit6d", "fit3d"):
        r = train[tag]
        print(
            f"  {tag}: rmse {r['rmse_pos_mm']:.2f} mm, {r['rmse_rot_mrad']:.1f} mrad,"
            f" cond {r['cond']:.1f}, rank {r['rank']}/{r['n_params']}, k {r['k']:.4f}"
        )
        for n, v, s in zip(ARM_NAMES, r["offsets_mrad"], r["offsets_se_mrad"]):
            print(f"    d_{n} {v:+7.2f} +- {s:5.2f} mrad")
    s = base["pal_sign"]
    print(f"  PAL sign {s['sign']:+.0f} ({s['rule']})")
    for k, v in s["rows"].items():
        print(
            f"    {k}: B1 {v['B1']['pos_mm']:.2f} mm, B1' {v['B1p']['pos_mm']:.2f} mm"
        )
    print(f"validation {base['validation']['session']} ({base['validation']['n']})")
    for m, v in base["validation"]["scores"].items():
        print(f"  {m:8s} {v['pos_mm']:6.2f} mm {v['rot_mrad']:7.1f} mrad")


def print_confirmation(c) -> None:
    print("\nconfirmation (2023-10-27)")
    c0 = c["c0"]
    print(
        f"  C0 worst residual / floor {c0['worst_residual_ratio']:.2f}, "
        f"worst frame dev / tol {c0['worst_frame_ratio']:.2f}, rule 1 {'pass' if c0['rule1_pass'] else 'FAIL'}"
    )
    for rid, r in c0["runs"].items():
        print(
            f"    {rid}: rms {np.round(r['rms_pos_mm'], 2)} mm {np.round(r['rms_rot_mrad'], 1)} mrad"
        )
    print(
        f"  rule 2 {'pass' if c['rules']['rule2_fixture_se'] else 'FAIL'}; rule 3 {c['rules']['rule3_min_postures']}"
    )
    print("  C1 (camera - mocap, mrad; band)")
    for rid, r in c["c1"]["runs"].items():
        print(
            f"    {rid}: "
            + " ".join(
                f"{n[-1]}:{v['gap_mrad']:+.1f}/{v['band_mrad']:.1f}{v['class'][0]}"
                for n, v in r["rows"].items()
            )
        )
    print("  C2")
    for k, v in c["c2"]["summary"].items():
        print(
            f"    {k:22s} band {v['band']:.2f} mean {v['mean_gap']:+.2f} max|g| {v['max_abs_gap']:.2f}"
            f"  beyond {v['beyond']} borderline {v['borderline']} within {v['within']}"
        )
    print(f"  verdict: C1 {c['verdict']['c1']}; C2 {c['verdict']['c2']}")
    print(f"  final: {c['verdict']['final']}")


def print_diagnostics(d) -> None:
    print("\ndiagnostics (descriptive)")
    for sid, v in d.items():
        print(
            f"  {sid} ({v['n']}): "
            + ", ".join(
                f"{m} {x['pos_mm']:.2f} mm/{x['rot_mrad']:.1f} mrad"
                for m, x in v["scores"].items()
            )
        )


def golden(res: dict) -> dict:
    """What the tests and the note quote."""
    r = strip(res)
    r.pop("seconds", None)
    r["baselines"].pop("models", None)
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
    """A small HTML table of the headline numbers."""
    rows = "".join(
        f"<tr><td>{m}</td><td>{v['pos_mm']:.2f}</td><td>{v['rot_mrad']:.1f}</td></tr>"
        for m, v in r["baselines"]["validation"]["scores"].items()
    )
    return (
        "<html><body><h1>TIAGo head camera vs mocap (#101)</h1>"
        f"<p>{r['confirmation']['verdict']['final']}</p>"
        "<table border=1><tr><th>model</th><th>mm</th><th>mrad</th></tr>"
        f"{rows}</table></body></html>"
    )


class _Carrier:
    """What the run archive reads from a calibration object (no solver is run)."""

    def __init__(self, res: dict, started: str):
        names = ["d_" + n for n in ARM_NAMES]
        rec = res["training"]["fit6d"]
        self.robot_name = hc.SETTINGS["robot"]
        self.calib_config = {
            "robot_name": hc.SETTINGS["robot"],
            "param_name": names,
        }
        self.results_data = {"calibrated parameters values": rec["offsets_mrad"]}
        self._config_file_path = str(hc.CONFIG)
        self.robot = SimpleNamespace(robot_urdf=str(hc.URDF))
        from figaroh.tools.stages import StageResult

        self.stages = [
            StageResult("data", "ok", "protocol sha256 verified"),
            StageResult(
                "fit",
                "ok" if rec["status"] > 0 else "failed",
                f"least_squares status {rec['status']}",
                metrics={
                    "rmse_pos": {"value": rec["rmse_pos_mm"], "unit": "mm"},
                    "rmse_rot": {"value": rec["rmse_rot_mrad"], "unit": "mrad"},
                },
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
    conf, train, base = r["confirmation"], r["training"], r["baselines"]
    heldout = {
        "primary": "leave-one-out re-registered RMSE (position mm, rotation mrad)",
        "validation": base["validation"],
        "confirmation_baselines": conf["baselines"],
        "c2": conf["c2"]["summary"],
        "diagnostics": r["diagnostics"],
    }
    corrections = {
        "gauge": {"estimated": "arm_1..arm_6 offsets, X, Y, depth scale k"},
        "training": {k: train[k] for k in ("session", "n", "fit6d", "fit3d")},
        "pal_sign": base["pal_sign"],
        "c1": conf["c1"],
    }
    verdict = {
        "status": "complete" if conf["rules"]["usable"] else "not usable",
        "selected_stage": "fit",
        "splits": {
            "validation_source": "none: nothing is selected on any session",
            "training": [train["session"]],
            "validation": [base["validation"]["session"]],
            "confirmation": sorted(conf["runs"]),
        },
        "rules": conf["rules"],
        "verdicts": conf["verdict"],
        "bands": "tests/golden/tiago_head_camera_truth.json (frozen)",
    }
    (run_dir / "report.html").write_text(report_html(r))
    for name, data in (
        ("heldout.json", heldout),
        ("corrections.json", corrections),
        ("verdict.json", verdict),
    ):
        (run_dir / name).write_text(json.dumps(data, indent=2, default=str))
    archive_run(carrier, run_dir)
    sessions = hc.load_sessions()
    inputs = {}
    for sid, s in sessions.items():
        for kind in ("camera", "joints", "mocap"):
            path = getattr(s, kind)
            if path is not None:
                inputs[f"{sid}/{kind}"] = describe_file(path)
    inputs["protocol.yaml"] = describe_file(hc.PROTOCOL)
    inputs["config"] = describe_file(hc.CONFIG)
    inputs["frozen bands"] = describe_file(FROZEN)
    artifacts = {
        name: describe_file(run_dir / name)
        for name in ("heldout.json", "corrections.json", "verdict.json")
    }
    write_reproduction_record(
        run_dir,
        processing={
            "command": "head_camera_study.py " + " ".join(sys.argv[1:]),
            "model": "joint_offset arm_1..arm_6, two-chain relative pose, X, Y, k",
            "estimation": "whitened 6D least squares; LOO re-registration",
            "primary_metric": "leave-one-out re-registered RMSE",
            "noise_level_mm": LEVEL,
            "min_postures": hc.MIN_POSTURES,
        },
        inputs=inputs,
        artifacts=artifacts,
    )
    checks = audit(run_dir)
    print(f"\nRun archived to {run_dir}")
    return 1 if missing(checks) else 0


if __name__ == "__main__":
    sys.exit(main())
