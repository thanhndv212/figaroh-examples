"""TIAGo head camera (#101): what the gauge and the noise alone explain.

A simulated fixture. One truth per seed (joint offsets of the whole head and
arm, the camera frame ``X``, a board frame ``Y`` per session) is measured at
the *real* postures of the training (2023-11-27 center), validation (11-23)
and confirmation (10-27, 11/11/8 paired postures) sessions. The camera is
simulated with the grid noise (sigma_z of ``noise_grid_mm``, the other axes by
the frozen ratios); the mocap twin has its own synthetic frames and noise.
Criteria are fixed in ``config/tiago_head_camera_config.yaml`` before any run.

- F1 gauge: the naive 22-parameter set has rank 19; the null space is the
  analytic one (torso, head_2 with X, arm_7 with Y); head_1 + arm_1 is a
  near-null direction with no rotation signature; the naive fit's head_2 and
  arm_7 are arbitrary while its predictions are unchanged.
- F2 fixed fit: the gauge-fixed fit recovers arm_2 .. arm_6 and
  arm_1 - head_1 (z-scores) and predicts a held-out session.
- F3 held head: PAL-sized head offsets in the truth, the model holds them at 0.
- F4 depth scale: a depth-scale parameter ``k`` enters only if it removes a
  bias larger than one standard error without worsening the conditioning by
  more than twice.
- F5 camera vs mocap twin at the paired 10-27 postures: parameter bands
  (``2 RMS_s(theta_cam - theta_mc)``, arm_1 .. arm_6) and transfer bands
  (``|mean G| + 2 sd G``) for both directions; the C0 sensor-agreement floor.
- F6 descriptive: a 5 mrad arm_5 slip between test1 and test2/test3.

RNG: ``default_rng(seed)`` for the truth; noise ``default_rng(10000 + 100 k +
seed)``, ``k = 0`` camera, ``1`` mocap, ``2`` depth scale. Only simulated
measurements are fitted; the real sessions enter through their postures, the
pairing index of the 10-27 runs, and the registration-only frame of the
training session (``Y0``).

    python head_camera_truth.py --out bands.json
    python head_camera_truth.py --golden tests/golden/tiago_head_camera_truth.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from scipy.linalg import subspace_angles
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import head_camera as hc  # noqa: E402

SETTINGS = hc.SETTINGS
FX = SETTINGS["fixture"]
SEEDS = tuple(FX["seeds"])
GRID_MM = hc.GRID_MM
USED_MM = None  # set from the noise stage in main (the golden records it)
STREAMS = {"camera": 0, "mocap": 1, "depth": 2}
COMPARABLE = ["arm_2", "arm_3", "arm_4", "arm_5", "arm_6", "arm_1_minus_head_1"]
I4 = np.eye(4)


def stream(kind: str, seed: int) -> np.random.Generator:
    """Noise stream of a sensor: independent of the truth and of the other sensors."""
    return np.random.default_rng(10_000 + 100 * STREAMS[kind] + seed)


def perturbation(rng, mm: float, mrad: float) -> np.ndarray:
    """6-vector: ``mm`` translation and ``mrad`` rotation, random directions."""
    out = np.empty(6)
    for sl, mag in ((slice(0, 3), mm * 1e-3), (slice(3, 6), mrad * 1e-3)):
        d = rng.normal(size=3)
        out[sl] = mag * d / np.linalg.norm(d)
    return out


@dataclass
class Truth:
    """``off``: (10,) offsets in ``hc.JOINTS`` order; ``X``; ``Y`` per session id."""

    off: np.ndarray
    X: np.ndarray
    Y: Dict[str, np.ndarray]


class Fixture:
    """Postures of the real sessions and the registration-only board frame."""

    def __init__(self):
        self.kin = hc.Kinematics()
        sessions = hc.load_sessions()
        roles = hc.role_sets(sessions)
        self.train = roles["training"][0]
        self.val = roles["validation"][0]
        self.conf = sorted(roles["confirmation"], key=lambda s: s.id)
        self.order = [self.train, self.val] + self.conf
        self.Q = {s.id: hc.read_plan(s) for s in self.order}
        self.rows = {s.id: hc.paired_rows(s) for s in self.conf}
        # the one real-data computation of the fixture: the training session's
        # registration at the nominal joints (the pre-registration's noise stage
        # does the same on it), the common starting point of the board frames
        self.Y0 = hc.register_board(
            self.kin, hc.read_camera(self.train), I4, np.zeros(10)
        )

    def draw_truth(self, seed: int, head: Optional[List[float]] = None) -> Truth:
        """Truth of a seed. ``head`` replaces the head_1, head_2 offsets (F3)."""
        rng = np.random.default_rng(seed)
        off = rng.normal(0.0, FX["truth_offset_sd_rad"], 10)
        if head is not None:
            off[1:3] = head
        xp, yp = FX["x_perturb"], FX["y_perturb"]
        X = hc.hom(perturbation(rng, xp["mm"], xp["mrad"]))
        Y = {
            s.id: self.Y0 @ hc.hom(perturbation(rng, yp["mm"], yp["mrad"]))
            for s in self.order
        }
        return Truth(off, X, Y)

    def clean(self, session_id: str, truth: Truth, off=None, X=None, Y=None):
        """Noise-free board poses (n, 4, 4) of a session under ``truth``."""
        off = truth.off if off is None else off
        Tc, Ta = self.kin.poses(self.Q[session_id], off)
        return hc.predict(
            Tc,
            Ta,
            truth.X if X is None else X,
            truth.Y[session_id] if Y is None else Y,
        )

    def measure(self, M: np.ndarray, session_id: str, sigma, rng, scale=1.0):
        """Noisy ``Obs`` of the board poses ``M`` (depth scaled by ``scale``)."""
        n = len(M)
        p = M[:, :3, 3].copy()
        p[:, 2] *= scale
        p += sigma[:3] * rng.normal(size=(n, 3))
        w = sigma[3:] * rng.normal(size=(n, 3))
        R = Rotation.from_rotvec(-w).as_matrix() @ M[:, :3, :3]
        return hc.Obs(self.Q[session_id], R, p)

    def camera_obs(self, seed: int, truth: Truth, level_mm: float, scale=None):
        """Camera observations of every session (dict by id); ``scale``: {id: depth scale}."""
        rng = stream("camera", seed)
        sigma = hc.sigma_vector(level_mm)
        return {
            s.id: self.measure(
                self.clean(s.id, truth), s.id, sigma, rng, (scale or {}).get(s.id, 1.0)
            )
            for s in self.order
        }

    def mocap_obs(self, seed: int, truth: Truth, level_mm: float, slip=None):
        """Mocap-twin observations of the confirmation runs (paired rows)."""
        rng = stream("mocap", seed)
        sigma = hc.mocap_sigma_vector(level_mm)
        xm = truth.X @ hc.hom(np.array(FX["x_mocap"]))
        out = {}
        for s in self.conf:
            ym = truth.Y[s.id] @ hc.hom(np.array(FX["y_mocap"]))
            off = truth.off if slip is None else slip[s.id]
            M = self.clean(s.id, truth, off=off, X=xm, Y=ym)
            out[s.id] = self.measure(M, s.id, sigma, rng).take(self.rows[s.id])
        return out


# --- F1: gauge --------------------------------------------------------------


def orbit_direction(fx: Fixture, frame: str, joint_index: int, X, h=1e-6):
    """Parameter direction (offset of one joint, frame ``X`` moved with it) of no effect.

    ``T(q + e) = T_joint(q) Rz(e) F`` for a revolute z joint, so the frame
    parameter ``F^-1 Rz(-e) F X`` keeps the pose: the group orbit.
    """
    _, F = fx.kin.frame_in_joint(frame)

    def x_of(e):
        Rz = hc.hom(np.array([0, 0, 0, 0, 0, -e]))
        return hc.vec(np.linalg.inv(X) @ np.linalg.inv(F) @ Rz @ F @ X)

    v = np.zeros(22)
    v[joint_index] = 1.0
    return v, (x_of(h) - x_of(-h)) / (2 * h)


def f1_cell(fx: Fixture, seed: int, level_mm: float, workdir=None) -> dict:
    """Rank, null space and near-null of the naive set on every session, and the naive fit."""
    crit = FX["f1"]
    truth = fx.draw_truth(seed)
    spec = hc.Spec(hc.ALL_OFFSETS)
    sigma = hc.sigma_vector(level_mm)
    theta = np.r_[truth.off, np.zeros(12)]
    ranks, angles, near, conc, rotfrac = {}, [], [], [], []
    for s in fx.order:
        M = fx.clean(s.id, truth)
        obs = hc.Obs(fx.Q[s.id], M[:, :3, :3], M[:, :3, 3])
        J = hc.jacobian(fx.kin, obs, spec, theta, sigma, truth.X, truth.Y[s.id])
        u, sv, vt = np.linalg.svd(J)
        ranks[s.id] = int(np.sum(sv > crit["rank_rtol"] * sv[0]))
        null = vt[crit["rank"] :].T
        t = np.zeros((22, 3))
        t[0, 0] = 1.0
        v2, dx = orbit_direction(fx, hc.CAMERA_FRAME, 2, truth.X)
        v2[10:16] = dx
        v9, dy = orbit_direction(fx, hc.HAND_FRAME, 9, truth.Y[s.id])
        v9[16:22] = dy
        t[:, 1], t[:, 2] = v2, v9
        angles.append(float(np.max(subspace_angles(null, t))))
        near.append(float(sv[crit["rank"] - 1] / sv[0]))
        v = vt[crit["rank"] - 1]
        conc.append(float(v[1] ** 2 + v[3] ** 2))
        # rotation signature of the analytic direction head_1 + arm_1 (the SVD
        # vector carries small admixtures of other parameters)
        e = np.zeros(22)
        e[1] = e[3] = 1.0
        Je = (J @ e).reshape(-1, 6)
        rotfrac.append(float(np.linalg.norm(Je[:, 3:]) / np.linalg.norm(Je)))
    return {
        "ranks": ranks,
        "null_angle": float(max(angles)),
        "near_null": float(max(near)),
        "near_null_concentration": float(min(conc)),
        "near_null_rot_fraction": float(max(rotfrac)),
    }


def f1_naive_fit(fx: Fixture, seed: int, level_mm: float) -> dict:
    """Naive 22-parameter fit vs the gauge-fixed fit: standard errors and predictions."""
    truth = fx.draw_truth(seed)
    obs = fx.camera_obs(seed, truth, level_mm)
    sigma = hc.sigma_vector(level_mm)
    tr, val = obs[fx.train.id], obs[fx.val.id]
    fixed = hc.fit(fx.kin, tr, hc.Spec(), sigma)
    naive = hc.fit(fx.kin, tr, hc.Spec(hc.ALL_OFFSETS), sigma)
    se_f, se_n = fixed.se(), naive.se()
    names = naive.names
    out = {
        "se_ratio_head_2": float(se_n[names.index("d_head_2")] / np.median(se_f[:6])),
        "se_ratio_arm_7": float(se_n[names.index("d_arm_7")] / np.median(se_f[:6])),
    }
    loo = {}
    for tag, f in (("fixed", fixed), ("naive", naive)):
        e = hc.loo_errors(fx.kin, val, f.offsets, sigma, I4)
        loo[tag] = hc.rmse(e)
    out["loo_fixed_mm"], out["loo_naive_mm"] = loo["fixed"][0], loo["naive"][0]
    out["loo_diff_mm"] = abs(loo["naive"][0] - loo["fixed"][0])
    return out


# --- F2 / F3 / F4: the fixed fit --------------------------------------------


def comparable(params: Dict[str, float]) -> Dict[str, float]:
    return {
        **{f"arm_{i}": params[f"d_arm_{i}"] for i in range(2, 7)},
        "arm_1_minus_head_1": params["d_arm_1"],
    }


def truth_comparable(truth: Truth) -> Dict[str, float]:
    o = truth.off
    return {
        **{f"arm_{i}": float(o[hc.ARM[0] + i - 1]) for i in range(2, 7)},
        "arm_1_minus_head_1": float(o[3] - o[1]),
    }


def val_error_mm(fx: Fixture, truth: Truth, f, val_obs, sigma) -> float:
    """RMS position error (mm) of the re-registered prediction at the validation
    postures against the noise-free truth."""
    r = hc.reregister(fx.kin, val_obs, f.offsets, sigma, I4)
    pred = hc.predict_fit(fx.kin, r, fx.Q[fx.val.id])
    true = fx.clean(fx.val.id, truth)
    return float(
        np.sqrt(np.mean(np.sum((pred[:, :3, 3] - true[:, :3, 3]) ** 2, axis=1))) * 1e3
    )


def fixed_cell(fx: Fixture, seed: int, level_mm: float, head=None) -> dict:
    """F2 (random head) or F3 (``head`` PAL-sized): the gauge-fixed fit on training."""
    truth = fx.draw_truth(seed, head)
    obs = fx.camera_obs(seed, truth, level_mm)
    sigma = hc.sigma_vector(level_mm)
    f = hc.fit(fx.kin, obs[fx.train.id], hc.Spec(), sigma)
    est, tru = comparable(f.params()), truth_comparable(truth)
    se = dict(zip(f.names, f.se()))
    se_of = {
        **{f"arm_{i}": se[f"d_arm_{i}"] for i in range(2, 7)},
        "arm_1_minus_head_1": se["d_arm_1"],
    }
    z = {k: (est[k] - tru[k]) / se_of[k] for k in COMPARABLE}
    return {
        "seed": seed,
        "z": z,
        "error_rad": {k: est[k] - tru[k] for k in COMPARABLE},
        "se_rad": se_of,
        "val_error_mm": val_error_mm(fx, truth, f, obs[fx.val.id], sigma),
        "cond": f.cond,
    }


def depth_cell(fx: Fixture, seed: int, level_mm: float) -> dict:
    """F4: training depth scaled by ``s``; fit without and with ``k``."""
    s = FX["depth_scale"][0] / FX["depth_scale"][1]
    truth = fx.draw_truth(seed)
    obs = fx.camera_obs(seed, truth, level_mm, scale={fx.train.id: s})
    sigma = hc.sigma_vector(level_mm)
    out = {"seed": seed, "s": s}
    for tag, spec in (("nok", hc.Spec()), ("k", hc.Spec(scale=True))):
        f = hc.fit(fx.kin, obs[fx.train.id], spec, sigma)
        se = dict(zip(f.names, f.se()))
        est = comparable(f.params())
        tru = truth_comparable(truth)
        out[tag] = {
            "error_rad": {k: est[k] - tru[k] for k in COMPARABLE},
            "se_rad": {
                **{f"arm_{i}": se[f"d_arm_{i}"] for i in range(2, 7)},
                "arm_1_minus_head_1": se["d_arm_1"],
            },
            "cond": f.cond,
            "k": f.k,
        }
    return out


# --- F5 / F6: camera and mocap twin on the 10-27 postures -------------------


def slip_offsets(fx: Fixture, truth: Truth, slip: float) -> Dict[str, np.ndarray]:
    """Per-run truth offsets: arm_5 shifted by ``slip`` from test2 on."""
    out = {}
    for i, s in enumerate(fx.conf):
        off = truth.off.copy()
        if i > 0:
            off[hc.ARM[0] + 4] += slip
        out[s.id] = off
    return out


def pair_cell(fx: Fixture, seed: int, level_mm: float, slip: float = 0.0) -> dict:
    """Camera and mocap twin on the paired 10-27 postures: parameters, transfer, C0."""
    truth = fx.draw_truth(seed)
    offs = slip_offsets(fx, truth, slip)
    sig_c, sig_m = hc.sigma_vector(level_mm), hc.mocap_sigma_vector(level_mm)
    rng = stream("camera", seed)
    cam, mc = {}, {}
    mrng = stream("mocap", seed)
    xm = truth.X @ hc.hom(np.array(FX["x_mocap"]))
    for s in fx.conf:
        rows = fx.rows[s.id]
        Mc = fx.clean(s.id, truth, off=offs[s.id])
        cam[s.id] = fx.measure(Mc, s.id, sig_c, rng).take(rows)
        ym = truth.Y[s.id] @ hc.hom(np.array(FX["y_mocap"]))
        Mm = fx.clean(s.id, truth, off=offs[s.id], X=xm, Y=ym)
        mc[s.id] = fx.measure(Mm, s.id, sig_m, mrng).take(rows)
    ids = [s.id for s in fx.conf]
    fits = {
        "cam": {i: hc.fit(fx.kin, cam[i], hc.Spec(), sig_c) for i in ids},
        "mc": {i: hc.fit(fx.kin, mc[i], hc.Spec(), sig_m) for i in ids},
    }
    # parameters: delta a1 .. a6 per run, camera minus mocap
    par = {
        i: (fits["cam"][i].offsets - fits["mc"][i].offsets)[
            list(hc.FIT_OFFSETS)
        ].tolist()
        for i in ids
    }
    # transfer: model of run r' scored on run r (LOO re-registered), both directions
    g = {"cam_to_mc": [], "mc_to_cam": []}
    e_cache = {}

    def e(obs_key, run_obs, sig, model_off, model_key):
        key = (obs_key, model_key)
        if key not in e_cache:
            e_cache[key] = hc.rmse(hc.loo_errors(fx.kin, run_obs, model_off, sig, I4))
        return e_cache[key]

    for r in ids:
        for rp in ids:
            if r == rp:
                continue
            # camera model on mocap rows of r, vs the mocap model (own sensor)
            a = e(("mc", r), mc[r], sig_m, fits["cam"][rp].offsets, ("cam", rp))
            b = e(("mc", r), mc[r], sig_m, fits["mc"][rp].offsets, ("mc", rp))
            g["cam_to_mc"].append([a[0] - b[0], a[1] - b[1]])
            a = e(("cam", r), cam[r], sig_c, fits["mc"][rp].offsets, ("mc", rp))
            b = e(("cam", r), cam[r], sig_c, fits["cam"][rp].offsets, ("cam", rp))
            g["mc_to_cam"].append([a[0] - b[0], a[1] - b[1]])
    # C0: M_cam = X' M_mc Y' per run, no kinematics
    c0 = {"x_err": [], "y_err": [], "rms": []}
    Xp = np.linalg.inv(truth.X) @ xm
    for s in fx.conf:
        i = s.id
        Mcam = cam[i].T
        Mmc = mc[i].T
        sig = np.sqrt(sig_c**2 + sig_m**2)
        r = hc.axb_agreement(
            Mcam, Mmc, sig, X0=Xp, Y0=np.linalg.inv(hc.hom(np.array(FX["y_mocap"])))
        )
        Yp = np.linalg.inv(truth.Y[i] @ hc.hom(np.array(FX["y_mocap"]))) @ truth.Y[i]
        c0["x_err"].append(hc.vec(np.linalg.inv(Xp) @ r["X"]).tolist())
        c0["y_err"].append(hc.vec(np.linalg.inv(Yp) @ r["Y"]).tolist())
        c0["rms"].append(r["rms_per_axis"].tolist())
    return {
        "seed": seed,
        "slip": slip,
        "par_diff_rad": par,
        "g": g,
        "c0": c0,
        "da5": {
            m: float(
                fits[m][ids[1]].offsets[hc.ARM[0] + 4]
                - fits[m][ids[0]].offsets[hc.ARM[0] + 4]
            )
            for m in ("cam", "mc")
        },
    }


# --- summaries --------------------------------------------------------------


def band(g: np.ndarray) -> float:
    return float(abs(np.mean(g)) + 2 * np.std(g, ddof=1))


def z_stats(cells) -> dict:
    z = np.array([[c["z"][k] for k in COMPARABLE] for c in cells])
    return {
        "rms": float(np.sqrt(np.mean(z**2))),
        "mean": float(np.mean(z)),
        "per_param_rms": {
            k: float(np.sqrt(np.mean(z[:, j] ** 2))) for j, k in enumerate(COMPARABLE)
        },
    }


def summarize_f4(cells, baseline=None) -> dict:
    """The depth-scale rule: bias of the no-k fit vs its SE, and the conditioning."""
    bias = {
        k: float(np.mean([c["nok"]["error_rad"][k] for c in cells])) for k in COMPARABLE
    }
    se = {k: float(np.mean([c["nok"]["se_rad"][k] for c in cells])) for k in COMPARABLE}
    ratio = {k: abs(bias[k]) / se[k] for k in COMPARABLE}
    bias_k = {
        k: float(np.mean([c["k"]["error_rad"][k] for c in cells])) for k in COMPARABLE
    }
    cond_nok = float(np.mean([c["nok"]["cond"] for c in cells]))
    cond_k = float(np.mean([c["k"]["cond"] for c in cells]))
    crit = FX["f4"]
    biased = max(ratio.values()) > crit["bias_se_min"]
    cond_ok = cond_k < crit["cond_ratio_max"] * cond_nok
    # the same statistic without any depth scale (the F2 cells): the level of
    # |mean error| / SE that noise alone gives over these seeds
    base = None
    if baseline is not None:
        base = {
            k: abs(float(np.mean([c["error_rad"][k] for c in baseline])))
            / float(np.mean([c["se_rad"][k] for c in baseline]))
            for k in COMPARABLE
        }
    return {
        "bias_over_se_without_scale": base,
        "bias_over_se": ratio,
        "bias_k_over_se": {k: abs(bias_k[k]) / se[k] for k in COMPARABLE},
        "cond_nok": cond_nok,
        "cond_k": cond_k,
        "k_hat": float(np.mean([c["k"]["k"] for c in cells])),
        "s": cells[0]["s"],
        "biased": bool(biased),
        "cond_ok": bool(cond_ok),
        "k_in": bool(biased and cond_ok),
    }


def summarize_pairs(cells) -> dict:
    ids = list(cells[0]["par_diff_rad"])
    d = np.array([[c["par_diff_rad"][i] for i in ids] for c in cells])  # seed, run, 6
    out = {
        "parameter_band_rad": {
            f"arm_{j + 1}": float(2 * np.sqrt(np.mean(d[:, :, j] ** 2)))
            for j in range(6)
        }
    }
    out["transfer"] = {}
    for key in ("cam_to_mc", "mc_to_cam"):
        g = np.array([c["g"][key] for c in cells]).reshape(-1, 2)
        out["transfer"][key] = {
            "pos_mm": {
                "mean": float(g[:, 0].mean()),
                "sd": float(g[:, 0].std(ddof=1)),
                "band": band(g[:, 0]),
            },
            "rot_mrad": {
                "mean": float(g[:, 1].mean()),
                "sd": float(g[:, 1].std(ddof=1)),
                "band": band(g[:, 1]),
            },
        }
    x = np.array([c["c0"]["x_err"] for c in cells]).reshape(-1, 6)
    y = np.array([c["c0"]["y_err"] for c in cells]).reshape(-1, 6)
    rms = np.array([c["c0"]["rms"] for c in cells]).reshape(-1, 6)
    out["c0"] = {
        "x_pos_mm": float(np.sqrt(np.mean(np.sum(x[:, :3] ** 2, axis=1))) * 1e3),
        "x_rot_mrad": float(np.sqrt(np.mean(np.sum(x[:, 3:] ** 2, axis=1))) * 1e3),
        "y_pos_mm": float(np.sqrt(np.mean(np.sum(y[:, :3] ** 2, axis=1))) * 1e3),
        "y_rot_mrad": float(np.sqrt(np.mean(np.sum(y[:, 3:] ** 2, axis=1))) * 1e3),
        "floor_pos_mm": [
            float(v) for v in np.sqrt(np.mean(rms[:, :3] ** 2, axis=0)) * 1e3
        ],
        "floor_rot_mrad": [
            float(v) for v in np.sqrt(np.mean(rms[:, 3:] ** 2, axis=0)) * 1e3
        ],
    }
    return out


def grade_f1(cells, naive) -> dict:
    crit = FX["f1"]
    ok = {
        "rank": all(r == crit["rank"] for c in cells for r in c["ranks"].values()),
        "null_space": max(c["null_angle"] for c in cells) < crit["null_angle_max"],
        "near_null": max(c["near_null"] for c in cells) < crit["near_null_max"],
        "near_null_concentration": min(c["near_null_concentration"] for c in cells)
        > crit["near_null_concentration_min"],
        "no_rotation_signature": max(c["near_null_rot_fraction"] for c in cells)
        < crit["near_null_rot_fraction_max"],
        "naive_se": min(min(n["se_ratio_head_2"], n["se_ratio_arm_7"]) for n in naive)
        > crit["naive_se_ratio_min"],
    }
    return ok


def run(seeds=SEEDS, levels=GRID_MM, used_mm: float = None, verbose=True) -> dict:
    """The whole fixture: F1-F6 on every seed at every grid level (F6 at the used level)."""
    fx = Fixture()
    used = float(used_mm if used_mm is not None else SETTINGS["noise_level_mm"])
    out = {"seeds": list(seeds), "levels": list(levels), "used_mm": used, "grid": {}}
    for lv in levels:
        if verbose:
            print(f"level {lv} mm", flush=True)
        f1 = [f1_cell(fx, s, lv) for s in seeds]
        naive = [f1_naive_fit(fx, s, lv) for s in seeds]
        f2 = [fixed_cell(fx, s, lv) for s in seeds]
        f3 = [fixed_cell(fx, s, lv, head=FX["pal_head_rad"]) for s in seeds]
        f4 = [depth_cell(fx, s, lv) for s in seeds]
        f5 = [pair_cell(fx, s, lv) for s in seeds]
        cell = {
            "f1": {
                "cells": f1,
                "naive": naive,
                "pass": grade_f1(f1, naive),
                "loo_diff_mm": float(max(n["loo_diff_mm"] for n in naive)),
            },
            "f2": {
                **z_stats(f2),
                "val_error_mm": float(np.mean([c["val_error_mm"] for c in f2])),
                "cond": float(np.mean([c["cond"] for c in f2])),
            },
            "f3": {
                **z_stats(f3),
                "val_error_mm": float(np.mean([c["val_error_mm"] for c in f3])),
                "val_error_vs_f2_mm": float(
                    np.mean(
                        [
                            abs(a["val_error_mm"] - b["val_error_mm"])
                            for a, b in zip(f3, f2)
                        ]
                    )
                ),
            },
            "f4": summarize_f4(f4, f2),
            "f5": summarize_pairs(f5),
            "cells": {"f2": f2, "f3": f3, "f4": f4, "f5": f5},
        }
        if lv == used:
            f6 = [pair_cell(fx, s, lv, slip=FX["arm5_slip_rad"]) for s in seeds]
            f6s = summarize_pairs(f6)
            cell["f6"] = {
                "da5_cam_mrad": float(np.mean([c["da5"]["cam"] for c in f6]) * 1e3),
                "da5_mc_mrad": float(np.mean([c["da5"]["mc"] for c in f6]) * 1e3),
                "slip_mrad": FX["arm5_slip_rad"] * 1e3,
                "transfer": f6s["transfer"],
                "parameter_band_rad": f6s["parameter_band_rad"],
                "transfer_without_slip": cell["f5"]["transfer"],
            }
        out["grid"][str(lv)] = cell
    return out


def grade(res: dict) -> dict:
    """Pass/fail of F1-F3 and the F4 decision at the used level."""
    c = res["grid"][str(res["used_mm"])]
    f2c, f3c = FX["f2"], FX["f3"]
    lv = res["used_mm"]
    return {
        "F1": all(c["f1"]["pass"].values())
        and c["f1"]["loo_diff_mm"] < FX["f1"]["prediction_tol_ratio"] * lv,
        "F2": f2c["z_rms"][0] <= c["f2"]["rms"] <= f2c["z_rms"][1]
        and abs(c["f2"]["mean"]) < f2c["z_mean_max"]
        and c["f2"]["val_error_mm"] < f2c["validation_error_ratio_max"] * lv,
        "F3": f3c["z_rms"][0] <= c["f3"]["rms"] <= f3c["z_rms"][1]
        and abs(c["f3"]["mean"]) < f3c["z_mean_max"]
        and c["f3"]["val_error_vs_f2_mm"] < f3c["prediction_tol_ratio"] * lv,
        "F4_k_in": c["f4"]["k_in"],
    }


def golden(res: dict, noise: dict) -> dict:
    """What the tests and the note quote: thresholds per level, and seed 0 in full."""
    used = str(res["used_mm"])
    levels = {}
    for lv, c in res["grid"].items():
        levels[lv] = {
            "f1": {"pass": c["f1"]["pass"], "loo_diff_mm": c["f1"]["loo_diff_mm"]},
            "f2": {k: c["f2"][k] for k in ("rms", "mean", "val_error_mm", "cond")},
            "f3": {
                k: c["f3"][k]
                for k in ("rms", "mean", "val_error_mm", "val_error_vs_f2_mm")
            },
            "f4": c["f4"],
            "f5": c["f5"],
        }
    s0 = res["grid"][used]["cells"]
    return {
        "used_mm": res["used_mm"],
        "seeds": res["seeds"],
        "noise": noise,
        "criteria": grade(res),
        "levels": levels,
        "f6": res["grid"][used]["f6"],
        "seed0": {
            "f1": res["grid"][used]["f1"]["cells"][0],
            "f2": s0["f2"][0],
            "f3": s0["f3"][0],
            "f4": s0["f4"][0],
            "f5": s0["f5"][0],
        },
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--golden", type=Path)
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    res = run(tuple(args.seeds))
    verdict = grade(res)
    print(json.dumps(verdict))
    if args.out:
        args.out.write_text(json.dumps(res, indent=1, default=float) + "\n")
    if args.golden:
        noise = hc.noise_estimates()
        args.golden.write_text(
            json.dumps(golden(res, noise), indent=1, sort_keys=True, default=float)
            + "\n"
        )


if __name__ == "__main__":
    main()
