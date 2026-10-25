"""TALOS torso-arm mocap calibration: frozen held-out protocol (#172).

Fits every model on the training run (2022-03-15 13:54, left arm) and
reports it on the two right-arm runs (14:18 validation, 14:39 confirmation)
and on five row folds of the training run. Roles, files and sha256 are fixed
in data/calibration/mocap/protocol.yaml (v2) and protocol_1354_kfold.yaml,
read through examples/calibration_harness; the design and the pre-registered
criteria are in docs/development/talos-mocap.md.

Measurement: the end-effector body in the waist body frame (6D),
``inverse(waist) * endeffector``, modelled as ``B * FK(q) * T * S_k``:

- ``B``: base_link in the waist body frame (core's base frame, 6D);
- ``FK``: base_link to arm_<side>_7_link, nominal or with the fitted errors;
- ``T``: the end-effector body on arm_<side>_7_link (core's tool frame, 6D);
- ``S_k``: the flip repair of row cluster ``k`` (identity for the main
  cluster, :func:`flip_clusters`), a rotation ``R_k`` about a centroid ``c``
  shared by the session, estimated with the registration.

Models (all on the training run; the joint parameters of the left arm and
the torso):

- ``registration only``: ``B`` and ``T``, nominal joints (the baseline);
- ``joint_offset``: joint-angle offsets, structural (ordered);
- ``full_params``: placement errors, structural (ordered);
- ``full map`` / ``full map_cv``: every placement error with core's priors.

Metrics (position in mm, orientation in deg):

- training run, k-fold: the fold's own frames carried to its held rows
  (same session, same mounting): selects the estimation method;
- right-arm runs: the model's torso parameters carried over, the right arm
  nominal; ``B``, ``T`` and the flip repair refitted on the run
  (``PRIMARY_METRIC``), or only ``T`` and the repair (``SECONDARY_METRIC``,
  ``B`` from training: the same body on the same link).

Run from examples/talos.
"""

from __future__ import annotations

import contextlib
import functools
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml
from scipy.optimize import least_squares

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.base_calibration import BaseCalibration  # noqa: E402
from figaroh.calibration.calibration_tools import (  # noqa: E402
    calc_updated_fkm,
    drop_calibration_parameters,
)
from figaroh.calibration.parameter import BASE_TPL  # noqa: E402
from figaroh.data.observations import PoseObservations  # noqa: E402
from figaroh.data.source import DataSource  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402
from figaroh.tools.stages import record_stage  # noqa: E402

from examples.calibration_harness import heldout as harness  # noqa: E402

TALOS = Path(__file__).resolve().parent
MOCAP = TALOS / "data" / "calibration" / "mocap"
PROTOCOL = MOCAP / "protocol.yaml"
KFOLD_PROTOCOL = MOCAP / "protocol_1354_kfold.yaml"
CONFIG = TALOS / "config" / "talos_mocap_config.yaml"
URDF = TALOS / "urdf" / "talos_full_v2.urdf"
PACKAGE_DIRS = str(TALOS.parents[1] / "models")

TORSO = ("torso_1_joint", "torso_2_joint")
#: metres per radian of the orientation residuals: 0.17 mm of position
#: noise per axis against about 1.6 mrad of rotation (the torso
#: registration of the training run), so both residuals carry about the same
LEVER = 0.1
#: a row more than this from its session's medoid orientation is flipped
FLIP_DEG = 45.0
#: a flip cluster with fewer rows is not repaired: its rotation (3) and the
#: shared centroid would absorb the row; its rows leave the metrics, listed
MIN_CLUSTER = 3
#: frozen with the protocol (the harness schema has no key for them)
PRIMARY_METRIC = "pos_rmse_rereg_mm"
SECONDARY_METRIC = "pos_rmse_waist_mm"
KFOLD_METRIC = "pos_rmse_mm"
#: torso_2 placement errors that neither the base frame nor the first arm
#: placement can absorb (they do not commute with the torso_2 rotation, nor
#: with torso_1's): the part of the torso one arm determines for the other
GAUGE_FREE_TORSO = ("d_px_torso_2_joint", "d_phix_torso_2_joint")

PLAN = harness.load_plan(PROTOCOL)  # verifies every file's sha256
KFOLD_PLAN = harness.load_plan(KFOLD_PROTOCOL)
_FOLD = PLAN.folds[0]
# role, file; training first, then the protocol's order
SETS = [(s.role, s.name) for s in (*_FOLD.training, *_FOLD.evaluated)]
TRAINING = SETS[0][1]
STRATA = PLAN.spec.strata
if tuple(STRATA.joints) != TORSO:
    raise ValueError(f"{PROTOCOL.name}: strata joints differ from TORSO")
ARM = {
    name: s["arm"]
    for s in yaml.safe_load(PROTOCOL.read_text())["sessions"]
    for name in s["files"]
}
_source = yaml.safe_load(CONFIG.read_text())["tasks"]["calibration"]["data"]
if Path(_source["source_file"]).name != TRAINING:
    raise ValueError("config source_file is not the protocol's training file")

STRUCTURAL = {"method": "structural", "structural_selection": "ordered"}
PRIORS = {
    "translation": 1e-3,
    "rotation": 2e-3,
    "joint_offset": 2e-2,
    "prismatic_offset": 2e-3,
}
# label -> (calibration level, frames_only, core estimation settings)
MODELS = {
    "registration only": ("full_params", True, STRUCTURAL),
    "joint_offset": ("joint_offset", False, STRUCTURAL),
    "full_params": ("full_params", False, STRUCTURAL),
    "full map": ("full_params", False, {"method": "map", "priors": PRIORS}),
    "full map_cv": ("full_params", False, {"method": "map_cv", "priors": PRIORS}),
}
BASELINE = "registration only"


# ── Poses ─────────────────────────────────────────────────────────────────────


def joint_names(arm: str) -> list:
    return [*TORSO, *(f"arm_{arm}_{i}_joint" for i in range(1, 8))]


def body_poses(df: pd.DataFrame, body: str) -> list:
    cols = [f"{body}_{c}" for c in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    return [pin.XYZQUATToSE3(r) for r in df[cols].to_numpy(dtype=float)]


def measured_poses(df: pd.DataFrame) -> list:
    """End-effector body in the waist body frame, one SE3 per row."""
    return [
        w.inverse() * e
        for w, e in zip(body_poses(df, "waist"), body_poses(df, "endeffector"))
    ]


def xyzrpy(M: pin.SE3) -> np.ndarray:
    return np.r_[M.translation, pin.rpy.matrixToRpy(M.rotation)]


def from_xyzrpy(v) -> pin.SE3:
    v = np.asarray(v, dtype=float)
    return pin.SE3(pin.rpy.rpyToMatrix(v[3:6]), v[:3])


def pose_errors(pred: list, meas: list) -> tuple:
    """(n,) position (mm) and orientation (deg) errors."""
    pos = np.array([1000 * np.linalg.norm(p.translation - m.translation)
                    for p, m in zip(pred, meas)])
    ori = np.array([np.degrees(np.linalg.norm(pin.log3(m.rotation.T @ p.rotation)))
                    for p, m in zip(pred, meas)])
    return pos, ori


def rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x**2))) if x.size else float("nan")


# ── Robot and calibrator ──────────────────────────────────────────────────────


@functools.lru_cache(maxsize=1)
def robot():
    with contextlib.redirect_stdout(io.StringIO()):
        return load_robot(str(URDF), package_dirs=PACKAGE_DIRS, load_by_urdf=True)


class TalosMocapCalibration(BaseCalibration):
    """TALOS torso-arm calibration from the waist-frame end-effector pose.

    ``load_data_set`` reads the shipped static-posture file (bodies in the
    Qualisys frame) instead of core's CSV layout; ``rows`` keeps a subset of
    its rows (a k-fold training part). The training rows must have no
    flipped orientation (:func:`flip_clusters`).
    """

    def __init__(self, robot, config_file=CONFIG, data_file=None, rows=None):
        super().__init__(robot, str(config_file))
        cfg = self.calib_config
        cfg["known_baseframe"] = False
        cfg["known_tipframe"] = False
        path = Path(data_file) if data_file else TALOS / cfg["data_file"]
        cfg["data_file"] = str(path)
        self._data_path = str(path.resolve())
        self.rows = None if rows is None else [int(i) for i in rows]

    def load_data_set(self):
        cfg = self.calib_config
        df = pd.read_csv(self._data_path)
        if self.rows is not None:
            df = df.iloc[self.rows]
        arm = "left" if "arm_left_1_joint" in df.columns else "right"
        if f"arm_{arm}_7_link" != cfg["end_frame"]:
            raise ValueError(f"{self._data_path}: {arm} arm, chain {cfg['end_frame']}")
        clusters = flip_clusters(self.model, df, arm)
        if np.any(clusters):
            raise ValueError(
                f"{self._data_path}: rows {np.flatnonzero(clusters).tolist()} "
                "are flipped; a training set must have none"
            )
        joints = [self.model.names[i] for i in cfg["actJoint_idx"]]
        values = np.array([xyzrpy(y) for y in measured_poses(df)])[:, None, :]
        self.observations = PoseObservations(
            joint_names=tuple(joints),
            q=df[joints].to_numpy(dtype=float),
            values=values,
            point_names=("1",),
            measurability=np.ones((1, 6), bool),
            frame="waist_frame",
            registered_to="base_link",
            orientation="rpy",
            sample_index=np.arange(len(df)),
            source=DataSource.from_files(
                [self._data_path], adapter="TalosMocapCalibration.load_data_set"
            ),
        )
        self.PEE_measured, self.q_measured = self.observations.to_legacy(
            self.model, cfg
        )
        cfg["NbSample"] = len(self.q_measured)
        record_stage(
            self, "data", "ok", "waist-frame end-effector poses",
            {"samples": len(self.q_measured)},
        )

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        """World-frame log-map residuals, rotation rows times ``LEVER``."""
        pee = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )
        res = self._compute_logmap_residuals(
            self.PEE_measured, pee, position_frame="world"
        )
        n = self.calib_config["NbSample"]
        res[3 * n:] *= LEVER
        return res


def is_frame(name: str) -> bool:
    return name in BASE_TPL or "EE" in name


def joint_of(name: str) -> str | None:
    """The joint a joint-level parameter belongs to (``None`` for frames)."""
    if is_frame(name):
        return None
    return name[name.index("_", 2 if name.startswith("d_") else 0) + 1:]


def fit(
    level: str,
    frames_only: bool = False,
    data_file=None,
    rows=None,
    estimation: dict | None = None,
    instance: dict | None = None,
) -> TalosMocapCalibration:
    """Fit on the training run (or ``data_file``, ``rows`` of it)."""
    calib = TalosMocapCalibration(robot(), CONFIG, data_file or MOCAP / TRAINING, rows)
    cfg = calib.calib_config
    if estimation is not None:
        cfg["estimation"] = dict(estimation)
    if instance:
        cfg["instance"] = dict(cfg.get("instance") or {}, **instance)
    cfg["calib_model"] = level
    cfg["validation_data_file"] = None  # no held-out at fit time
    with contextlib.redirect_stdout(io.StringIO()):
        calib.initialize()
        if frames_only:
            joints = [n for n in cfg["param_name"] if not is_frame(n)]
            drop_calibration_parameters(cfg, joints)
        calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


def fit_model(label: str, data_file=None, rows=None) -> TalosMocapCalibration:
    level, frames_only, estimation = MODELS[label]
    return fit(level, frames_only, data_file, rows, estimation)


def fitted_params(calib) -> dict:
    return dict(zip(calib.calib_config["param_name"], map(float, calib.LM_result.x)))


# ── Forward kinematics of either arm ──────────────────────────────────────────


def configurations(model, df: pd.DataFrame, arm: str) -> np.ndarray:
    q = np.tile(pin.neutral(model), (len(df), 1))
    for j in joint_names(arm):
        q[:, model.joints[model.getJointId(j)].idx_q] = df[j].to_numpy(dtype=float)
    return q


def chain_poses(calib, params: dict, q: np.ndarray, arm: str, level=None) -> list:
    """``B * FK * T`` of ``arm`` at configurations ``q``, as SE3.

    ``params`` may hold the parameters of any chain: only the frames and the
    joint-level parameters of ``arm``'s chain are applied (missing ones are
    nominal). ``level`` defaults to the calibrator's.
    """
    model = calib.model
    joints = set(joint_names(arm))
    keep = {n: v for n, v in params.items() if is_frame(n) or joint_of(n) in joints}
    cfg = dict(
        calib.calib_config,
        start_frame="universe",
        end_frame=f"arm_{arm}_7_link",
        actJoint_idx=sorted(model.getJointId(j) for j in joints),
        param_name=list(keep),
        calib_model=level or calib.calib_config["calib_model"],
        NbSample=len(q),
        measurability=[True] * 6,
        calibration_index=6,
        NbMarkers=1,
    )
    pee = calc_updated_fkm(model, calib.data, np.array(list(keep.values())), q, cfg)
    return [from_xyzrpy(v) for v in pee.reshape(6, -1).T]


# ── Flips and registration ────────────────────────────────────────────────────


def _angle_deg(Ra, Rb) -> float:
    return float(np.degrees(np.linalg.norm(pin.log3(Ra.T @ Rb))))


@functools.lru_cache(maxsize=1)
def _nominal_calibrator():
    """A calibrator for :func:`chain_poses` of the nominal model (either arm)."""
    calib = TalosMocapCalibration(robot(), CONFIG)
    calib.calib_config["calib_model"] = "full_params"
    return calib


def flip_clusters(model, df: pd.DataFrame, arm: str) -> np.ndarray:
    """(n,) cluster index per row: 0 for the main cluster, 1.. for flips.

    Fit-free: the rotation of the end-effector body relative to
    arm_<side>_7_link (nominal model, waist rotation taken as identity); the
    medoid is the row within ``FLIP_DEG`` of the most others (first on a
    tie), a row further than ``FLIP_DEG`` from it is flipped, and flipped
    rows within ``FLIP_DEG`` of one another (transitively) form a cluster,
    numbered by their first row.
    """
    calib = _nominal_calibrator()
    F = chain_poses(calib, {}, configurations(model, df, arm), arm, "full_params")
    R = [f.rotation.T @ y.rotation for f, y in zip(F, measured_poses(df))]
    n = len(R)
    D = np.array([[_angle_deg(a, b) for b in R] for a in R])
    medoid = int(np.argmax(np.sum(D < FLIP_DEG, axis=1)))
    labels = np.zeros(n, int)
    flipped = [i for i in range(n) if D[medoid, i] > FLIP_DEG]
    k = 0
    for i in flipped:
        if labels[i]:
            continue
        k += 1
        stack = [i]
        labels[i] = k
        while stack:
            j = stack.pop()
            for m in flipped:
                if not labels[m] and D[j, m] < FLIP_DEG:
                    labels[m] = k
                    stack.append(m)
    return labels


def repairable(labels: np.ndarray) -> np.ndarray:
    """``labels`` with clusters below ``MIN_CLUSTER`` rows set to -1 (dropped)
    and the others renumbered 1.. in order."""
    labels = np.asarray(labels, int)
    out = np.where(labels == 0, 0, -1)
    k = 0
    for c in range(1, labels.max() + 1):
        if np.sum(labels == c) >= MIN_CLUSTER:
            k += 1
            out[labels == c] = k
    return out


def _project_rotation(M: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def _mean_se3(Ms: list) -> pin.SE3:
    return pin.SE3(
        _project_rotation(sum(M.rotation for M in Ms)),
        np.mean([M.translation for M in Ms], axis=0),
    )


def register(F: list, Y: list, labels: np.ndarray, B0: pin.SE3, fix_base=False):
    """Fit ``Y_i = B * F_i * T * S_k(i)``: ``B`` (unless ``fix_base``), ``T``
    and the flip repair. ``F`` is the chain FK without frames.

    Starts from ``B0``, ``T`` from the main-cluster rows given ``B0``, and
    each ``R_k`` from its rows. Returns a dict with the predicted poses, the
    frames, and per flip cluster its rows, angle (deg) and axis.
    """
    labels = np.asarray(labels, int)
    k = int(labels.max())
    T0 = _mean_se3([(B0 * f).inverse() * y for f, y, lab in zip(F, Y, labels)
                    if lab == 0])
    w0 = []
    for c in range(1, k + 1):
        rel = [(B0 * f * T0).rotation.T @ y.rotation
               for f, y, lab in zip(F, Y, labels) if lab == c]
        w0.append(pin.log3(_project_rotation(sum(rel))))
    nb = 0 if fix_base else 6
    x0 = np.r_[np.zeros(nb + 6), *w0, np.zeros(3 if k else 0)]

    def unpack(x):
        B = B0 if fix_base else B0 * pin.exp6(x[:6])
        T = T0 * pin.exp6(x[nb:nb + 6])
        w = x[nb + 6:nb + 6 + 3 * k].reshape(k, 3)
        c = x[nb + 6 + 3 * k:] if k else np.zeros(3)
        S = [pin.SE3.Identity()] + [
            pin.SE3(pin.exp3(wi), (np.eye(3) - pin.exp3(wi)) @ c) for wi in w
        ]
        return B, T, w, c, S

    def predict(x):
        B, T, _, _, S = unpack(x)
        return [B * f * T * S[lab] for f, lab in zip(F, labels)]

    def residual(x):
        out = []
        for p, y in zip(predict(x), Y):
            out.append(p.translation - y.translation)
            out.append(LEVER * pin.log3(y.rotation.T @ p.rotation))
        return np.concatenate(out)

    sol = least_squares(residual, x0, method="lm")
    B, T, w, c, _ = unpack(sol.x)
    flips = [
        {
            "rows": np.flatnonzero(labels == i + 1).tolist(),
            "angle_deg": float(np.degrees(np.linalg.norm(wi))),
            "axis": (wi / np.linalg.norm(wi)).round(3).tolist(),
        }
        for i, wi in enumerate(w)
    ]
    return {
        "pred": predict(sol.x),
        "B": B,
        "T": T,
        "centroid_m": c.round(4).tolist() if k else None,
        "flips": flips,
        "success": bool(sol.success),
    }


# ── Held-out evaluation ───────────────────────────────────────────────────────


def torso_params(params: dict, names=None) -> dict:
    """The joint-level parameters of the torso (or the ``names`` among them)."""
    return {
        n: v
        for n, v in params.items()
        if joint_of(n) in TORSO and (names is None or n in names)
    }


def torso_strata(df: pd.DataFrame, train: pd.DataFrame) -> dict:
    return harness.posture_strata(
        df[list(TORSO)].to_numpy(),
        train[list(TORSO)].to_numpy(),
        STRATA.repeated_tol,
        STRATA.out_of_range_margin,
    )


def _by_mask(norm: np.ndarray, masks: dict) -> dict:
    return {k: (int(m.sum()), rms(norm[m])) for k, m in masks.items()}


def session_errors(calib, params: dict, path: Path, B_train: pin.SE3) -> dict:
    """Errors of a run of the other (or the same) arm with ``params``.

    Only the torso part of ``params`` reaches another arm (its own joint
    parameters are nominal). ``rereg`` refits ``B``, ``T`` and the flip
    repair; ``waist`` keeps ``B_train``; ``unflipped`` re-registers the main
    cluster alone (no repair).
    """
    df = pd.read_csv(path)
    arm = ARM[path.name]
    level = calib.calib_config["calib_model"]
    joint = {n: v for n, v in params.items() if not is_frame(n)}
    raw = flip_clusters(calib.model, df, arm)
    labels = repairable(raw)
    kept = labels >= 0
    df, labels = df[kept], labels[kept]
    F = chain_poses(calib, joint, configurations(calib.model, df, arm), arm, level)
    Y = measured_poses(df)
    re = register(F, Y, labels, B_train)
    wa = register(F, Y, labels, B_train, fix_base=True)
    main = labels == 0
    un = register([f for f, m in zip(F, main) if m], [y for y, m in zip(Y, main) if m],
                  labels[main], B_train)
    pos, ori = pose_errors(re["pred"], Y)
    pos_w, ori_w = pose_errors(wa["pred"], Y)
    pos_u, _ = pose_errors(un["pred"], [y for y, m in zip(Y, main) if m])
    strata = torso_strata(df, pd.read_csv(MOCAP / TRAINING))
    strata["flipped"] = ~main
    strata["unflipped"] = main
    shift = B_train.inverse() * re["B"]
    return {
        "n": len(df),
        "arm": arm,
        "n_flipped": int((~main).sum()),
        "dropped_rows": np.flatnonzero(~kept).tolist(),
        PRIMARY_METRIC: rms(pos),
        "ori_rmse_rereg_deg": rms(ori),
        "max_rereg_mm": float(pos.max()),
        SECONDARY_METRIC: rms(pos_w),
        "ori_rmse_waist_deg": rms(ori_w),
        "pos_rmse_unflipped_mm": rms(pos_u),
        "waist_shift": {
            "mm": float(1000 * np.linalg.norm(shift.translation)),
            "deg": float(np.degrees(np.linalg.norm(pin.log3(shift.rotation)))),
        },
        "flips": re["flips"],
        "centroid_m": re["centroid_m"],
        "strata": _by_mask(pos, strata),
        "_pos": pos,
    }


def heldout_errors(calib, path: Path, names=None) -> dict:
    """``session_errors`` with the model's torso parameters (``names`` among
    them, e.g. ``GAUGE_FREE_TORSO``)."""
    params = fitted_params(calib)
    B = from_xyzrpy([params[n] for n in BASE_TPL])
    return session_errors(calib, torso_params(params, names), path, B)


def training_errors(calib) -> dict:
    """In-sample position and orientation RMSE of a fit."""
    overall = calib.evaluation_metrics["per_dof_stats"]["overall"]
    return {
        "pos_rmse_mm": float(overall["pos_rmse_mm"]),
        "ori_rmse_deg": float(overall["orient_rmse_deg"]),
    }


def rows_errors(calib, path: Path, rows) -> dict:
    """Errors on ``rows`` of a run of the trained arm with the fit's frames."""
    df = pd.read_csv(path).iloc[list(rows)]
    arm = ARM[path.name]
    params = fitted_params(calib)
    pred = chain_poses(calib, params, configurations(calib.model, df, arm), arm)
    pos, ori = pose_errors(pred, measured_poses(df))
    return {"n": len(df), KFOLD_METRIC: rms(pos), "ori_rmse_deg": rms(ori), "_pos": pos}


def run_kfold(models=None) -> dict:
    """``{model: {KFOLD_METRIC: pooled over the held rows, per-fold errors}}``
    on the training run's folds, plus ``"_harness"`` for :func:`select_model`."""
    labels = list(MODELS) if models is None else list(models)

    def fitter(label):
        def go(fold):
            (train,) = fold.training
            return fit_model(label, train.path, train.rows)

        return go

    def evaluate(calib, held):
        return {
            **rows_errors(calib, held.path, held.rows),
            "train_rmse_mm": training_errors(calib)["pos_rmse_mm"],
        }

    res = harness.run(KFOLD_PLAN, {m: fitter(m) for m in labels}, evaluate)
    out = {}
    for m in labels:
        folds = [s for f in res[m].values() for s in f.values()]
        out[m] = {
            KFOLD_METRIC: rms(np.concatenate([f.pop("_pos") for f in folds])),
            "fold_rmse_mm": [f[KFOLD_METRIC] for f in folds],
            "fold_train_rmse_mm": [f["train_rmse_mm"] for f in folds],
        }
    out["_harness"] = res
    out["_label"] = KFOLD_PLAN.label
    return out


def select_model(kfold: dict) -> tuple:
    """Model with the lowest mean k-fold error (never the confirmation run)."""
    return harness.select(kfold["_harness"], KFOLD_METRIC, role="validation")


def run(models=None) -> dict:
    """Fit every model on the whole training run and evaluate the right-arm
    runs: ``{model: {"train": ..., "n_params", "sets": {file: errors},
    "sets_gauge_free": ...}}``."""
    labels = list(MODELS) if models is None else list(models)
    out = {}
    for label in labels:
        calib = fit_model(label)
        names = list(calib.calib_config["param_name"])
        out[label] = {
            "n_params": len(names),
            "n_joint_params": sum(not is_frame(n) for n in names),
            "torso_params": list(torso_params(fitted_params(calib))),
            "train": training_errors(calib),
            "sets": {f: heldout_errors(calib, MOCAP / f) for _, f in SETS[1:]},
            "sets_gauge_free": {
                f: heldout_errors(calib, MOCAP / f, GAUGE_FREE_TORSO)
                for _, f in SETS[1:]
            },
            "_calib": calib,
        }
    return out


def _short(name: str) -> str:
    return name.removeprefix("qualisys_2022-03-15-").removesuffix("_static_postures.csv")


def _r(v, nd=3):
    return None if v is None or np.isnan(v) else round(float(v), nd)


def snapshot(kfold: dict, results: dict, selection=None) -> dict:
    """The golden view (tests/golden/talos_mocap.json): rounded numbers of
    the k-fold plan, the selection (``select_model(kfold)`` unless given)
    and every model's held-out errors."""
    best, scores = selection or select_model(kfold)
    models = {}
    for label, res in results.items():
        sets = {}
        for role, f in SETS[1:]:
            e, g = res["sets"][f], res["sets_gauge_free"][f]
            sets[f] = {
                "role": role,
                "n": e["n"],
                "n_flipped": e["n_flipped"],
                "dropped_rows": list(e["dropped_rows"]),
                "flip_rows": [[int(r) for r in fl["rows"]] for fl in e["flips"]],
                **{k: _r(e[k]) for k in (
                    PRIMARY_METRIC, "ori_rmse_rereg_deg", SECONDARY_METRIC,
                    "ori_rmse_waist_deg", "pos_rmse_unflipped_mm")},
                "gauge_free_torso_" + PRIMARY_METRIC: _r(g[PRIMARY_METRIC]),
                "strata": {k: {"n": n, "rmse": _r(v)} for k, (n, v)
                           in e["strata"].items()},
            }
        models[label] = {
            "n_params": res["n_params"],
            "n_joint_params": res["n_joint_params"],
            "train_pos_rmse_mm": _r(res["train"]["pos_rmse_mm"]),
            "train_ori_rmse_deg": _r(res["train"]["ori_rmse_deg"]),
            "kfold_" + KFOLD_METRIC: _r(kfold[label][KFOLD_METRIC]),
            "kfold_train_mean_mm": _r(np.mean(kfold[label]["fold_train_rmse_mm"])),
            "sets": sets,
        }
    return {
        "protocol": PLAN.protocol,
        "primary_metric": PRIMARY_METRIC,
        "kfold_metric": KFOLD_METRIC,
        "selected": best,
        "selection_scores": {m: _r(v) for m, v in scores.items()},
        "models": models,
    }


def main(argv=None) -> None:
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", metavar="PATH", type=Path,
                        help="also write the golden snapshot to PATH")
    parser.add_argument("--no-plot", action="store_true",
                        help="accepted for symmetry; never plots")
    args = parser.parse_args(argv)
    kfold = run_kfold()
    best, scores = select_model(kfold)
    results = run()
    if args.json:
        args.json.write_text(
            json.dumps(snapshot(kfold, results), indent=2, sort_keys=True) + "\n")
    print(f"Protocol {PLAN.protocol}: {PLAN.label}")
    print(f"K-fold: {kfold['_label']}")
    print("Position RMSE (mm); orientation RMSE (deg)")
    for label, res in results.items():
        k = kfold[label]
        print(
            f"\n{label} ({res['n_params']} parameters, {res['n_joint_params']} "
            f"joint-level): training {res['train']['pos_rmse_mm']:.2f} mm "
            f"{res['train']['ori_rmse_deg']:.3f} deg; k-fold {k[KFOLD_METRIC]:.2f}"
        )
        for role, f in SETS[1:]:
            e, g = res["sets"][f], res["sets_gauge_free"][f]
            print(
                f"  {role:12s} {_short(f):10s} n={e['n']:2d} (flipped "
                f"{e['n_flipped']:2d})  re-registered {e[PRIMARY_METRIC]:5.2f} "
                f"({e['ori_rmse_rereg_deg']:.2f} deg)  waist kept "
                f"{e[SECONDARY_METRIC]:5.2f}  unflipped only "
                f"{e['pos_rmse_unflipped_mm']:5.2f}  gauge-free torso "
                f"{g[PRIMARY_METRIC]:5.2f}"
            )
            print("    " + "  ".join(
                f"{s} ({n}) {v:.2f}" for s, (n, v) in e["strata"].items()))
            print("    flips: " + "; ".join(
                f"{len(fl['rows'])} rows {fl['angle_deg']:.1f} deg about "
                f"{fl['axis']}" for fl in e["flips"]))
    print(f"\nSelected on the k-fold plan ({KFOLD_METRIC}): {best}")
    print("  " + "  ".join(f"{m} {s:.2f}" for m, s in scores.items()))


if __name__ == "__main__":
    main()
