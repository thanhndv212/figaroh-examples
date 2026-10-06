"""UR10 dynamic identification fixture with a known truth (#21).

The legacy UR10 CSVs (``data/identification_*_simulation.csv``) carry no
verified inertias: their generator and parameter vector were never saved
(``docs/development/ur10-signal-audit-2026-10-02.md``). This fixture is
generated from scratch, so an estimator can be judged against the truth.

- **Truth:** the repository URDF's inertias, perturbed with seed
  ``TRUTH_SEED`` in a way that stays physically consistent (the second
  moment of mass stays positive definite), saved as a URDF and as FIGAROH
  standard parameters. No friction, actuator inertia or joint offset.
- **Trajectories:** analytic q/dq/ddq sampled at ``SAMPLE_RATE_HZ``: training is core's optimal exciting trajectory (C2
  splines through rest waypoints, frozen in ``train_waypoints.json``),
  validation a seeded Fourier series. Both stay within the joint, velocity
  and effort limits and clear of the table, wall, floor and the robot
  itself (``CollisionChecker``).
- **Effort:** Pinocchio RNEA on the truth model, checked independently
  against CRBA + nonlinear effects, the joint torque regressor, FIGAROH's
  regressor, ABA, and the power balance d(T + V)/dt = tau . dq.
- **Noise:** never stored. Drawn from NumPy's legacy ``RandomState``, whose
  stream is frozen, with the seeds in ``NOISE_SEEDS``; the manifest keeps a
  hash of each draw so a changed stream is caught.

The frozen benchmark protocol (rank, base parameters, scaling, splits,
budgets, metrics) is ``data/truth/protocol.yaml``.

    python identification_truth.py              # check fixture, baseline table
    python identification_truth.py --write DIR  # regenerate into DIR
    python identification_truth.py --optimize DIR  # rerun the optimiser (slow)

Guide: ``docs/development/ur10-dynamic-truth-fixture.md``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import coal
import numpy as np
import pandas as pd
import pinocchio as pin
import yaml

HERE = Path(__file__).parent
project_root = HERE.parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.identification.identification_tools import (  # noqa: E402
    calculate_first_second_order_differentiation,
)
from figaroh.identification.parameter import get_standard_parameters  # noqa: E402
from figaroh.tools.qrdecomposition import get_baseParams  # noqa: E402
from figaroh.tools.regressor import (  # noqa: E402
    build_regressor_basic,
    build_regressor_reduced,
    get_index_eliminate,
)

FIXTURE_DIR = HERE / "data" / "truth"
URDF = HERE / "urdf" / "ur10_robot.urdf"
FIXTURE_VERSION = 1
WAYPOINTS_FILE = "train_waypoints.json"

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
SAMPLE_RATE_HZ = 100.0
# UR "home": arm up, forearm horizontal, so gravity loads every pitch joint
CENTER = [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]
AMPLITUDE_RAD = 1.2  # max |q - CENTER| per joint
VELOCITY_LIMIT = [2.0943951, 2.0943951, 3.14159265, 3.14159265, 3.14159265, 3.14159265]
VELOCITY_FRACTION = 0.7
TORQUE_LIMIT = [330.0, 330.0, 150.0, 54.0, 54.0, 54.0]  # Nm
TORQUE_FRACTION = 0.8

TRUTH_SEED = 0
# log-normal spread of the mass and of the principal second moments, COM
# shift (m) per axis, and rotation (rad) of the principal axes
TRUTH_SPREAD = {"mass": 0.10, "com_m": 0.02, "moments": 0.20, "axes_rad": 0.20}

# Training: core's exciting-trajectory optimiser (examples/ur10
# optimal_trajectory.py: IPOPT on the base-regressor condition number), run
# once by `--optimize` over `optimizer_seeds`; the best-conditioned feasible
# result is frozen as WAYPOINTS_FILE, because IPOPT results depend on the
# numerical stack (#60). Validation: a different family, a Fourier series,
# the first feasible candidate (never tuned to the estimator).
TRAJECTORIES = {
    "train": {
        "kind": "optimal",
        "optimizer_seeds": list(range(16)),
        "segments": 2,
        # joint box for the optimiser, offsets from CENTER (rad): keeps the
        # tool above the table and the wrist camera off the forearm. 4000
        # uniform samples of it are collision free at CLEARANCE_M; a
        # rest-to-rest segment stays in the box spanned by its waypoints.
        "box_low": [-np.pi, -0.8, -0.8, -1.2, -0.2, -1.5],
        "box_high": [np.pi, 0.2, 0.2, 1.2, 1.5, 1.2],
    },
    "validation": {
        "kind": "fourier",
        "fundamental_hz": 0.125,
        "harmonics": 5,
        "candidate_seeds": list(range(100, 120)),
        "select": "first",
    },
}

# Collision model: the URDF's collision geometry (arm, wrist camera and tool,
# robot base, table) plus a floor the URDF lacks. Pairs: geometries on
# different, non-adjacent joints. Clearance to the environment and between
# robot bodies, checked on the analytic trajectory at COLLISION_RATE_HZ (a
# robot point moves at most ~7 mm between checks, below both margins).
OBSTACLES = ["support_link_0", "support_link_1", "wall_link_0", "floor"]
CLEARANCE_M = {"environment": 0.05, "self": 0.01}
COLLISION_RATE_HZ = 400.0

# effort noise per joint as a fraction of that joint's noise-free training
# effort RMS (wrist_3 carries ~0.07 Nm RMS, the shoulder ~47 Nm, so one
# absolute level would be noise-free on one and swamp the other); position
# noise in rad. The resolved sigmas are frozen in protocol.yaml.
NOISE_LEVELS = {
    "none": {"effort_fraction": 0.0, "position_rad": 0.0},
    "low": {"effort_fraction": 0.01, "position_rad": 1e-5},
    "high": {"effort_fraction": 0.05, "position_rad": 1e-4},
}
NOISE_SEEDS = {
    "train": [101, 102, 103, 104, 105],
    "validation": [201, 202, 203, 204, 205],
}

# identification setting of the fixture: rigid-body inertias only
IDENTIF_CONFIG = {
    "has_friction": False,
    "has_actuator_inertia": False,
    "has_joint_offset": False,
    "is_joint_torques": True,
    "is_external_wrench": False,
    "force_torque": None,
    "act_idxv": list(range(6)),
}
ZERO_COLUMN_TOL = 1e-6
FLOAT_FORMAT = "%.17g"  # round-trips float64 exactly


class _Robot:
    """The ``robot.model`` / ``robot.data`` pair FIGAROH's regressor needs."""

    def __init__(self, model: pin.Model):
        self.model = model
        self.data = model.createData()


def columns() -> dict:
    return {s: [f"{s}{i}" for i in range(6)] for s in ("q", "dq", "ddq")} | {
        "tau": [f"tau{i + 1}" for i in range(6)]
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nominal_model() -> pin.Model:
    return pin.buildModelFromUrdf(str(URDF))


# --- truth -----------------------------------------------------------------


def draw_truth(model: pin.Model, seed: int = TRUTH_SEED) -> list:
    """Seeded, physically consistent perturbation of each link inertia.

    The rotational inertia about the COM is ``tr(S) 1 - S`` with ``S`` the
    second moment of mass. Scaling the eigenvalues of ``S`` by positive
    factors and rotating its eigenvectors keeps it positive definite, so the
    triangle inequalities hold by construction.
    """
    rng = np.random.RandomState(seed)
    truth = []
    for jid in range(1, model.njoints):
        nominal = model.inertias[jid]
        m = nominal.mass * np.exp(rng.normal(0.0, TRUTH_SPREAD["mass"]))
        c = nominal.lever + rng.normal(0.0, TRUTH_SPREAD["com_m"], 3)
        Ic = nominal.inertia
        S = 0.5 * np.trace(Ic) * np.eye(3) - Ic
        s, V = np.linalg.eigh(S)
        s = s * np.exp(rng.normal(0.0, TRUTH_SPREAD["moments"], 3))
        R = pin.exp3(rng.normal(0.0, TRUTH_SPREAD["axes_rad"], 3)) @ V
        S = (R * s) @ R.T
        S = 0.5 * (S + S.T)
        truth.append(pin.Inertia(m, c, np.trace(S) * np.eye(3) - S))
    return truth


def with_inertias(model: pin.Model, inertias: list) -> pin.Model:
    out = pin.Model(model)
    for jid, inertia in enumerate(inertias, start=1):
        out.inertias[jid] = inertia
    return out


def pseudo_inertia_min_eig(inertia: pin.Inertia) -> float:
    """Smallest eigenvalue of the 4x4 pseudo-inertia (> 0: consistent)."""
    m, c, Ic = inertia.mass, inertia.lever, inertia.inertia
    S = 0.5 * np.trace(Ic) * np.eye(3) - Ic + m * np.outer(c, c)
    J = np.block([[S, m * c[:, None]], [m * c[None, :], np.array([[m]])]])
    return float(np.linalg.eigvalsh(J).min())


def write_truth_urdf(inertias: list, path: Path, model: pin.Model) -> None:
    """Copy of the repository URDF with each moving link's inertial replaced.

    Pinocchio's joint frame is the child link frame, so the COM and the
    inertia about it are written in that frame with a zero rotation.
    Pinocchio merges links attached by fixed joints into the moving body
    (the wrist_3 body includes the tool mount and camera), so a truth is the
    whole body: it goes on the moving link and the attached links lose their
    inertials.
    """
    tree = ET.parse(URDF)
    root = tree.getroot()
    joints = root.findall("joint")
    child = {j.get("name"): j.find("child").get("link") for j in joints}
    links = {link.get("name"): link for link in root.findall("link")}
    moving = {child[model.names[jid]] for jid in range(1, model.njoints)}
    attached, grew = set(), True
    while grew:
        grew = False
        for j in joints:
            parent, link = j.find("parent").get("link"), j.find("child").get("link")
            if (
                j.get("type") == "fixed"
                and (parent in moving or parent in attached)
                and link not in attached
            ):
                attached.add(link)
                grew = True
    for name in attached:
        inertial = links[name].find("inertial")
        if inertial is not None:
            links[name].remove(inertial)
    for jid, inertia in enumerate(inertias, start=1):
        link = links[child[model.names[jid]]]
        inertial = link.find("inertial")
        inertial.find("mass").set("value", repr(float(inertia.mass)))
        origin = inertial.find("origin")
        origin.set("xyz", " ".join(repr(float(v)) for v in inertia.lever))
        origin.set("rpy", "0 0 0")
        Ic = inertia.inertia
        for key, (a, b) in {
            "ixx": (0, 0),
            "ixy": (0, 1),
            "ixz": (0, 2),
            "iyy": (1, 1),
            "iyz": (1, 2),
            "izz": (2, 2),
        }.items():
            inertial.find("inertia").set(key, repr(float(Ic[a, b])))
    comment = ET.Comment(
        f" UR10 dynamic truth fixture v{FIXTURE_VERSION} (figaroh-examples #21): "
        f"inertias of ur10_robot.urdf perturbed with seed {TRUTH_SEED}. "
        "Generated by examples/ur10/identification_truth.py; do not edit. "
    )
    root.insert(0, comment)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    Path(path).write_text("\n".join(x.rstrip() for x in lines) + "\n", encoding="utf-8")


# --- trajectories ----------------------------------------------------------
#
# A trajectory is stored as its definition, evaluated analytically:
# {"kind": "fourier", fundamental_hz, harmonics, center, coefficients} or
# {"kind": "spline", interval_s, waypoints} (core's C2 cubic splines through
# rest waypoints, as built by figaroh.utils.cubic_spline.CubicSpline).


def fourier(spec: dict, seed: int) -> dict:
    """One period of a finite Fourier series (definition).

    q_j = c_j + s_j sum_k (a_jk sin(k w t) - b_jk cos(k w t)) / (k w), with
    one gain s_j per joint chosen so |q - c| <= AMPLITUDE_RAD and
    |dq| <= VELOCITY_FRACTION * VELOCITY_LIMIT over the samples.
    """
    rng = np.random.RandomState(seed)
    K = spec["harmonics"]
    a = rng.uniform(-1.0, 1.0, (6, K))
    b = rng.uniform(-1.0, 1.0, (6, K))
    defn = {
        "kind": "fourier",
        "fundamental_hz": spec["fundamental_hz"],
        "harmonics": K,
        "center": list(CENTER),
        "coefficients": {"a": a.tolist(), "b": b.tolist()},
    }
    q, dq, _ = trajectory_at(defn, sample_times(defn))
    gain = np.minimum(
        AMPLITUDE_RAD / np.abs(q - np.array(CENTER)).max(0),
        VELOCITY_FRACTION * np.array(VELOCITY_LIMIT) / np.abs(dq).max(0),
    )
    defn["coefficients"] = {
        "a": (a * gain[:, None]).tolist(),
        "b": (b * gain[:, None]).tolist(),
    }
    return defn


def _spline(defn: dict):
    import ndcurves

    wps = np.array(defn["waypoints"])
    curve = ndcurves.piecewise()
    zero = np.zeros((wps.shape[1], 1))
    for i in range(len(wps) - 1):
        rest = ndcurves.curve_constraints()
        rest.init_vel, rest.end_vel, rest.init_acc, rest.end_acc = (
            zero,
            zero,
            zero,
            zero,
        )
        times = np.array([i, i + 1]) * defn["interval_s"]
        curve.append(ndcurves.exact_cubic(wps[i : i + 2].T, times, rest))
    return curve


def duration(defn: dict) -> float:
    if defn["kind"] == "fourier":
        return 1.0 / defn["fundamental_hz"]
    return (len(defn["waypoints"]) - 1) * defn["interval_s"]


def sample_times(defn: dict, rate_hz: float = SAMPLE_RATE_HZ) -> np.ndarray:
    return np.arange(int(round(duration(defn) * rate_hz))) / rate_hz


def trajectory_at(defn: dict, t: np.ndarray) -> tuple:
    """Analytic q, dq, ddq at times ``t``."""
    t = np.asarray(t, dtype=float)
    if defn["kind"] == "fourier":
        a = np.array(defn["coefficients"]["a"])
        b = np.array(defn["coefficients"]["b"])
        kw = np.arange(1, defn["harmonics"] + 1) * 2 * np.pi * defn["fundamental_hz"]
        S, C = np.sin(np.outer(t, kw)), np.cos(np.outer(t, kw))
        q = np.array(defn["center"]) + S @ (a / kw).T - C @ (b / kw).T
        return q, C @ a.T + S @ b.T, -S @ (a * kw).T + C @ (b * kw).T
    curve = _spline(defn)
    t = np.clip(t, curve.min(), curve.max())
    return tuple(
        np.array([curve.derivate(x, order) if order else curve(x) for x in t])
        for order in (0, 1, 2)
    )


def knots(defn: dict) -> np.ndarray:
    """Times where the third derivative may jump (spline waypoints)."""
    if defn["kind"] == "fourier":
        return np.array([])
    return np.arange(len(defn["waypoints"])) * defn["interval_s"]


def rnea(model: pin.Model, q, dq, ddq) -> np.ndarray:
    data = model.createData()
    return np.array([pin.rnea(model, data, *x).copy() for x in zip(q, dq, ddq)])


def base_reference(model: pin.Model, q, dq, ddq) -> dict:
    """FIGAROH's base parameters of the fixture's rigid-body model."""
    std = get_standard_parameters(model, IDENTIF_CONFIG)
    W = build_regressor_basic(_Robot(model), q, dq, ddq, IDENTIF_CONFIG)
    idx_e, params_r = get_index_eliminate(W, std, ZERO_COLUMN_TOL)
    W_e = build_regressor_reduced(W, idx_e)
    W_b, params_base, idx_base = get_baseParams(W_e, params_r, std)
    # the protocol rebuilds W_b as the selected columns of W_e
    tol = 1e-12 * np.abs(W_e).max()
    if not np.allclose(W_b, W_e[:, list(idx_base)], rtol=0, atol=tol):
        raise RuntimeError("base regressor is not a column subset of the reduced one")
    return {
        "W": W,
        "W_b": W_b,
        "eliminated": list(idx_e),
        "base_indices": [int(i) for i in idx_base],
        "base_names": list(params_base),
        "standard_names": list(std),
    }


def add_collision_model(geom: pin.GeometryModel) -> list:
    """Add the floor and the checked pairs to ``geom``; returns (pair, kind).

    Pairs: geometries on different, non-adjacent joints (a fixed body counts
    as joint 0). Kind: "environment" when one side is an obstacle.
    """
    floor = pin.GeometryObject(
        "floor",
        0,
        pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.05])),
        coal.Box(10, 10, 0.1),
    )
    geom.addGeometryObject(floor)
    names = [g.name for g in geom.geometryObjects]
    parent = [g.parentJoint for g in geom.geometryObjects]
    pairs = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if abs(parent[i] - parent[j]) <= 1:
                continue  # same or adjacent joint (both fixed: same)
            geom.addCollisionPair(pin.CollisionPair(i, j))
            env = names[i] in OBSTACLES or names[j] in OBSTACLES
            pairs.append(((names[i], names[j]), "environment" if env else "self"))
    return pairs


class CollisionChecker:
    """Clearance of the UR10 (arm, base, wrist camera and tool) to itself and
    to the table, wall and floor, on the URDF's collision geometry."""

    def __init__(self):
        self.model, self.geom, _ = pin.buildModelsFromUrdf(
            str(URDF), package_dirs=[str(HERE.parents[1] / "models")]
        )
        pairs = add_collision_model(self.geom)
        self.pairs = [p for p, _ in pairs]
        self.kind = [k for _, k in pairs]
        self.data = self.model.createData()
        self.gdata = pin.GeometryData(self.geom)
        for request, kind in zip(self.gdata.collisionRequests, self.kind):
            request.security_margin = CLEARANCE_M[kind]

    def configurations(self, defn: dict) -> np.ndarray:
        return trajectory_at(defn, sample_times(defn, COLLISION_RATE_HZ))[0]

    def clear(self, defn: dict) -> bool:
        """True when no pair comes closer than its clearance."""
        return not any(
            pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, True)
            for q in self.configurations(defn)
        )

    def report(self, defn: dict, screen: float = 3.0) -> dict:
        """Smallest distance per pair class, and the closest pair.

        Exact distances are computed only for pairs a collision query flags
        within ``screen`` times their clearance; a class with no flagged pair
        reports ``min_distance_m: null`` (farther than that everywhere).
        """
        requests = self.gdata.collisionRequests
        for request, kind in zip(requests, self.kind):
            request.security_margin = screen * CLEARANCE_M[kind]
        dmin = np.full(len(self.pairs), np.inf)
        try:
            for q in self.configurations(defn):
                pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q)
                for k, result in enumerate(self.gdata.collisionResults):
                    if result.isCollision():
                        d = pin.computeDistance(self.geom, self.gdata, k).min_distance
                        dmin[k] = min(dmin[k], d)
        finally:
            for request, kind in zip(requests, self.kind):
                request.security_margin = CLEARANCE_M[kind]
        out = {"rate_hz": COLLISION_RATE_HZ, "pairs": len(self.pairs)}
        for kind in CLEARANCE_M:
            k = [i for i, x in enumerate(self.kind) if x == kind]
            i = k[int(np.argmin(dmin[k]))]
            close = np.isfinite(dmin[i])
            out[kind] = {
                "required_m": CLEARANCE_M[kind],
                "min_distance_m": float(dmin[i]) if close else None,
                "closest_pair": list(self.pairs[i]) if close else None,
                "screened_beyond_m": screen * CLEARANCE_M[kind],
            }
        return out


def infeasible(defn: dict, truth: pin.Model, checker: CollisionChecker) -> str:
    """Why a trajectory is not usable ("" when it is): joint, velocity or
    truth-effort limits, or collision clearance."""
    model = checker.model
    q, dq, ddq = trajectory_at(defn, sample_times(defn))
    if np.any(q < model.lowerPositionLimit) or np.any(q > model.upperPositionLimit):
        return "position"
    if np.any(np.abs(dq) > np.array(VELOCITY_LIMIT)):
        return "velocity"
    tau = rnea(truth, q, dq, ddq)
    if np.any(np.abs(tau) > TORQUE_FRACTION * np.array(TORQUE_LIMIT)):
        return "torque"
    if not checker.clear(defn):
        return "collision"
    return ""


def condition_number(model: pin.Model, defn: dict) -> float:
    q, dq, ddq = trajectory_at(defn, sample_times(defn))
    return float(np.linalg.cond(base_reference(model, q, dq, ddq)["W_b"]))


def make_trajectory(
    split: str, model: pin.Model, truth: pin.Model, checker: CollisionChecker, waypoints
) -> dict:
    """The split's trajectory definition with its selection record.

    ``optimal``: the committed core-optimised waypoints (``waypoints``), which
    must be feasible. ``fourier``: a candidate seed selected by
    ``TRAJECTORIES[split]["select"]`` among the feasible ones.
    """
    spec = TRAJECTORIES[split]
    if spec["kind"] == "optimal":
        defn = {
            "kind": "spline",
            "interval_s": waypoints["interval_s"],
            "waypoints": waypoints["waypoints"],
        }
        reason = infeasible(defn, truth, checker)
        if reason:
            raise RuntimeError(f"{split}: optimised trajectory fails on {reason}")
        out = {"definition": defn, "selection": waypoints["selection"]}
    else:
        best, rejected = None, {}
        for seed in spec["candidate_seeds"]:
            defn = fourier(spec, seed)
            reason = infeasible(defn, truth, checker)
            if reason:
                rejected.setdefault(reason, []).append(seed)
                continue
            cond = condition_number(model, defn)
            if best is None or cond < best[2]:
                best = (seed, defn, cond)
            if spec["select"] == "first":
                break
        if best is None:
            raise RuntimeError(f"{split}: no feasible candidate seed")
        out = {
            "definition": best[1],
            "selection": {
                "kind": "fourier",
                "candidate_seeds": spec["candidate_seeds"],
                "select": spec["select"],
                "seed": best[0],
                "rejected_seeds": rejected,
            },
        }
    out["collision"] = checker.report(out["definition"])
    return out


def _optimize_once(seed: int) -> dict:
    """One run of core's trajectory optimiser (as ``optimal_trajectory.py``).

    The joint limits are narrowed to the training box. Core's own collision
    constraint is inactive: ``robot.geom_model`` has no collision pairs for
    the UR10 (they come only from an SRDF), and given pairs it checks
    waypoints only, which let splines sweep through the table. The box keeps
    the motion clear; :func:`infeasible` checks it at COLLISION_RATE_HZ.
    Returns the waypoints of the stacked segments, or ``None`` if a segment
    failed.
    """
    import contextlib
    import io

    from figaroh.tools.robot import load_robot

    from examples.ur10.utils.ur10_tools import OptimalTrajectoryIPOPT

    class Recording(OptimalTrajectoryIPOPT):
        # keep each problem's last objective point: core evaluates the
        # objective at the returned solution last
        def create_ipopt_problem(self, *args):
            problem = super().create_ipopt_problem(*args)
            objective = problem.objective

            def recorded(X, _objective=objective, _problem=problem):
                _problem.last_x = np.array(X, dtype=float)
                return _objective(X)

            problem.objective = recorded
            self.problems.append(problem)
            return problem

    robot = load_robot(
        str(URDF), package_dirs=str(HERE.parents[1] / "models"), load_by_urdf=True
    )
    spec = TRAJECTORIES["train"]
    idx = [robot.model.joints[robot.model.getJointId(j)].idx_q for j in JOINTS]
    robot.model.lowerPositionLimit[idx] = np.array(CENTER) + spec["box_low"]
    robot.model.upperPositionLimit[idx] = np.array(CENTER) + spec["box_high"]
    config = str(HERE / "config" / "ur10_unified_config.yaml")
    opt = Recording(robot=robot, active_joints=JOINTS, config_file=config)
    opt.problems = []
    ps = opt.identif_config
    ps["active_joints"] = JOINTS
    ps["act_Jid"] = [opt.model.getJointId(j) for j in JOINTS]
    ps["act_J"] = [opt.model.joints[j] for j in ps["act_Jid"]]
    ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
    ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]
    np.random.seed(seed)
    with contextlib.redirect_stdout(io.StringIO()):
        opt.initialize()
        results = opt.solve(stack_reps=TRAJECTORIES["train"]["segments"])
    if len(results["T_F"]) < TRAJECTORIES["train"]["segments"]:
        return None
    waypoints, status = [], []
    for record in results["iteration_data"]:
        final = record["final_waypoint"]
        problem = next(
            p
            for p in reversed(opt.problems)
            if hasattr(p, "last_x") and np.array_equal(p.last_x[-len(JOINTS) :], final)
        )
        if np.any(problem.vel_wps) or np.any(problem.acc_wps):
            raise RuntimeError("expected rest waypoints (zero velocity/acceleration)")
        segment = np.vstack([problem.wp_init, problem.last_x.reshape(-1, len(JOINTS))])
        waypoints.extend(segment if not waypoints else segment[1:])
        status.append({k: record[k] for k in ("status", "converged", "attempt")})
    defn = {
        "kind": "spline",
        "interval_s": float(opt.trajectory_config["t_s"]),
        "waypoints": np.array(waypoints).tolist(),
    }
    # the rebuilt spline is the one core optimised (its first segment)
    t0 = np.asarray(results["T_F"][0]).ravel()
    q0 = trajectory_at(defn, t0 - t0[0])[0]
    if not np.allclose(q0, np.asarray(results["P_F"][0]), rtol=0, atol=1e-9):
        raise RuntimeError("rebuilt spline differs from the optimised trajectory")
    return {"definition": defn, "segments": status}


def optimize_training(seeds=None) -> dict:
    """Run core's optimiser per seed; keep the best-conditioned feasible run.

    Returns the content of ``WAYPOINTS_FILE``.
    """
    seeds = TRAJECTORIES["train"]["optimizer_seeds"] if seeds is None else seeds
    model = nominal_model()
    truth = with_inertias(model, draw_truth(model))
    checker = CollisionChecker()
    runs, best = [], None
    for seed in seeds:
        run = _optimize_once(seed)
        if run is None:
            runs.append({"seed": seed, "result": "segment failed"})
            print(f"seed {seed}: segment failed")
            continue
        reason = infeasible(run["definition"], truth, checker)
        cond = condition_number(model, run["definition"])
        runs.append(
            {
                "seed": seed,
                "result": reason or "feasible",
                "condition_number": cond,
                "segments": run["segments"],
            }
        )
        print(f"seed {seed}: {reason or 'feasible'}, condition number {cond:.1f}")
        if not reason and (best is None or cond < best[2]):
            best = (seed, run, cond)
    if best is None:
        raise RuntimeError("no optimised trajectory is feasible; try more seeds")
    seed, run, cond = best
    return {
        "interval_s": run["definition"]["interval_s"],
        "waypoints": run["definition"]["waypoints"],
        "selection": {
            "kind": "core optimal trajectory (IPOPT, base-regressor condition number)",
            "config": "examples/ur10/config/ur10_unified_config.yaml",
            "segments": TRAJECTORIES["train"]["segments"],
            "joint_box_rad": {
                "low": (np.array(CENTER) + TRAJECTORIES["train"]["box_low"]).tolist(),
                "high": (np.array(CENTER) + TRAJECTORIES["train"]["box_high"]).tolist(),
            },
            "optimizer_seeds": list(seeds),
            "seed": seed,
            "condition_number": cond,
            "runs": runs,
            "environment": {
                "pinocchio": pin.__version__,
                "numpy": np.__version__,
                "python": sys.version.split()[0],
            },
        },
    }


# --- independent checks ----------------------------------------------------


def independent_checks(truth: pin.Model, split: dict, defn: dict) -> dict:
    """Largest discrepancies of the saved effort and derivatives (absolute)."""
    q, dq, ddq, tau = split["q"], split["dq"], split["ddq"], split["tau"]
    data = truth.createData()
    phi = np.concatenate([truth.inertias[j].toDynamicParameters() for j in range(1, 7)])
    out = {"crba_nle": 0.0, "pinocchio_regressor": 0.0, "aba_ddq": 0.0}
    for k in range(len(q)):
        M = pin.crba(truth, data, q[k])
        M = np.triu(M) + np.triu(M, 1).T
        nle = pin.nonLinearEffects(truth, data, q[k], dq[k])
        out["crba_nle"] = max(out["crba_nle"], np.abs(M @ ddq[k] + nle - tau[k]).max())
        Y = pin.computeJointTorqueRegressor(truth, data, q[k], dq[k], ddq[k])
        out["pinocchio_regressor"] = max(
            out["pinocchio_regressor"], np.abs(Y @ phi - tau[k]).max()
        )
        a = pin.aba(truth, data, q[k], dq[k], tau[k])
        out["aba_ddq"] = max(out["aba_ddq"], np.abs(a - ddq[k]).max())
    W = build_regressor_basic(_Robot(truth), q, dq, ddq, IDENTIF_CONFIG)
    std = np.array(list(get_standard_parameters(truth, IDENTIF_CONFIG).values()))
    out["figaroh_regressor"] = float(np.abs(W @ std - tau.T.ravel()).max())

    # power balance: d(T + V)/dt = tau . dq, energy differentiated at +-h
    h = 1e-5
    power = np.einsum("ij,ij->i", tau, dq)
    t = split["t"]
    energy, plus_minus = [], []
    for dt in (-h, h):
        qh, dqh, _ = trajectory_at(defn, t + dt)
        plus_minus.append((qh, dqh))
        energy.append(
            np.array(
                [
                    pin.computeKineticEnergy(truth, data, x, v)
                    + pin.computePotentialEnergy(truth, data, x)
                    for x, v in zip(qh, dqh)
                ]
            )
        )
    dE = (energy[1] - energy[0]) / (2 * h)
    # the clipped spline ends hold still; skip the first and last rows there
    inner = (t - h >= 0) & (t + h <= duration(defn))
    out["power_balance_w"] = float(np.abs(dE - power)[inner].max())
    out["power_scale_w"] = float(np.abs(power).max())

    # analytic derivatives against central differences, every coordinate,
    # away from spline waypoints (where the third derivative jumps)
    (qm, dqm), (qp, dqp) = plus_minus
    smooth = inner & np.all(np.abs(t[:, None] - knots(defn)[None, :]) > 2 * h, axis=1)
    out["dq_central_difference"] = (
        np.abs((qp - qm) / (2 * h) - dq)[smooth].max(0).tolist()
    )
    out["ddq_central_difference"] = (
        np.abs((dqp - dqm) / (2 * h) - ddq)[smooth].max(0).tolist()
    )
    return {k: (float(v) if np.isscalar(v) else v) for k, v in out.items()}


# --- noise -----------------------------------------------------------------


def standard_draws(seed: int, n: int) -> tuple:
    """(effort, position) standard normal draws; legacy stream is frozen."""
    rng = np.random.RandomState(seed)
    return rng.standard_normal((n, 6)), rng.standard_normal((n, 6))


def draws_sha256(seed: int, n: int) -> str:
    e, p = standard_draws(seed, n)
    return hashlib.sha256(np.ascontiguousarray(np.vstack([e, p])).tobytes()).hexdigest()


# --- loading ---------------------------------------------------------------


def manifest() -> dict:
    return json.loads((FIXTURE_DIR / "manifest.json").read_text())


def protocol() -> dict:
    return yaml.safe_load((FIXTURE_DIR / "protocol.yaml").read_text())


def truth_model() -> pin.Model:
    return pin.buildModelFromUrdf(str(FIXTURE_DIR / "ur10_truth.urdf"))


def truth_parameters(directory: Path = FIXTURE_DIR) -> pd.DataFrame:
    """FIGAROH standard parameters: ``parameter``, ``truth``, ``nominal``."""
    return pd.read_csv(directory / "truth_parameters.csv", float_precision="round_trip")


def read_split(split: str, directory: Path = FIXTURE_DIR) -> dict:
    df = pd.read_csv(directory / f"{split}.csv", float_precision="round_trip")
    out = {"t": df["t"].to_numpy()}
    for key, names in columns().items():
        out[key] = df[names].to_numpy()
    return out


def load_split(
    split: str,
    derivatives: str = "analytic",
    noise: str = "none",
    noise_seed: int | None = None,
) -> dict:
    """One split as the estimator sees it.

    ``derivatives="analytic"``: saved q/dq/ddq, all rows. ``"differentiated"``:
    q only, differentiated by core's helper with the fixture clock; it keeps
    the first n-2 rows, velocities half a step late (historical alignment).
    Noise is added to effort and position (``NOISE_LEVELS``) when ``noise``
    is not ``"none"``; ``noise_seed`` must then be one of the split's
    ``NOISE_SEEDS``. ``tau_true`` is the noise-free effort on the same rows.
    """
    data = read_split(split)
    n = len(data["t"])
    level = NOISE_LEVELS[noise]
    q, tau = data["q"], data["tau"]
    if noise != "none":
        if noise_seed not in NOISE_SEEDS[split]:
            raise ValueError(f"{split}: noise seed must be one of {NOISE_SEEDS[split]}")
        e, p = standard_draws(noise_seed, n)
        sigma = protocol()["noise"]["effort_sigma_nm"][noise]
        tau = tau + e * np.array([sigma[j] for j in JOINTS])
        q = q + p * level["position_rad"]
    if derivatives == "analytic":
        # analytic derivatives are exact; position noise only enters via q
        out = {"q": q, "dq": data["dq"], "ddq": data["ddq"], "tau": tau}
        rows = slice(None)
    elif derivatives == "differentiated":
        cfg = {"ts": 1.0 / SAMPLE_RATE_HZ}
        qd, dq, ddq = calculate_first_second_order_differentiation(
            nominal_model(), q, cfg
        )
        rows = slice(0, len(qd))
        out = {"q": qd, "dq": dq, "ddq": ddq, "tau": tau[rows]}
    else:
        raise ValueError(derivatives)
    out["t"] = data["t"][rows]
    out["tau_true"] = data["tau"][rows]
    return out


# --- generation ------------------------------------------------------------


def generate(out_dir: Path, waypoints_file: Path | None = None) -> dict:
    """Write the fixture to ``out_dir``; returns the manifest.

    The training trajectory comes from ``waypoints_file`` (default: the
    committed ``WAYPOINTS_FILE``), written by :func:`optimize_training`;
    it is copied into ``out_dir``.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    waypoints_file = Path(waypoints_file or FIXTURE_DIR / WAYPOINTS_FILE)
    waypoints = json.loads(waypoints_file.read_text())
    if waypoints_file.resolve() != (out_dir / WAYPOINTS_FILE).resolve():
        (out_dir / WAYPOINTS_FILE).write_text(waypoints_file.read_text())
    model = nominal_model()
    inertias = draw_truth(model)
    truth = with_inertias(model, inertias)
    urdf_path = out_dir / "ur10_truth.urdf"
    write_truth_urdf(inertias, urdf_path, model)

    reloaded = pin.buildModelFromUrdf(str(urdf_path))
    std_truth = get_standard_parameters(reloaded, IDENTIF_CONFIG)
    std_nominal = get_standard_parameters(model, IDENTIF_CONFIG)
    phi_generated = np.array(
        list(get_standard_parameters(truth, IDENTIF_CONFIG).values())
    )
    pd.DataFrame(
        {
            "parameter": list(std_truth),
            "truth": list(std_truth.values()),
            "nominal": list(std_nominal.values()),
        }
    ).to_csv(out_dir / "truth_parameters.csv", index=False, float_format=FLOAT_FORMAT)

    splits, refs, checks = {}, {}, {}
    checker = CollisionChecker()
    for split in TRAJECTORIES:
        traj = make_trajectory(split, model, truth, checker, waypoints)
        t = sample_times(traj["definition"])
        q, dq, ddq = trajectory_at(traj["definition"], t)
        df = pd.DataFrame({"t": t})
        for key, value in zip(columns(), (q, dq, ddq, rnea(truth, q, dq, ddq))):
            df[columns()[key]] = value
        path = out_dir / f"{split}.csv"
        df.to_csv(path, index=False, float_format=FLOAT_FORMAT)
        saved = read_split(split, out_dir)
        splits[split] = dict(traj, **saved)
        refs[split] = base_reference(model, saved["q"], saved["dq"], saved["ddq"])
        checks[split] = independent_checks(reloaded, saved, traj["definition"])

    ref = refs["train"]
    phi_std = np.array(list(std_truth.values()))
    tau_train = splits["train"]["tau"]
    phi_base, *_ = np.linalg.lstsq(ref["W_b"], tau_train.T.ravel(), rcond=None)
    effort_scale = np.sqrt(np.mean(tau_train**2, axis=0))

    entry = {
        "fixture_version": FIXTURE_VERSION,
        "issue": "figaroh-examples#21",
        "generator": "examples/ur10/identification_truth.py",
        "environment": {
            "pinocchio": pin.__version__,
            "numpy": np.__version__,
            "python": sys.version.split()[0],
        },
        "model": {
            "nominal_urdf": "examples/ur10/urdf/ur10_robot.urdf",
            "nominal_urdf_sha256": sha256(URDF),
            "truth_urdf_sha256": sha256(urdf_path),
            "joints": JOINTS,
            "extras": "none: no friction, actuator inertia or joint offset in the truth",
            "truth_seed": TRUTH_SEED,
            "truth_spread": TRUTH_SPREAD,
            "urdf_reload_max_abs_error": float(np.abs(phi_std - phi_generated).max()),
            "pseudo_inertia_min_eigenvalue": {
                JOINTS[j - 1]: pseudo_inertia_min_eig(reloaded.inertias[j])
                for j in range(1, 7)
            },
        },
        "sampling": {"rate_hz": SAMPLE_RATE_HZ, "clock": "exact (simulation)"},
        "limits": {
            "position": "URDF joint limits",
            "velocity_limit": VELOCITY_LIMIT,
            "fourier_design": {
                "center_rad": CENTER,
                "amplitude_rad": AMPLITUDE_RAD,
                "velocity_fraction": VELOCITY_FRACTION,
            },
            "torque_limit_nm": TORQUE_LIMIT,
            "torque_fraction": TORQUE_FRACTION,
            "collision": {
                "geometry": "URDF collision geometry plus a floor box (top at z = 0)",
                "obstacles": OBSTACLES,
                "pairs": "geometries on different, non-adjacent joints",
                "clearance_m": CLEARANCE_M,
                "rate_hz": COLLISION_RATE_HZ,
            },
        },
        "splits": {},
        "noise": {
            "levels": NOISE_LEVELS,
            "seeds": NOISE_SEEDS,
            "generator": "numpy.random.RandomState(seed): effort draws (n, 6), then position draws (n, 6)",
            "draws_sha256": {},
        },
        "files": {},
    }
    for split, traj in splits.items():
        n = len(traj["t"])
        entry["splits"][split] = {
            "rows": n,
            "duration_s": n / SAMPLE_RATE_HZ,
            "selection": traj["selection"],
            "definition": traj["definition"],
            "collision": traj["collision"],
            "base_condition_number": float(np.linalg.cond(refs[split]["W_b"])),
            "rank": len(refs[split]["base_names"]),
            "q_range_rad": np.ptp(traj["q"], axis=0).tolist(),
            "dq_max_abs": np.abs(traj["dq"]).max(0).tolist(),
            "ddq_max_abs": np.abs(traj["ddq"]).max(0).tolist(),
            "tau_max_abs_nm": np.abs(traj["tau"]).max(0).tolist(),
            "tau_rms_nm": np.sqrt(np.mean(traj["tau"] ** 2, axis=0)).tolist(),
            "checks": checks[split],
        }
        entry["noise"]["draws_sha256"][split] = {
            str(s): draws_sha256(s, n) for s in NOISE_SEEDS[split]
        }
    for name in (
        WAYPOINTS_FILE,
        "ur10_truth.urdf",
        "truth_parameters.csv",
        "train.csv",
        "validation.csv",
    ):
        entry["files"][name] = sha256(out_dir / name)
    (out_dir / "manifest.json").write_text(json.dumps(entry, indent=2) + "\n")

    W_b = ref["W_b"]
    column_norm = np.linalg.norm(W_b, axis=0) / np.sqrt(len(tau_train))
    protocol_entry = {
        "protocol_version": 1,
        "fixture_version": FIXTURE_VERSION,
        "status": "frozen; revise by adding protocol_version 2, never by editing v1",
        "splits": {
            "fit": "train.csv, all rows; nothing else may be fitted or tuned on",
            "validation": "validation.csv, all rows; never used to fit, select or tune",
            "derivatives": {
                "analytic": "saved q/dq/ddq (isolates the estimator)",
                "differentiated": "q only, core calculate_first_second_order_differentiation at 1/rate_hz, first n-2 rows, no filter",
            },
            "decimation": "none",
            "edge_exclusion": "none",
        },
        "noise": {
            "levels": NOISE_LEVELS,
            "effort_sigma_nm": {
                level: dict(zip(JOINTS, (v["effort_fraction"] * effort_scale).tolist()))
                for level, v in NOISE_LEVELS.items()
            },
            "seeds": NOISE_SEEDS,
            "validation_noise": "same sigmas as training, paired seed",
            "report": "every seed of a level; mean and worst case",
        },
        "rank": {
            "standard_parameters": len(ref["standard_names"]),
            "zero_column_tolerance": ZERO_COLUMN_TOL,
            "eliminated_indices": ref["eliminated"],
            "base_parameters": len(ref["base_names"]),
            "base_indices": ref["base_indices"],
            "base_names": ref["base_names"],
            "validation_rank": len(refs["validation"]["base_names"]),
            "base_truth": dict(zip(ref["base_names"], phi_base.tolist())),
        },
        "scaling": {
            "effort_nm_per_joint": dict(zip(JOINTS, effort_scale.tolist())),
            "effort_rule": "per-joint RMS of the noise-free training effort; NRMSE divides by it",
            "base_column_rms": dict(zip(ref["base_names"], column_norm.tolist())),
            "base_rule": "base parameter error times its training column RMS (Nm): effort it accounts for",
            "condition_number": "of the unscaled training base regressor",
            "train_condition_number": entry["splits"]["train"]["base_condition_number"],
        },
        "budgets": {
            "max_iterations": 1000,
            "max_wall_time_s": 600,
            "nonlinear_starts": 5,
            "start_seeds": [0, 1, 2, 3, 4],
            "rule": "a fit that hits a budget is reported as not converged, with its iterate, never retried with a larger budget under v1",
        },
        "metrics": {
            "train": ["effort RMSE per joint (Nm) against the fitted effort"],
            "validation": [
                "effort RMSE per joint (Nm) against the noise-free truth effort",
                "effort NRMSE per joint (%) = RMSE / scaling.effort_nm_per_joint",
                "effort RMSE per joint (Nm) against the noisy validation effort (same level, paired seed)",
            ],
            "parameters": [
                "base parameter error, scaled by base_column_rms (Nm), max and RMS",
                "standard parameters (physical estimators): error per link against truth_parameters.csv, diagnostic only (not identifiable)",
                "pseudo-inertia minimum eigenvalue per link (feasibility, checked independently of the solver)",
            ],
            "status": "convergence and feasibility reported separately; a solver error is not proof of infeasibility",
        },
        "rules": [
            "No parameter truth is inferred from the legacy data/identification_*_simulation.csv files.",
            "Extras (friction, actuator inertia, offsets) are zero in the truth; a frozen-extra comparison fixes them at zero, a joint-extra one reports them separately.",
            "Report tested core/examples revisions, Pinocchio version and the fixture file hashes in manifest.json.",
        ],
    }
    (out_dir / "protocol.yaml").write_text(
        "# Frozen benchmark protocol of the UR10 dynamic truth fixture (#21).\n"
        "# Generated by examples/ur10/identification_truth.py; do not edit.\n"
        + yaml.safe_dump(protocol_entry, sort_keys=False, width=100)
    )
    return entry


# --- core pipeline ---------------------------------------------------------


def identification(
    derivatives: str = "analytic",
    noise: str = "none",
    seed: int | None = None,
    val_seed: int | None = None,
):
    """An initialized ``UR10Identification`` that reads the fixture.

    Training is ``train.csv``, held-out validation is ``validation.csv``
    (``load_split`` with the paired noise seed). The model is the nominal
    URDF, so ``standard_parameter`` is the nominal prior, not the truth.
    The fixture's derivatives are used as given (no filter): call
    ``solve(decimate=False)``.
    """
    from figaroh.tools.robot import load_robot

    from examples.ur10.utils.ur10_tools import UR10Identification

    class FixtureIdentification(UR10Identification):
        def load_trajectory_data(self, data_source=None):
            split = data_source or "train"
            d = load_split(split, derivatives, noise, val_seed if data_source else seed)
            return {
                "timestamps": d["t"].reshape(-1, 1),
                "positions": d["q"],
                "velocities": d["dq"],
                "accelerations": d["ddq"],
                "torques": d["tau"],
            }

        def process_kinematics_data(self, filter_config=None):
            self.processed_data = {
                k: self.raw_data[k]
                for k in ("timestamps", "positions", "velocities", "accelerations")
            }

    robot = load_robot(
        str(URDF), package_dirs=str(HERE.parents[1] / "models"), load_by_urdf=True
    )
    ident = FixtureIdentification(
        robot, str(HERE / "config" / "ur10_unified_config.yaml")
    )
    ps = ident.identif_config
    ps["ts"] = 1.0 / SAMPLE_RATE_HZ
    ps["validation_data_file"] = "validation"
    ps["act_Jid"] = [ident.model.getJointId(j) for j in JOINTS]
    ps["act_J"] = [ident.model.joints[j] for j in ps["act_Jid"]]
    ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
    ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]
    ident.initialize()
    return ident


# --- baseline ---------------------------------------------------------------


def ols_case(
    derivatives: str, noise: str, seed: int | None, val_seed: int | None
) -> dict:
    """Base OLS on the training split, judged by the frozen protocol."""
    proto = protocol()
    model = nominal_model()
    eliminated = proto["rank"]["eliminated_indices"]
    idx = proto["rank"]["base_indices"]
    names = proto["rank"]["base_names"]
    truth_b = np.array([proto["rank"]["base_truth"][n] for n in names])
    col = np.array([proto["scaling"]["base_column_rms"][n] for n in names])
    scale = np.array(list(proto["scaling"]["effort_nm_per_joint"].values()))

    def W_base(d):
        W = build_regressor_basic(
            _Robot(model), d["q"], d["dq"], d["ddq"], IDENTIF_CONFIG
        )
        return build_regressor_reduced(W, eliminated)[:, idx]

    train = load_split("train", derivatives, noise, seed)
    phi, *_ = np.linalg.lstsq(W_base(train), train["tau"].T.ravel(), rcond=None)
    val = load_split("validation", derivatives, noise, val_seed)
    pred = (W_base(val) @ phi).reshape(6, -1).T
    rmse = np.sqrt(np.mean((pred - val["tau_true"]) ** 2, axis=0))
    scaled = (phi - truth_b) * col
    return {
        "derivatives": derivatives,
        "noise": noise,
        "seed": seed,
        "val_rmse_nm": rmse,
        "val_nrmse_pct": 100 * rmse / scale,
        "base_err_max_nm": float(np.abs(scaled).max()),
        "base_err_rms_nm": float(np.sqrt(np.mean(scaled**2))),
    }


def check_fixture() -> list:
    """Problems found in the committed fixture (empty when it is intact)."""
    problems = []
    entry = manifest()
    for name, digest in entry["files"].items():
        if sha256(FIXTURE_DIR / name) != digest:
            problems.append(f"{name}: hash differs from manifest")
    for split, info in entry["splits"].items():
        n = info["rows"]
        for s, digest in entry["noise"]["draws_sha256"][split].items():
            if draws_sha256(int(s), n) != digest:
                problems.append(f"{split}: noise stream of seed {s} changed")
    return problems


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--write", type=Path, help="regenerate the fixture into this directory"
    )
    parser.add_argument(
        "--optimize",
        type=Path,
        help=f"rerun core's trajectory optimiser; write {WAYPOINTS_FILE} here",
    )
    args = parser.parse_args(argv)
    if args.optimize:
        args.optimize.mkdir(parents=True, exist_ok=True)
        path = args.optimize / WAYPOINTS_FILE
        path.write_text(json.dumps(optimize_training(), indent=2) + "\n")
        print(f"optimised training waypoints written to {path}")
        return
    if args.write:
        entry = generate(args.write)
        print(
            json.dumps(
                {k: entry["splits"][k]["checks"] for k in entry["splits"]}, indent=2
            )
        )
        print(f"fixture written to {args.write}")
        return

    problems = check_fixture()
    for p in problems:
        print(f"PROBLEM: {p}")
    rows = [ols_case("analytic", "none", None, None)]
    for derivatives in ("analytic", "differentiated"):
        for noise in ("low", "high"):
            for s, v in zip(NOISE_SEEDS["train"], NOISE_SEEDS["validation"]):
                rows.append(ols_case(derivatives, noise, s, v))
    rows.append(ols_case("differentiated", "none", None, None))
    df = pd.DataFrame(rows)
    df["val_nrmse_max_pct"] = [r.max() for r in df.val_nrmse_pct]
    df["val_rmse_max_nm"] = [r.max() for r in df.val_rmse_nm]
    summary = df.groupby(["derivatives", "noise"], sort=False).agg(
        cases=("base_err_rms_nm", "size"),
        base_err_rms_nm=("base_err_rms_nm", "mean"),
        base_err_max_nm=("base_err_max_nm", "max"),
        val_rmse_max_nm=("val_rmse_max_nm", "max"),
        val_nrmse_max_pct=("val_nrmse_max_pct", "max"),
    )
    print("Base OLS on the UR10 truth fixture (protocol v1)")
    print(summary.to_string(float_format=lambda x: f"{x:.3g}"))
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
