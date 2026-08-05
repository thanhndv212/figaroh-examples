#!/usr/bin/env python3
"""
Generate optimal calibration configurations for Tiago Pro right arm.

Uses Figaroh's D-optimality criterion (SOCP) to select configurations from a
random pool that maximally excite the kinematic parameters to identify.

Usage:
    python3 generate_optimal_configs.py
    python3 generate_optimal_configs.py --pool-size 500 --output data/optimal_configs.yaml
"""

import argparse
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import trimesh
import viser
import viser.transforms as vtf
import yaml

from figaroh.calibration.calibration_tools import (
    get_param_from_yaml,
    add_base_name,
    add_pee_name,
    calculate_base_kinematics_regressor,
)

_HERE = Path(__file__).parent
_URDF_DEFAULT = _HERE / "urdf" / "tiago_pro.urdf"
_SRDF = _HERE / "urdf" / "tiago_pro.srdf"
_MODELS_DIR = _HERE.parents[1] / "models"
_CONFIG = _HERE / "config" / "tiago_pro_calibration_config.yaml"
_OUT_DEFAULT = _HERE / "data" / "optimal_configs.yaml"


def _pkg_dirs():
    return [str(_MODELS_DIR)]


def _load_mesh(path: str):
    try:
        mesh = trimesh.load(path, force="mesh")
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(mesh.dump())
        return mesh
    except Exception:
        return None


ACTIVE_JOINTS = [
    "torso_lift_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]

_RANGE_FRACTION = 0.85


# ── D-optimality ─────────────────────────────────────────────────────────────

def _rearrange_rb(R_b, param):
    Rb_rearr = np.empty_like(R_b)
    ci = param["calibration_index"]
    ns = param["NbSample"]
    for i in range(ci):
        for j in range(ns):
            Rb_rearr[j * ci + i, :] = R_b[i * ns + j]
    return Rb_rearr


def _sub_info_matrices(R_rearr, param):
    ci = param["calibration_index"]
    ns = param["NbSample"]
    return [
        R_rearr[it * ci:(it + 1) * ci, :].T @ R_rearr[it * ci:(it + 1) * ci, :]
        for it in range(ns)
    ]


def _run_socp(sub_mats):
    try:
        import picos as pc
    except ImportError:
        print("picos not installed — falling back to Detmax.")
        return None

    n = len(sub_mats)
    problem = pc.Problem()
    w = pc.RealVariable("w", n, lower=0)
    t = pc.RealVariable("t", 1)
    Mw = pc.sum(w[i] * sub_mats[i] for i in range(n))
    problem.add_constraint(1 | w <= 1)
    problem.add_constraint(t <= pc.DetRootN(Mw))
    problem.set_objective("max", t)
    problem.solve(solver="cvxopt")
    return [float(w.value[i]) for i in range(n)]


def _run_detmax(sub_mats, n_choose):
    import random
    pool = list(range(len(sub_mats)))

    def crit(indices):
        M = sum(sub_mats[i] for i in indices)
        try:
            d = float(np.linalg.det(M))
            if d <= 0:
                return 0.0
            return d ** (1 / M.shape[0])
        except Exception:
            return 0.0

    cur = random.sample(pool, n_choose)
    remaining = [i for i in pool if i not in cur]

    for _ in range(100):
        best_add, best_crit = None, crit(cur)
        for k in remaining:
            c = crit(cur + [k])
            if c > best_crit:
                best_crit, best_add = c, k
        if best_add is None:
            break
        cur.append(best_add)
        remaining.remove(best_add)

        worst_rm, worst_crit = None, float("inf")
        for j in cur:
            tmp = [x for x in cur if x != j]
            c = crit(tmp)
            if c < worst_crit:
                worst_crit, worst_rm = c, j
        if worst_rm is not None and worst_rm != best_add:
            cur.remove(worst_rm)
            remaining.append(worst_rm)
        else:
            break

    weights = [0.0] * len(sub_mats)
    for i in cur:
        weights[i] = 1.0 / len(cur)
    return weights


# ── Kinematics helpers ───────────────────────────────────────────────────────

def _get_joint_limits(model, joint_names, fraction=_RANGE_FRACTION):
    lbs, ubs = [], []
    for jname in joint_names:
        jid = model.getJointId(jname)
        if jid >= model.njoints:
            raise ValueError(f"Joint '{jname}' not found in model.")
        idx_q = model.joints[jid].idx_q
        center = (model.upperPositionLimit[idx_q] + model.lowerPositionLimit[idx_q]) / 2
        half = (model.upperPositionLimit[idx_q] - model.lowerPositionLimit[idx_q]) / 2 * fraction
        lbs.append(center - half)
        ubs.append(center + half)
    return np.array(lbs), np.array(ubs)


def _q_from_active(model, active_vals, joint_names):
    q = pin.neutral(model)
    for jname, val in zip(joint_names, active_vals):
        jid = model.getJointId(jname)
        if jid < model.njoints and model.joints[jid].nq == 1:
            q[model.joints[jid].idx_q] = val
    return q


# ── Viser visualization ──────────────────────────────────────────────────────

def _check_collisions(model, collision_model, q):
    collision_data = pin.GeometryData(collision_model)
    pin.computeCollisions(model, model.createData(), collision_model, collision_data, q, False)
    return any(r.isCollision() for r in collision_data.collisionResults)


def _display_config(server, mesh_handles, model, visual_model, visual_data,
                    collision_model, data, q, idx, total, joint_names, active_vals):
    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, visual_model, visual_data)

    for i, handle in mesh_handles.items():
        T = visual_data.oMg[i]
        handle.position = T.translation
        handle.wxyz = vtf.SO3.from_matrix(T.rotation).wxyz

    in_collision = _check_collisions(model, collision_model, q)
    status = "  ⚠ COLLISION" if in_collision else "  OK"

    print(f"\n{'─'*50}")
    print(f"Config {idx + 1}/{total}{status}")
    for jname, val in zip(joint_names, active_vals):
        print(f"  {jname:30s}  {np.degrees(val):+7.2f}°")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=str(_URDF_DEFAULT))
    parser.add_argument("--config", default=str(_CONFIG))
    parser.add_argument("--pool-size", type=int, default=500)
    parser.add_argument("--n-configs", type=int, default=50,
                        help="Number of configurations to select (default: 50)")
    parser.add_argument("--output", "-o", default=str(_OUT_DEFAULT))
    parser.add_argument("--no-viser", action="store_true")
    args = parser.parse_args()

    print(f"Loading model from {args.urdf} ...")
    pkg_dirs = _pkg_dirs()
    model = pin.buildModelFromUrdf(args.urdf)
    try:
        _, collision_model, visual_model = pin.buildModelsFromUrdf(
            args.urdf, package_dirs=pkg_dirs
        )
    except ValueError as e:
        print(f"[warn] geometry loading failed ({e})\n  → collision checking disabled.")
        collision_model = pin.GeometryModel()
        visual_model    = pin.GeometryModel()
    data = model.createData()

    collision_model.addAllCollisionPairs()
    pin.removeCollisionPairsFromXML(model, collision_model, _SRDF.read_text(), verbose=False)
    print(f"Collision pairs active: {len(collision_model.collisionPairs)}")

    # Figaroh's get_param_from_yaml only needs robot.model + q0
    class _Robot:
        def __init__(self, m):
            self.model = m
            self.data = m.createData()
            self.q0 = pin.neutral(m)

    with open(args.config) as f:
        config = yaml.safe_load(f)
    param = get_param_from_yaml(_Robot(model), config["calibration"])
    param["known_baseframe"] = False
    param["known_tipframe"] = False

    lbs, ubs = _get_joint_limits(model, ACTIVE_JOINTS)
    print(f"\nGenerating {args.pool_size} collision-free candidate configurations ...")
    rng = np.random.default_rng(42)
    collision_data = pin.GeometryData(collision_model)
    pool_active = []
    pool_q = []
    n_tried = 0
    while len(pool_active) < args.pool_size:
        n_tried += 1
        active_vals = rng.uniform(lbs, ubs)
        q = _q_from_active(model, active_vals, ACTIVE_JOINTS)
        pin.computeCollisions(model, model.createData(), collision_model, collision_data, q, False)
        if any(r.isCollision() for r in collision_data.collisionResults):
            continue
        pool_active.append(active_vals)
        pool_q.append(q)
    pool_active = np.array(pool_active)
    pool_q = np.array(pool_q)
    print(f"Found {args.pool_size} valid configs out of {n_tried} tried.")

    print("Computing kinematic regressors ...")
    param["NbSample"] = args.pool_size
    _, R_b, _, _, _ = calculate_base_kinematics_regressor(
        pool_q, model, data, param
    )
    add_base_name(param)
    add_pee_name(param)
    R_rearr = _rearrange_rb(R_b, param)
    sub_mats = _sub_info_matrices(R_rearr, param)

    n_params = R_b.shape[1]
    ci = param["calibration_index"]
    n_min = n_params // ci + 1
    n_choose = max(args.n_configs, n_min)
    print(f"\nIdentifiable parameters: {n_params}")
    print(f"Minimum configurations needed: {n_min}")
    print(f"Target configurations: {n_choose}")

    print(f"\nRunning Detmax to select {n_choose} configurations ...")
    weights = _run_detmax(sub_mats, n_choose)

    chosen = sorted(
        [i for i, w in enumerate(weights) if w > 1e-5],
        key=lambda i: weights[i],
        reverse=True,
    )
    print(f"\nSelected {len(chosen)} optimal configurations.")

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    output_data = {
        "calibration_joint_names": ACTIVE_JOINTS,
        "calibration_joint_configurations": [pool_active[i].tolist() for i in chosen],
    }
    with open(args.output, "w") as f:
        yaml.dump(output_data, f, sort_keys=False, default_flow_style=False)
    print(f"Saved to {args.output}")

    if args.no_viser:
        return

    print("\nStarting Viser at http://localhost:8080 ...")
    print("Controls: Enter → next config | q → quit")
    server = viser.ViserServer()
    time.sleep(0.5)

    q0 = pin.neutral(model)
    visual_data = pin.GeometryData(visual_model)
    pin.forwardKinematics(model, data, q0)
    pin.updateGeometryPlacements(model, data, visual_model, visual_data)

    print("Loading meshes into Viser ...")
    mesh_handles = {}
    for i, geom_obj in enumerate(visual_model.geometryObjects):
        mesh = _load_mesh(geom_obj.meshPath)
        if mesh is None:
            continue
        T = visual_data.oMg[i]
        vertices = np.array(mesh.vertices, dtype=np.float32) * geom_obj.meshScale
        faces = np.array(mesh.faces, dtype=np.uint32)
        handle = server.scene.add_mesh_simple(
            f"robot/{geom_obj.name}",
            vertices=vertices,
            faces=faces,
            position=T.translation,
            wxyz=vtf.SO3.from_matrix(T.rotation).wxyz,
            color=(0.8, 0.8, 0.8),
        )
        mesh_handles[i] = handle
    print(f"Loaded {len(mesh_handles)} mesh objects.")

    for idx, config_idx in enumerate(chosen):
        active_vals = pool_active[config_idx]
        q = _q_from_active(model, active_vals, ACTIVE_JOINTS)
        _display_config(server, mesh_handles, model, visual_model, visual_data,
                        collision_model, data, q, idx, len(chosen), ACTIVE_JOINTS, active_vals)
        print("Press Enter for next config, 'q' to quit.")
        line = input().strip().lower()
        if line == "q":
            break

    print("\nDone. Optimal configs saved to:", args.output)


if __name__ == "__main__":
    main()
