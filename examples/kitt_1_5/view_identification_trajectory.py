#!/usr/bin/env python3
"""Play back the KITT1_5 right-arm identification trajectory on the full robot.

Uses the full-body MuJoCo model (converted from ``urdf/kitt_full_robot.urdf``).
Only the 7 right-arm joints are driven by the identification trajectory; the rest
of the body stays at a fixed home pose.

Collision: right-arm geoms collide only with the rest of the robot (not with
adjacent arm links). If a sample would penetrate, it is skipped and the last
collision-free pose is held — so playback never enters a colliding state.

Usage (from examples/kitt_1_5/):
    python view_identification_trajectory.py
    python view_identification_trajectory.py --speed 0.5
    python view_identification_trajectory.py --cycles 5
    python view_identification_trajectory.py --loop
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("mujoco is required", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent
DEFAULT_MJCF = ROOT / "mjcf" / "kitt_full_robot.xml"
DEFAULT_URDF = ROOT / "urdf" / "kitt_full_robot.urdf"
DEFAULT_TRUTH = ROOT / "data" / "identification_truth.npz"

RIGHT_ARM_JOINTS = [f"right_arm_joint_{i}" for i in range(1, 8)]
RIGHT_ARM_BODIES = (
    "Joint1_R",
    "Joint2_R",
    "Joint3_R",
    "Joint4_R",
    "Joint5_R",
    "Joint6_R",
    "Joint7_R",
    "rightfinger1_Link",
    "rightfinger2_Link",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Full-body MuJoCo playback of right-arm identification trajectory"
    )
    p.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_TRUTH,
        help="Path to identification_truth.npz (default: data/identification_truth.npz)",
    )
    p.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MJCF,
        help=(
            "Full-body model path (MJCF or URDF). "
            f"Default: {DEFAULT_MJCF.relative_to(ROOT)}. "
            f"URDF alternative: {DEFAULT_URDF.relative_to(ROOT)}. "
            "Pass the right-arm MJCF for arm-only mode."
        ),
    )
    p.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = realtime). Default 1.0.",
    )
    p.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="Number of contiguous periods to play (default 1). "
        "Use 0 or --loop for endless seamless cycling.",
    )
    p.add_argument(
        "--loop",
        action="store_true",
        help="Endless seamless multi-cycle playback until the viewer closes "
        "(equivalent to --cycles 0). No hold between periods.",
    )
    p.add_argument(
        "--hold",
        type=float,
        default=1.0,
        help="Seconds to hold the last pose after all cycles finish "
        "(ignored while --loop / --cycles 0). Default 1.0.",
    )
    p.add_argument(
        "--no-collision-guard",
        action="store_true",
        help="Disable collision filtering; apply every trajectory sample as-is.",
    )
    return p.parse_args()


def _joint_addrs(
    model: mujoco.MjModel, names: list[str]
) -> tuple[list[int], list[int]]:
    """Return (qpos addresses, qvel/dof addresses) for named joints."""
    qadr: list[int] = []
    dadr: list[int] = []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError(f"joint not found in model: {name}")
        qadr.append(int(model.jnt_qposadr[jid]))
        dadr.append(int(model.jnt_dofadr[jid]))
    return qadr, dadr


def _body_ids(model: mujoco.MjModel, names: tuple[str, ...]) -> set[int]:
    ids: set[int] = set()
    for name in names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            ids.add(int(bid))
    return ids


def configure_arm_vs_body_collision(model: mujoco.MjModel) -> int:
    """Enable only right-arm ↔ rest-of-robot contacts.

    Masks:
      arm geoms:  contype=2, conaffinity=1
      other geoms: contype=1, conaffinity=2
    Adjacent arm links do not collide with each other; torso/leg self-contacts
    that are always penetrating at home pose are also suppressed.
    """
    arm_bodies = _body_ids(model, RIGHT_ARM_BODIES)
    if not arm_bodies:
        # Arm-only model: leave default collision (or none).
        return 0
    n_arm = 0
    for gi in range(model.ngeom):
        if int(model.geom_bodyid[gi]) in arm_bodies:
            model.geom_contype[gi] = 2
            model.geom_conaffinity[gi] = 1
            n_arm += 1
        else:
            model.geom_contype[gi] = 1
            model.geom_conaffinity[gi] = 2
    return n_arm


def contact_pairs(
    model: mujoco.MjModel, data: mujoco.MjData
) -> list[tuple[str, str, float]]:
    pairs: list[tuple[str, str, float]] = []
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1])
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2])
        pairs.append((b1 or f"body{c.geom1}", b2 or f"body{c.geom2}", float(c.dist)))
    return pairs


def apply_right_arm(
    data: mujoco.MjData,
    home_qpos: np.ndarray,
    qadr: list[int],
    dadr: list[int],
    q_arm: np.ndarray,
    dq_arm: np.ndarray | None,
) -> None:
    data.qpos[:] = home_qpos
    data.qvel[:] = 0.0
    for j, adr in enumerate(qadr):
        data.qpos[adr] = q_arm[j]
    if dq_arm is not None:
        for j, adr in enumerate(dadr):
            data.qvel[adr] = dq_arm[j]


def main() -> None:
    args = parse_args()
    if not args.model.exists():
        print(f"Model not found: {args.model}", file=sys.stderr)
        sys.exit(1)
    if not args.data.exists():
        print(
            f"Trajectory data not found: {args.data}\n"
            "Run: python generate_mujoco_data.py --trajectory fourier",
            file=sys.stderr,
        )
        sys.exit(1)

    n_cycles = 0 if args.loop else max(args.cycles, 0)
    if args.cycles < 0:
        print("--cycles must be >= 0", file=sys.stderr)
        sys.exit(1)

    mj_model = mujoco.MjModel.from_xml_path(str(args.model))
    mj_data = mujoco.MjData(mj_model)

    truth = np.load(args.data)
    t = np.asarray(truth["t"], dtype=float).ravel()
    q = np.asarray(truth["q_true"], dtype=float)
    dq = (
        np.asarray(truth["dq_true"], dtype=float)
        if "dq_true" in truth.files
        else None
    )

    full_body = all(
        mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0
        for n in RIGHT_ARM_JOINTS
    )
    arm_only = (not full_body) and q.shape[1] == mj_model.nq

    if full_body:
        if q.shape[1] != len(RIGHT_ARM_JOINTS):
            print(
                f"Expected {len(RIGHT_ARM_JOINTS)}-DOF right-arm trajectory, "
                f"got q shape {q.shape}",
                file=sys.stderr,
            )
            sys.exit(1)
        qadr, dadr = _joint_addrs(mj_model, RIGHT_ARM_JOINTS)
        n_arm_geoms = configure_arm_vs_body_collision(mj_model)
        home_qpos = np.zeros(mj_model.nq, dtype=float)
        # Prefer named keyframe if present
        kid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if kid >= 0:
            home_qpos[:] = mj_model.key_qpos[kid]
        mode = f"full-body ({args.model.name}), arm-vs-body collision geoms={n_arm_geoms}"
    elif arm_only:
        qadr = list(range(mj_model.nq))
        dadr = list(range(mj_model.nv))
        home_qpos = np.zeros(mj_model.nq, dtype=float)
        n_arm_geoms = 0
        mode = f"arm-only ({args.model.name})"
    else:
        print(
            f"Model nq={mj_model.nq} incompatible with trajectory dim {q.shape[1]} "
            f"and missing right_arm_joint_* names.",
            file=sys.stderr,
        )
        sys.exit(1)

    n = len(t)
    if n < 2:
        print("Trajectory needs at least 2 samples", file=sys.stderr)
        sys.exit(1)

    dt = float(np.median(np.diff(t)))
    period = n * dt
    wrap_err = float(np.max(np.abs(q[-1] - q[0])))
    sleep_dt = dt / max(args.speed, 1e-6)
    noise_scale = float(truth["noise_scale"]) if "noise_scale" in truth else float("nan")
    guard = full_body and (not args.no_collision_guard)

    # Baseline: home + first sample must be collision-free (or we start ungarded)
    apply_right_arm(
        mj_data, home_qpos, qadr, dadr, q[0], None if dq is None else dq[0]
    )
    mujoco.mj_forward(mj_model, mj_data)
    if guard and mj_data.ncon > 0:
        print(
            "[warn] home+q[0] already in collision; listing pairs then continuing "
            "with guard (will hold last safe once one exists):",
            file=sys.stderr,
        )
        for a, b, dist in contact_pairs(mj_model, mj_data)[:8]:
            print(f"  {a} <-> {b}  dist={dist:.4f}", file=sys.stderr)

    cycles_label = "∞" if n_cycles == 0 else str(n_cycles)
    print(
        f"Mode: {mode}\n"
        f"Loaded {n} samples/period, period≈{period:.3f}s, dt≈{dt:.4f}s, "
        f"wrap |q_end-q_start|_∞={wrap_err:.3e}, "
        f"cycles={cycles_label}, speed={args.speed}x, "
        f"collision_guard={'on' if guard else 'off'}, noise_scale={noise_scale}"
    )
    print("Close the MuJoCo window to stop.")

    last_safe_qpos = mj_data.qpos.copy()
    last_safe_qvel = mj_data.qvel.copy()
    skipped = 0
    reported = 0

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -15
        viewer.cam.distance = 3.2
        viewer.cam.lookat[:] = [0.0, 0.0, 0.7]

        wall0 = time.time()
        sample_i = 0
        cycle = 0

        while viewer.is_running():
            if n_cycles != 0 and cycle >= n_cycles:
                break

            local = sample_i % n
            q_arm = q[local]
            dq_arm = None if dq is None else dq[local]
            apply_right_arm(mj_data, home_qpos, qadr, dadr, q_arm, dq_arm)
            mujoco.mj_forward(mj_model, mj_data)

            if guard and mj_data.ncon > 0:
                skipped += 1
                if reported < 5:
                    pairs = contact_pairs(mj_model, mj_data)
                    print(
                        f"[collision] sample {local}: holding last safe pose; "
                        f"{pairs[0][0]} <-> {pairs[0][1]} dist={pairs[0][2]:.4f}"
                    )
                    reported += 1
                mj_data.qpos[:] = last_safe_qpos
                mj_data.qvel[:] = last_safe_qvel
                mujoco.mj_forward(mj_model, mj_data)
            else:
                last_safe_qpos[:] = mj_data.qpos
                last_safe_qvel[:] = mj_data.qvel

            viewer.sync()

            target = wall0 + (sample_i * dt) / max(args.speed, 1e-6)
            delay = target - time.time()
            if delay > 0:
                time.sleep(delay)
            elif sleep_dt > 0 and delay < -2 * sleep_dt:
                wall0 = time.time() - (sample_i * dt) / max(args.speed, 1e-6)

            sample_i += 1
            if sample_i % n == 0:
                cycle += 1

        if args.hold > 0 and n_cycles != 0 and viewer.is_running():
            time.sleep(args.hold)

    if skipped:
        print(f"Collision guard held last safe pose for {skipped} samples.")
    print("Viewer closed.")


if __name__ == "__main__":
    main()
