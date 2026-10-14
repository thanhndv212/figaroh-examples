"""TIAGo Pro mocap: reproducible checks on the shipped sessions (#97).

Two numbers of the study come from the data and are computed here, not
written down:

- the **measurement noise** of the simulated fixture
  (``reregistration_noise``): the 2026-07-01 and 2026-07-02 sessions repeat
  14 postures (joint difference < 1e-3 rad). Re-registering one session onto
  the other with a rigid transform removes the mocap and base frame drift
  between the days; the scatter left is the measurement noise plus whatever
  drift is not rigid, so it is an **upper bound** and a **substitute** for a
  repeat-posture measurement within one session (which does not exist);
- the **outlier scan** of the training session (``outlier_scan``): which
  rows a registration-only fit flags, and the physical evidence for each
  (joint limits, distance to the commanded posture, singularity). The
  decision is recorded in data/README.md.

    python data_audit.py        # from examples/tiago_pro
"""

from __future__ import annotations

import contextlib
import functools
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

TIAGO_PRO = Path(__file__).resolve().parent
DATA = TIAGO_PRO / "data"
JOINTS = ["torso_lift_joint"] + [f"arm_right_{i}_joint" for i in range(1, 8)]
TRAINING = "calibration_samples_20260805_1246.csv"
SESSION_A = "calibration_samples_20260701_0913.csv"
SESSION_B = "calibration_samples_20260702_0756.csv"
# joint difference below which two rows are the same posture (rad, m)
REPEAT_TOL = 1e-3
Z_FLAG = 3.0  # rows above this many standard deviations are flagged


def repeated_postures(a: pd.DataFrame, b: pd.DataFrame, tol: float = REPEAT_TOL):
    """Row indices ``(ia, ib)`` of the postures of ``a`` that ``b`` repeats."""
    qa, qb = a[JOINTS].to_numpy(), b[JOINTS].to_numpy()
    dist = np.abs(qa[:, None, :] - qb[None, :, :]).max(axis=2)
    ib = dist.argmin(axis=1)
    ia = np.flatnonzero(dist[np.arange(len(qa)), ib] < tol)
    return ia, ib[ia]


def kabsch(P: np.ndarray, Q: np.ndarray):
    """Rotation ``R`` and translation ``t`` minimising ``|R p + t - q|``."""
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, qc - R @ pc


@functools.lru_cache(maxsize=None)
def reregistration_noise(first: str = SESSION_A, second: str = SESSION_B) -> dict:
    """Noise from the postures two sessions repeat, after rigid registration.

    Returns the pair count, the shift and rotation the registration removes
    (the day-to-day frame drift), and ``sigma_mm``: the per-axis standard
    deviation of one measurement, ``sqrt(SSE / (3 n - 6)) / sqrt(2)`` (the
    difference of two measurements has twice the variance; 6 dof are spent
    on the transform).
    """
    a, b = pd.read_csv(DATA / first), pd.read_csv(DATA / second)
    ia, ib = repeated_postures(a, b)
    P = a[["x1", "y1", "z1"]].to_numpy()[ia]
    Q = b[["x1", "y1", "z1"]].to_numpy()[ib]
    R, t = kabsch(P, Q)
    res = (Q - (P @ R.T + t)) * 1000.0
    dof = 3 * len(ia) - 6
    sigma_diff = float(np.sqrt((res**2).sum() / dof))
    return {
        "n_pairs": int(len(ia)),
        "shift_mm": [float(v) for v in (Q - P).mean(0) * 1000.0],
        "rotation_deg": float(np.degrees(np.arccos((np.trace(R) - 1) / 2))),
        "residual_rms_mm": [float(v) for v in np.sqrt((res**2).mean(0))],
        "sigma_diff_mm": sigma_diff,
        "sigma_mm": sigma_diff / np.sqrt(2.0),
    }


def noise_grid(low: float = 0.5, high: float = 3.0) -> tuple:
    """Simulation noise levels (mm): ``low``, the repeat estimate, ``high``."""
    return (low, float(round(reregistration_noise()["sigma_mm"], 2)), high)


def _zscores(v: np.ndarray) -> np.ndarray:
    return (v - v.mean()) / v.std()


def _row_errors(calib) -> tuple:
    """Per-row position (mm) and orientation (deg) error of a fitted model."""
    r = calib._compute_logmap_residuals(
        calib.PEE_measured, calib.get_pose_from_measure(calib.LM_result.x)
    ).reshape(6, -1)
    return (
        1000.0 * np.linalg.norm(r[:3], axis=0),
        np.degrees(np.linalg.norm(r[3:], axis=0)),
    )


def posture_evidence(path: Path, rows) -> list:
    """Physical evidence for ``rows`` of a session.

    Per row: smallest joint-limit margin as a fraction of the joint range,
    the largest gap between the recorded and the commanded posture
    (``optimal_configs.yaml``, the plan the session executed) and the
    smallest singular value of the tool Jacobian (small = near-singular).
    """
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(TIAGO_PRO / "tiago_pro_local.urdf"))
    data = model.createData()
    frame = model.getFrameId("gripper_right_pal_atc_base_link")
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    lo, hi = model.lowerPositionLimit[idx], model.upperPositionLimit[idx]
    df = pd.read_csv(path)
    q = df[JOINTS].to_numpy()
    plan = yaml.safe_load((DATA / "optimal_configs.yaml").read_text())
    plan = np.array(plan["calibration_joint_configurations"])
    gap = np.abs(q[:, None, :] - plan[None]).max(axis=2).min(axis=1)

    def sigma_min(row):
        qq = pin.neutral(model)
        qq[idx] = row
        pin.computeJointJacobians(model, data, qq)
        pin.updateFramePlacements(model, data)
        J = pin.getFrameJacobian(model, data, frame, pin.LOCAL_WORLD_ALIGNED)
        return float(np.linalg.svd(J, compute_uv=False)[-1])

    sv = np.array([sigma_min(r) for r in q])
    margin = np.minimum(q - lo, hi - q) / (hi - lo)
    return {
        "median_gap_rad": float(np.median(gap)),
        "median_sigma_min": float(np.median(sv)),
        "rows": {
            int(i): {
                "min_limit_margin": float(margin[i].min()),
                "commanded_gap_rad": float(gap[i]),
                "sigma_min": float(sv[i]),
                "sigma_min_rank": int(1 + (sv < sv[i]).sum()),
            }
            for i in rows
        },
    }


def outlier_scan(path: Path = DATA / TRAINING) -> dict:
    """Rows a registration-only fit flags (> ``Z_FLAG`` sigma), with evidence.

    Registration only = base frame (6D) and tool frame fitted, nominal joints
    (the baseline of the study). The error of a row is the norm of its
    position (or orientation) residual; its z-score is taken against the
    other rows. ``full_fit`` repeats the scan after the full-parameter fit
    for reference, since the historical note's flagged row came from it.
    """
    from examples.tiago_pro import heldout_protocol as hp

    out = {}
    for label, frames_only, level in (
        ("registration_only", True, "full_params"),
        ("full_fit", False, "full_params"),
    ):
        with contextlib.redirect_stdout(io.StringIO()):
            calib = hp.fit(level, frames_only=frames_only, data_file=str(path))
        pos, ori = _row_errors(calib)
        flagged = {}
        for name, v in (("position_mm", pos), ("orientation_deg", ori)):
            z = _zscores(v)
            top = np.argsort(-z)[:3]
            out.setdefault(label, {})[name] = {
                "rmse": float(np.sqrt(np.mean(v**2))),
                "top_rows": [int(i) for i in top],
                "top_values": [float(v[i]) for i in top],
                "top_z": [float(z[i]) for i in top],
            }
            flagged[name] = [int(i) for i in np.flatnonzero(z > Z_FLAG)]
        out[label]["flagged"] = flagged
    rows = sorted(
        {
            r
            for label in out.values()
            for k in ("position_mm", "orientation_deg")
            for r in label[k]["top_rows"][:1]
        }
    )
    out["evidence"] = posture_evidence(path, rows)
    return out


def main() -> None:
    n = reregistration_noise()
    print(
        f"Repeat postures {SESSION_A[-17:-4]} / {SESSION_B[-17:-4]}: {n['n_pairs']}"
        f"\n  frame drift removed: shift {np.round(n['shift_mm'], 2)} mm, "
        f"rotation {n['rotation_deg']:.2f} deg"
        f"\n  residual rms per axis {np.round(n['residual_rms_mm'], 2)} mm"
        f"\n  sigma (one measurement, upper bound) {n['sigma_mm']:.2f} mm"
        f"\n  simulation noise grid {noise_grid()} mm"
    )
    scan = outlier_scan()
    print(f"\nOutlier scan of {TRAINING} (flag > {Z_FLAG:g} sigma)")
    for label in ("registration_only", "full_fit"):
        for name, s in scan[label].items():
            if name == "flagged":
                print(f"  {label}: flagged rows {s}")
                continue
            print(
                f"  {label} {name}: rmse {s['rmse']:.2f}, top rows {s['top_rows']} "
                f"z {np.round(s['top_z'], 2).tolist()}"
            )
    ev = scan["evidence"]
    print(
        f"  evidence (median gap to plan {ev['median_gap_rad']:.3f} rad, "
        f"median sigma_min {ev['median_sigma_min']:.3f})"
    )
    for r, e in ev["rows"].items():
        print(f"    row {r}: {e}")


if __name__ == "__main__":
    main()
