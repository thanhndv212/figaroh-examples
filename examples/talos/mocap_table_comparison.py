"""TALOS mocap against table contact on the shared torso and left arm (#172, C3).

The table-contact study (#98, examples/talos_table_contact) fits the chain
left_sole_link -> gripper_left_base_link from gap measurements; the mocap
study fits base_link -> arm_left_7_link from poses. Both use core's
``full_params`` placement errors in each joint's own frame, so the torso and
left-arm parameters (torso_1, torso_2, arm_left_1..7) name the same physical
errors; their values are not comparable one by one (each chain has its own
gauge: the table chain's legs and sole, the mocap chain's waist frame).
The comparison is therefore on predictions:

- ``R_table``: the table fit's torso and left-arm parameters (its structural
  reference model, legs, plane and contact dropped) on the mocap training
  run, with ``B`` and ``T`` re-registered on all its rows;
- ``R_nominal``: the same with nominal joints (the registration-only fit);
- ``R_mocap``: the k-fold error of the model the mocap study selected
  (mocap_heldout.select_model).

Pre-registered verdict (docs/development/talos-mocap.md):

- ``agrees``: ``R_table <= R_nominal - 0.5 (R_nominal - R_mocap)`` (the table
  parameters recover at least half of what the mocap fit recovers);
- ``partial``: ``R_table < R_nominal``;
- ``disagrees``: otherwise.

Also reported: the position RMS difference between the two registered
predictions (table parameters vs the selected mocap fit) on the training
postures, the gauge-invariant form of "the same geometry".

    python mocap_table_comparison.py            # from examples/talos
"""

from __future__ import annotations

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.parameter import BASE_TPL  # noqa: E402

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.talos import mocap_heldout as mh  # noqa: E402

TABLE = project_root / "examples" / "talos_table_contact"
#: the table-contact study's reference model (its config default)
TABLE_MODEL = "structural"
SHARED = tuple(mh.joint_names("left"))
AGREE_FRACTION = 0.5


def table_fit():
    """The left table-contact chain's reference fit (#98)."""
    if str(TABLE) not in sys.path:
        sys.path.insert(0, str(TABLE))
    from examples.talos_table_contact import heldout_protocol as tc

    plan = harness.load_plan(tc.CHAINS["left"][1])  # verifies every sha256
    train = plan.folds[0].training[0].path
    with contextlib.redirect_stdout(io.StringIO()):
        fitted = tc.fit_model(tc._load_robot(), "left", TABLE_MODEL, train)
    names = fitted.calib.calib_config["param_name"]
    return dict(zip(names, map(float, fitted.x))), plan.label


def shared_params(params: dict) -> dict:
    """The torso and left-arm placement errors among ``params``."""
    return {n: v for n, v in params.items() if mh.joint_of(n) in SHARED}


def registered(calib, joint: dict, df: pd.DataFrame, B0) -> list:
    """``B * FK_left(joint) * T`` with ``B``, ``T`` registered on ``df``."""
    q = mh.configurations(calib.model, df, "left")
    F = mh.chain_poses(calib, joint, q, "left", "full_params")
    Y = mh.measured_poses(df)
    return mh.register(F, Y, np.zeros(len(df), int), B0)["pred"], Y


def verdict(r_table: float, r_nominal: float, r_mocap: float) -> str:
    if r_table <= r_nominal - AGREE_FRACTION * (r_nominal - r_mocap):
        return "agrees"
    if r_table < r_nominal:
        return "partial"
    return "disagrees"


def compare(selected_calib, r_mocap: float, table=None) -> dict:
    """C3 on the mocap training run; ``selected_calib`` is the mocap study's
    selected model fitted on the whole run, ``r_mocap`` its k-fold error."""
    params, label = table if table is not None else table_fit()
    joint = shared_params(params)
    df = pd.read_csv(mh.MOCAP / mh.TRAINING)
    mine = mh.fitted_params(selected_calib)
    B0 = mh.from_xyzrpy([mine[n] for n in BASE_TPL])
    calib = mh._nominal_calibrator()
    pred_t, Y = registered(calib, joint, df, B0)
    pred_0, _ = registered(calib, {}, df, B0)
    q = mh.configurations(selected_calib.model, df, "left")
    pred_m = mh.chain_poses(selected_calib, mine, q, "left")
    r_table = mh.rms(mh.pose_errors(pred_t, Y)[0])
    r_nominal = mh.rms(mh.pose_errors(pred_0, Y)[0])
    return {
        "table_plan": label,
        "table_model": TABLE_MODEL,
        "shared_params": sorted(joint),
        "n_table_params": len(params),
        "R_table_mm": r_table,
        "R_nominal_mm": r_nominal,
        "R_mocap_kfold_mm": float(r_mocap),
        "R_table_ori_deg": mh.rms(mh.pose_errors(pred_t, Y)[1]),
        "prediction_difference_mm": mh.rms(mh.pose_errors(pred_t, pred_m)[0]),
        "verdict": verdict(r_table, r_nominal, r_mocap),
    }


def main() -> None:
    kfold = mh.run_kfold()
    best, _ = mh.select_model(kfold)
    calib = mh.fit_model(best)
    out = compare(calib, kfold[best][mh.KFOLD_METRIC])
    print(f"Table contact ({out['table_plan']}), {out['table_model']}: "
          f"{len(out['shared_params'])} shared parameters of {out['n_table_params']}")
    print(f"Mocap training run, position RMSE with B and T re-registered:")
    print(f"  nominal joints        {out['R_nominal_mm']:.2f} mm")
    print(f"  table-contact joints  {out['R_table_mm']:.2f} mm "
          f"({out['R_table_ori_deg']:.2f} deg)")
    print(f"  mocap {best} (k-fold) {out['R_mocap_kfold_mm']:.2f} mm")
    print(f"  table vs mocap predictions differ by {out['prediction_difference_mm']:.2f} mm")
    print(f"C3: {out['verdict']}")


if __name__ == "__main__":
    main()
