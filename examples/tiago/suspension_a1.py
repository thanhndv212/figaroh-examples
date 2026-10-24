"""TIAGo base suspension (#70), amendment A1: the series reading on the confirmation runs.

Amendment A1 (docs/development/tiago-suspension.md, section 13; ``amendment_a1``
in the study config) was approved before any read of ``xyz_mirror_1630`` and
``xyz_mirror_1642``. With the arm load passing through the suspension, the
question is whether the training S1 **predicts** runs it was not fitted on and
whether that prediction is the arm load. Per DOF, on each confirmation run:

- A1-1 transfers: held-out R2 of the training S1 >= ``transfer_r2_min``;
- A1-2 is the arm load: R2 of the S1 prediction against the B2 prediction
  >= ``arm_load_r2_min`` (both from training parameters);
- A1-3 stiffness holds: S1 refitted on the run, ``k`` within ``k_rel_band`` of
  training, neither at its bound;
- A1-4 the base moves: RMS of the base DOF >= ``base_motion_snr_min`` times its
  sd on the noise floor (``postures_1720``, base at rest).

A DOF passing all four on both runs reads "series compliance"; A1-1 failing
while B2 transfers on both reads "rigid base"; anything else "no answer".
Validation (``xyz_1628``) is scored the same way and marked not blind. The
section 8 verdict is not changed. Read once:

    python suspension_a1.py --out ../../tests/golden/tiago_suspension_a1.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import suspension_model as sm  # noqa: E402
from examples.tiago import suspension_study as ss  # noqa: E402

A1 = sm.SETTINGS["amendment_a1"]
MZ_ABOUT = "centre"


def items(r2_s1, r2_arm, k_train, k_run, train_bound, run_bound, snr, r2_b2, a1=A1) -> dict:
    """The four A1 items of one run, (6,) bool each, from its numbers."""
    k_train, k_run = np.asarray(k_train, float), np.asarray(k_run, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        rel = np.abs(k_run - k_train) / np.abs(k_train)
    return {
        "transfers": (np.asarray(r2_s1) >= a1["transfer_r2_min"]).tolist(),
        "arm_load": (np.asarray(r2_arm) >= a1["arm_load_r2_min"]).tolist(),
        "k_holds": ((
            np.nan_to_num(rel, nan=np.inf) <= a1["k_rel_band"]
        ) & ~np.asarray(train_bound, bool) & ~np.asarray(run_bound, bool)).tolist(),
        "base_moves": (np.asarray(snr) >= a1["base_motion_snr_min"]).tolist(),
        "b2_transfers": (np.asarray(r2_b2) >= a1["rigid_b2_r2_min"]).tolist(),
    }


def reading(runs: list) -> list:
    """Per DOF, the A1 reading over the confirmation runs (``items`` of each)."""
    out = []
    for j in range(6):
        every = lambda key: all(bool(r[key][j]) for r in runs)  # noqa: E731
        if all(every(k) for k in ("transfers", "arm_load", "k_holds", "base_moves")):
            out.append("series compliance")
        elif not every("transfers") and every("b2_transfers"):
            out.append("rigid base")
        else:
            out.append("no answer")
    return out


def score_run(s1, b2, r, w, floor_sd) -> dict:
    """A1 numbers and items of one run with the frozen training fits."""
    p_s1, p_b2 = sm.predict(s1, r), sm.predict(b2, r)
    refit = sm.fit_model("S1", r, w)
    x = r.x[r.valid]
    num = {
        "r2_s1": ss._r2(w, p_s1, r.valid),
        "r2_arm": ss._r2(p_b2, p_s1, r.valid),
        "r2_b2": ss._r2(w, p_b2, r.valid),
        "k_run": refit.k(),
        "run_bound": refit.stiffness_bound_dofs(),
        "snr": np.std(x, axis=0) / floor_sd,
    }
    it = items(
        num["r2_s1"], num["r2_arm"], s1.k(), num["k_run"],
        s1.stiffness_bound_dofs(), num["run_bound"], num["snr"], num["r2_b2"],
    )  # fmt: skip
    return {**{k: np.asarray(v).tolist() for k, v in num.items()}, "items": it}


def run(verbose: bool = True) -> dict:
    sessions = sm.load_sessions(verify=True)
    ids = sm.SETTINGS["sessions"]
    tr = sessions[ids["training"]]
    rt, wt = sm.read_regressors(tr), sm.read_target(tr, mz_about=MZ_ABOUT)
    s1 = sm.fit_model(A1["model"], rt, wt)
    b2 = sm.fit_model("B2", rt, wt)
    fl = sm.read_regressors(sessions[ids["noise_floor"]])
    floor_sd = np.std(fl.x[fl.valid], axis=0)
    out = {
        "a1": dict(A1),
        "training": {
            "k": s1.k().tolist(),
            "at_bound": s1.stiffness_bound_dofs().tolist(),
            "b2_mass": float(b2.theta[0]),
        },
        "noise_floor_sd": floor_sd.tolist(),
    }
    val = sessions[ids["validation"]]
    rv = sm.read_regressors(val)
    wv = sm.read_target(val, unlock_validation=True, mz_about=MZ_ABOUT)
    out["validation_not_blind"] = score_run(s1, b2, rv, wv, floor_sd)
    conf = {}
    for sid in ids["confirmation"]:
        if verbose:
            print(f"[a1] reading {sid}", file=sys.stderr)
        s = sessions[sid]
        r = sm.read_regressors(s, unlock_confirmation=True)
        w = sm.read_target(s, unlock_confirmation=True, mz_about=MZ_ABOUT)
        conf[sid] = score_run(s1, b2, r, w, floor_sd)
    out["confirmation"] = conf
    out["reading"] = reading([conf[s]["items"] for s in ids["confirmation"]])
    return out


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, help="write the result JSON here")
    args = parser.parse_args(argv)
    res = ss._clean(run())
    text = json.dumps(res, indent=1)
    if args.out:
        args.out.write_text(text + "\n")
    print(json.dumps(res["reading"], indent=1))


if __name__ == "__main__":
    main()
