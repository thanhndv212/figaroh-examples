"""TEMPORARY diagnostics for #48 — not a test (no ``test_`` prefix).

Prints one JSON line per fixture describing how the TALOS calibration
solve behaves on this platform, so Linux CI and local runs can be
compared: versions, input hashes, identifiable parameters, every
least-squares call, Jacobian conditioning, held-out metrics and the
true-model noise floor. Remove before merging the #48 fix.
"""

from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import json
import platform
import sys
import tempfile
from pathlib import Path

import numpy as np
import pinocchio
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT)]

import figaroh.calibration.base_calibration as bc  # noqa: E402
import test_talos_table_contact as single  # noqa: E402
from examples.talos_table_contact.generate_synthetic_data import (  # noqa: E402
    build_dataset,
)
from examples.talos_table_contact.utils.talos_table_tools import (  # noqa: E402
    TalosTableContactCalibration,
)

KEYS = ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg")
calls = []
_real_least_squares = bc.least_squares


def recording_least_squares(fun, x0, **kw):
    res = _real_least_squares(fun, x0, **kw)
    sv = np.linalg.svd(res.jac, compute_uv=False) if res.jac is not None else []
    calls.append(
        dict(
            status=int(res.status),
            nfev=int(res.nfev),
            cost=float(res.cost),
            x_norm=float(np.linalg.norm(res.x)),
            x0_norm=float(np.linalg.norm(x0)),
            sv_max=float(sv[0]) if len(sv) else None,
            sv_min=float(sv[-1]) if len(sv) else None,
            n_sv_below_1e_6_rel=(
                int(np.sum(np.asarray(sv) < 1e-6 * sv[0])) if len(sv) else None
            ),
        )
    )
    return res


bc.least_squares = recording_least_squares


def sha(a) -> str:
    return hashlib.sha256(np.ascontiguousarray(a, dtype=float).tobytes()).hexdigest()[
        :12
    ]


def chain(robot, df, table_poses, contact_offset):
    path = Path(tempfile.mkdtemp()) / "data.csv"
    df.to_csv(path, index=False)
    c = TalosTableContactCalibration(robot, str(single.CONFIG_PATH))
    c.calib_config["data_file"] = str(path)
    c._data_path = str(path)
    c.set_nominal_table_poses(table_poses)
    c.set_nominal_contact_offset(contact_offset)
    c.initialize()
    return c


def run_single():
    with contextlib.redirect_stdout(io.StringIO()):
        df_tr, df_val, gt, robot = build_dataset(
            n_sessions=2,
            n_train_per_session=40,
            n_val_per_session=10,
            seed=42,
            encoder_noise_std=single.ENCODER_NOISE_STD,
            include_truth_objects=True,
        )
    truth = gt["truth_objects"]
    nominal = [
        single.NOMINAL_TABLE_POSE * single.cartesian_to_SE3([dx, dy, 0, 0, 0, 0])
        for dx, dy in gt["session_offsets"]
    ]
    out = dict(
        fixture="single_chain",
        data_sha=dict(train=sha(df_tr.to_numpy()), val=sha(df_val.to_numpy())),
        n_train=len(df_tr),
        n_val=len(df_val),
    )
    for attempt in (1, 2):
        calls.clear()
        with contextlib.redirect_stdout(io.StringIO()):
            c = chain(robot, df_tr, nominal, single.NOMINAL_CONTACT_OFFSET)
            res = c.solve(
                method="lm",
                max_iterations=3,
                outlier_threshold=3.0,
                enable_logging=False,
                html_report=False,
            )
            single._load_split(c, df_val)
            v0 = np.zeros(len(c.calib_config["param_name"]))
            before, after = c.gap_metrics(v0), c.gap_metrics(res.x)
        out[f"attempt_{attempt}"] = dict(
            n_deltaX=c.n_deltaX,
            param_names_sha=hashlib.sha256(
                "|".join(c.calib_config["param_name"]).encode()
            ).hexdigest()[:12],
            calls=list(calls),
            x_sha=sha(res.x),
            before={k: round(before[k], 4) for k in KEYS},
            after={k: round(after[k], 4) for k in KEYS},
        )
        if attempt == 1:
            out["param_names"] = list(c.calib_config["param_name"])
    with contextlib.redirect_stdout(io.StringIO()):
        true_robot = copy.copy(robot)
        true_robot.model = truth["true_model"]
        true_robot.data = truth["true_model"].createData()
        ct = chain(
            true_robot, df_val, truth["true_table_pose"], truth["true_contact_offset"]
        )
        floor = ct.gap_metrics(np.zeros(len(ct.calib_config["param_name"])))
    out["true_model_floor"] = {k: round(floor[k], 4) for k in KEYS}
    return out


if __name__ == "__main__":
    header = dict(
        python=platform.python_version(),
        system=platform.platform(),
        numpy=np.__version__,
        scipy=scipy.__version__,
        pinocchio=pinocchio.__version__,
    )
    print("TALOS_DIAG " + json.dumps(dict(header=header, **run_single())), flush=True)
