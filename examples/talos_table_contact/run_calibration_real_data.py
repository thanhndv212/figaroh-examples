#!/usr/bin/env python3

# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Run the TALOS table-contact calibration against real hardware data.

    python run_calibration_real_data.py

Unlike run_calibration.py (which always runs against synthetic data with a
known, injected ground truth -- see its own docstring), this script uses
the actual joint-encoder recordings from the physical rig described in the
project's manuscript: `data/real/{left,right}_{train,validation}.csv`,
copied from the deprecated FIGAROH repository's
`data/talos/contacts/{compiled_measurements_left_1028,
compiled_measurements_right_1107, validation_left_1107,
validation_right_1107}.csv` -- the exact files the older prototype script
(`scripts/talos_contact_calibration.py`) used, which the manuscript's own
2.3x cross-validated pose-error reduction was computed from. There is no
injected ground truth here (there's nothing to inject -- this is real,
already-recorded data), so "verification" here means the same thing it
means for the manuscript itself: does the flush-contact gap actually drop,
on training postures and on a genuinely disjoint held-out set, when this
implementation's own faithful (PLANE_TPL/CONTACT_TPL, target-is-zero)
cost function is fit to it -- not a synthetic sanity check.

Caveat worth stating plainly: the left chain's training data was recorded
on 2022-10-28 and its validation data on 2022-11-07 (11 days apart, based
on the file names) -- a different day than the right chain's own
train/validation, both recorded together on 2022-11-07. Cross-validating
across that gap is standard practice (joint-placement errors are
mechanical, not day-of-experiment properties) but means, unlike the
synthetic round-trip tests, this script cannot rule out the table having
been physically nudged between the two days for the left chain specifically.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

from generate_synthetic_data import (
    NOMINAL_CONTACT_OFFSET,
    NOMINAL_TABLE_POSE,
    _load_robot,
)
from utils.talos_table_tools import MultiChainCalibration, TalosTableContactCalibration
from export_report import build_two_chain_report, write_html_report, write_yaml_report

from figaroh.tools.geometric_calibration_export import export_geometric_calibration_yaml
from figaroh.tools.run_archive import archive_run, compute_run_dir

HERE = Path(__file__).parent
DATA_DIR = HERE / "data" / "real"
LEFT_CONFIG = HERE / "config" / "talos_table_left_config.yaml"
RIGHT_CONFIG = HERE / "config" / "talos_table_right_config.yaml"

# Column order for the 32 joint values in every data row of
# data/real/*.csv (copied from the deprecated repo's
# data/talos/contacts/*.csv), taken from the one file among the four
# whose header happens to still carry it correctly
# (compiled_measurements_left_1028.csv / left_train.csv here). The other
# three files' headers are missing their leading placeholder columns,
# which silently shifts every column by 4 if read with pandas' own
# header -- see _load_real_touches.
_CANONICAL_JOINT_ORDER = [
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
    "gripper_left_joint",
    "gripper_right_joint",
    "head_1_joint",
    "head_2_joint",
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_6_joint",
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint",
    "torso_1_joint",
    "torso_2_joint",
]
# Every data row is: gripper side, gripper frame, "handle", contact
# marker id, "joint_states", then the 32 joint values in the order above.
_N_METADATA_FIELDS = 5


def _load_real_touches(path) -> pd.DataFrame:
    """Parse one of data/real/*.csv from its raw row layout rather than
    trusting its header (see module-level comment above), returning a
    DataFrame with the canonical joint names as columns, the contact
    marker id kept for traceability, and session_id=0 (one physical
    table setup per file -- there is no session column in the source
    data)."""
    raw = pd.read_csv(path, header=None, skiprows=1)
    n_joints = len(_CANONICAL_JOINT_ORDER)
    marker_ids = raw.iloc[:, 3].astype(str).to_numpy()
    values = raw.iloc[:, _N_METADATA_FIELDS : _N_METADATA_FIELDS + n_joints].to_numpy(
        dtype=float
    )
    df = pd.DataFrame(values, columns=_CANONICAL_JOINT_ORDER)
    df["contact_marker"] = marker_ids
    df["session_id"] = 0
    return df


def _load_split(calib: TalosTableContactCalibration, df: pd.DataFrame) -> None:
    """Point an already-initialized calibration at a different dataset,
    reusing its existing base-parameter basis (a held-out split). The
    real CSVs have no session_id column -- everything is one physical
    table setup per file, session 0, matching set_nominal_table_poses'
    single-entry call below."""
    N = len(df)
    q0 = np.asarray(calib.calib_config["q0"], dtype=float)
    q = np.tile(q0, (N, 1))
    for j_id, name in enumerate(calib.model.names[1:], start=1):
        name = str(name)
        if name in df.columns:
            idx = calib.model.joints[j_id].idx_q
            q[:, idx] = df[name].to_numpy()
    calib.q_measured = q
    calib.session_ids = np.zeros(N, dtype=int)
    calib._fk_config["NbSample"] = N
    # _compute_logmap_residuals (core) reads calib_config["NbSample"] and
    # PEE_measured's own length, not _fk_config's -- all three must stay
    # in sync for a held-out split (PEE_measured is always the zero
    # vector -- the flush-contact target -- just resized).
    calib.calib_config["NbSample"] = N
    calib.PEE_measured = np.zeros(3 * N)


def _print_metrics(label: str, before: dict, after: dict) -> None:
    print(f"\n{label}")
    for key, unit in (
        ("z_rmse_mm", "mm"),
        ("roll_rmse_deg", "deg"),
        ("pitch_rmse_deg", "deg"),
    ):
        factor = before[key] / after[key] if after[key] > 1e-12 else float("inf")
        print(
            f"  {key:16s} {before[key]:8.4f} {unit} -> {after[key]:8.4f} {unit}"
            f"   ({factor:5.1f}x reduction)"
        )


def _build_chain(robot, config_path, train_csv, asset_id=None):
    df_train = _load_real_touches(train_csv)
    staged_path = Path(str(train_csv) + ".staged.csv")
    df_train.to_csv(staged_path, index=False)

    calib = TalosTableContactCalibration(robot, str(config_path))
    calib.calib_config["data_file"] = str(staged_path)
    calib._data_path = str(staged_path)
    if asset_id:
        # Distinguishes the left/right chains in the run archive -- both
        # configs otherwise share the same (unspecified) asset_id, which
        # would collide compute_run_dir's path if both chains solve
        # within the same wall-clock second.
        instance = dict(calib.calib_config.get("instance") or {})
        instance["asset_id"] = asset_id
        calib.calib_config["instance"] = instance
    calib.set_nominal_table_poses([NOMINAL_TABLE_POSE])
    calib.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
    calib.initialize()
    staged_path.unlink(missing_ok=True)
    return calib


def _run_single_chain(
    label, robot, config_path, train_csv, val_csv, results_root=None, archive=True
):
    print(f"\n{'=' * 60}\n{label}\n{'=' * 60}")
    side = "left" if "left" in str(train_csv) else "right"
    calib = _build_chain(robot, config_path, train_csv, asset_id=f"talos-{side}")
    n_train = calib._fk_config["NbSample"]
    df_val = _load_real_touches(val_csv)

    print(
        f"Chain: {calib.calib_config['start_frame']} -> "
        f"{calib.calib_config['end_frame']}"
    )
    print(f"Training postures:   {n_train}")
    print(f"Validation postures: {len(df_val)}")
    print(f"Identifiable joint-placement (Delta X) parameters: {calib.n_deltaX}")

    var0 = np.zeros(len(calib.calib_config["param_name"]))
    train_before = calib.gap_metrics(var0)

    # Standard FIGAROH calibration entry point (as examples/tiago/calibration.py
    # uses) -- see run_calibration.py's own comment on solve()/get_pose_from_measure
    # for how this class's plane/contact gap residual feeds through core's own
    # SE3 log-map machinery instead of a bespoke one.
    result = calib.solve(
        method="lm",
        max_iterations=3,
        outlier_threshold=3.0,
        enable_logging=False,
        html_report=False,
    )
    train_after = calib.gap_metrics(result.x)
    _print_metrics("Training-set gap:", train_before, train_after)

    # Held-out validation done manually (not via BaseCalibration's own
    # _compute_validation_metrics) -- see run_calibration.py's comment on
    # TalosTableContactCalibration._compute_validation_metrics for why.
    _load_split(calib, df_val)
    val_before = calib.gap_metrics(var0)
    val_after = calib.gap_metrics(result.x)
    _print_metrics("Held-out validation gap:", val_before, val_after)

    split = calib.split_params(result.x)
    plane = split["plane"]
    contact = split["contact"]
    print(
        "\nRecovered corrections (no ground truth to compare against -- "
        "this is real, already-uncalibrated hardware):"
    )
    print(
        f"  plane    z={plane['plane_z_s0'] * 1e3:+.3f}mm "
        f"phix={np.rad2deg(plane['plane_phix_s0']):+.4f}deg "
        f"thetay={np.rad2deg(plane['plane_thetay_s0']):+.4f}deg"
    )
    print(
        f"  contact  z={contact['contact_z'] * 1e3:+.3f}mm "
        f"phix={np.rad2deg(contact['contact_phix']):+.4f}deg "
        f"thetay={np.rad2deg(contact['contact_thetay']):+.4f}deg"
    )

    if results_root is not None:
        run_dir = compute_run_dir(calib, root=str(Path(results_root) / "runs"))
        html_path = str(run_dir / f"{side}_calibration_report.html")
        yaml_path = str(run_dir / f"{side}_master_calibration.yaml")
        calib.export_html_report(output_path=html_path)
        export_geometric_calibration_yaml(
            calib,
            yaml_path,
            header_comment=f"TALOS table-contact calibration -- {side} chain (real data)",
        )
        if archive:
            archive_run(calib, run_dir)
        print(f"\nRun archived to:\n  {run_dir}")

    return calib, result


def _run_two_chain(robot, results_root=None):
    print(f"\n{'=' * 60}\nTwo-chain, shared torso (real data)\n{'=' * 60}")
    calib_left = _build_chain(
        robot, LEFT_CONFIG, DATA_DIR / "left_train.csv", asset_id="talos-dual"
    )
    calib_right = _build_chain(
        robot, RIGHT_CONFIG, DATA_DIR / "right_train.csv", asset_id="talos-dual"
    )
    n_train_left = calib_left._fk_config["NbSample"]
    n_train_right = calib_right._fk_config["NbSample"]
    coupler = MultiChainCalibration(calib_left, calib_right)

    print(f"Left Delta X: {calib_left.n_deltaX}  Right Delta X: {calib_right.n_deltaX}")
    print(f"Union Delta X: {coupler.n_dx}  " f"(shared: {coupler.shared_dx_names})")

    var0 = np.zeros(len(coupler.param_name))
    train_before = coupler.gap_metrics(var0)
    result = coupler.solve_lm()
    print(
        f"\nLM solve: success={result.success}  cost={result.cost:.3e}  nfev={result.nfev}"
    )
    train_after = coupler.gap_metrics(result.x)
    _print_metrics("Left training gap:", train_before["left"], train_after["left"])
    _print_metrics("Right training gap:", train_before["right"], train_after["right"])

    df_left_val = _load_real_touches(DATA_DIR / "left_validation.csv")
    df_right_val = _load_real_touches(DATA_DIR / "right_validation.csv")
    _load_split(calib_left, df_left_val)
    _load_split(calib_right, df_right_val)
    val_before = coupler.gap_metrics(var0)
    val_after = coupler.gap_metrics(result.x)
    _print_metrics("Left held-out gap:", val_before["left"], val_after["left"])
    _print_metrics("Right held-out gap:", val_before["right"], val_after["right"])

    if results_root is not None:
        # MultiChainCalibration isn't a BaseCalibration, so it has no
        # _run_provenance and can't use compute_run_dir/archive_run --
        # mirror their timestamped-directory convention by hand instead
        # of dumping straight into results/ (see export_report.py's
        # module docstring for why this class needs its own reporting).
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        run_dir = Path(results_root) / "runs" / "talos-dual" / "two_chain" / ts
        run_dir.mkdir(parents=True, exist_ok=True)
        report = build_two_chain_report(
            coupler,
            result,
            train_before,
            train_after,
            val_before,
            val_after,
            n_train_left=n_train_left,
            n_train_right=n_train_right,
            n_val_left=len(df_left_val),
            n_val_right=len(df_right_val),
        )
        write_yaml_report(report, str(run_dir / "two_chain_calibration_report.yaml"))
        write_html_report(
            report,
            str(run_dir / "two_chain_calibration_report.html"),
            title="TALOS Table-Contact Calibration -- Two-Chain Shared Torso (Real Data)",
        )
        print(f"\nRun archived to:\n  {run_dir}")


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="If set, write YAML + HTML calibration reports here "
        "(default: results/, use --no-save-results to skip).",
    )
    parser.add_argument(
        "--no-save-results",
        action="store_true",
        help="Don't write any report files, just print to stdout.",
    )
    parser.add_argument(
        "--archive",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Archive each single-chain run to results/runs/<asset>/"
            "calibration/<timestamp>/ (provenance, config snapshot, "
            "parameters, and the HTML report) and append a summary line "
            "to results/runs/index.jsonl. Never overwrites a prior run. "
            "Use --no-archive to skip. Ignored with --no-save-results."
        ),
    )
    args = parser.parse_args()
    save_results = not args.no_save_results
    archive = save_results and args.archive
    results_root = args.output_dir or str(HERE / "results") if save_results else None

    robot = _load_robot()

    _run_single_chain(
        "Left chain (real data: left_sole_link -> gripper_left_base_link)",
        robot,
        LEFT_CONFIG,
        DATA_DIR / "left_train.csv",
        DATA_DIR / "left_validation.csv",
        results_root=results_root,
        archive=archive,
    )
    _run_single_chain(
        "Right chain (real data: right_sole_link -> gripper_right_base_link)",
        robot,
        RIGHT_CONFIG,
        DATA_DIR / "right_train.csv",
        DATA_DIR / "right_validation.csv",
        results_root=results_root,
        archive=archive,
    )
    _run_two_chain(robot, results_root=results_root)


if __name__ == "__main__":
    main()
