# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]
#
# Licensed under the Apache License, Version 2.0.

"""Materialize SDP-reconstructed inertial parameters into a URDF.

Runs the same identification pipeline as ``identification.py``, takes the
soft-SDP full-parameter vector ``theta_r``, converts Pinocchio dynamic
parameters (about the link/joint origin) to URDF ``<inertial>`` (CoM +
inertia about CoM), and writes an updated URDF.

Joint → child-link map comes from the nominal URDF. ``Base_R`` is merged
into Pinocchio's universe and is **not** part of the identified set; it
is left unchanged.

Usage (cwd = ``examples/kitt_1_5/``)::

    python update_model.py
    python update_model.py --output results/urdf/right_arm_robot_identified.urdf
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.kitt_1_5.utils.kitt_1_5_tools import KITT1_5Identification
from figaroh.identification.identification_tools import get_standard_parameters
from figaroh.tools.regressor import build_regressor_basic
from figaroh.tools.robot import load_robot

_INERTIAL_KEYS = (
    "m",
    "mx",
    "my",
    "mz",
    "Ixx",
    "Ixy",
    "Iyy",
    "Ixz",
    "Iyz",
    "Izz",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Write SDP-reconstructed inertias into a KITT1_5 right-arm URDF"
    )
    p.add_argument(
        "--config",
        default="config/kitt_1_5_unified_config.yaml",
        help="Unified identification config",
    )
    p.add_argument(
        "--urdf",
        default="urdf/right_arm_robot.urdf",
        help="Nominal URDF to overlay",
    )
    p.add_argument(
        "--output",
        default="results/urdf/right_arm_robot_identified.urdf",
        help="Output URDF path",
    )
    p.add_argument(
        "--params-csv",
        default="results/urdf/full_parameters_identified.csv",
        help="Also dump the full SIP vector used for the overlay",
    )
    p.add_argument(
        "--verbose", "-v", action="store_true", help="Verbose logging"
    )
    return p.parse_args()


def dynamic_params_to_urdf_inertial(
    m: float,
    mx: float,
    my: float,
    mz: float,
    Ixx: float,
    Ixy: float,
    Iyy: float,
    Ixz: float,
    Iyz: float,
    Izz: float,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Convert Pinocchio dynamic parameters → URDF (mass, com, I_com).

    Pinocchio stores spatial inertia about the link origin with first
    moment ``h = m c`` and ``I_o = I_com + m (‖c‖²𝟙 − c cᵀ)``.
    """
    m = float(m)
    if m <= 1e-12:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    h = np.array([mx, my, mz], dtype=float)
    c = h / m
    I_o = np.array(
        [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]], dtype=float
    )
    I_com = I_o - m * (np.eye(3) * float(np.dot(c, c)) - np.outer(c, c))
    I_com = 0.5 * (I_com + I_com.T)
    return m, c, I_com


def joint_child_link_map(urdf_path: Path) -> Dict[str, str]:
    """Return ``{joint_name: child_link_name}`` from a URDF."""
    root = ET.parse(urdf_path).getroot()
    out: Dict[str, str] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        child = joint.find("child")
        if name and child is not None and child.get("link"):
            out[name] = child.get("link")
    return out


def _fmt(v: float) -> str:
    return f"{float(v):.10g}"


def _set_link_inertial(
    link: ET.Element, mass: float, com: np.ndarray, I_com: np.ndarray
) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.SubElement(inertial, "origin")
    origin.set("xyz", f"{_fmt(com[0])} {_fmt(com[1])} {_fmt(com[2])}")
    origin.set("rpy", origin.get("rpy") or "0 0 0")
    mass_el = inertial.find("mass")
    if mass_el is None:
        mass_el = ET.SubElement(inertial, "mass")
    mass_el.set("value", _fmt(mass))
    iner = inertial.find("inertia")
    if iner is None:
        iner = ET.SubElement(inertial, "inertia")
    iner.set("ixx", _fmt(I_com[0, 0]))
    iner.set("ixy", _fmt(I_com[0, 1]))
    iner.set("ixz", _fmt(I_com[0, 2]))
    iner.set("iyy", _fmt(I_com[1, 1]))
    iner.set("iyz", _fmt(I_com[1, 2]))
    iner.set("izz", _fmt(I_com[2, 2]))


def merge_full_sip(
    standard_parameter: Mapping[str, float],
    theta_r_dict: Mapping[str, float],
) -> Dict[str, float]:
    """URDF prior overwritten by reconstructed ``theta_r`` entries."""
    merged = {k: float(v) for k, v in standard_parameter.items()}
    for k, v in theta_r_dict.items():
        merged[k] = float(v)
    return merged


def p10_for_joint(params: Mapping[str, float], joint: str) -> Optional[Dict[str, float]]:
    out: Dict[str, float] = {}
    for key in _INERTIAL_KEYS:
        name = f"{key}_{joint}"
        if name not in params:
            return None
        out[key] = float(params[name])
    return out


def write_identified_urdf(
    nominal_urdf: Path,
    full_sip: Mapping[str, float],
    output_urdf: Path,
    *,
    joint_names: Optional[Sequence[str]] = None,
) -> Dict[str, str]:
    """Overlay full SIP onto child links of identified joints.

    Returns the ``{joint: link}`` pairs that were updated.
    """
    tree = ET.parse(nominal_urdf)
    root = tree.getroot()
    j2l = joint_child_link_map(nominal_urdf)
    if joint_names is None:
        # Infer joints that have a full p10 in the SIP vector
        bodies = set()
        for k in full_sip:
            for pref in _INERTIAL_KEYS:
                if k.startswith(pref + "_"):
                    bodies.add(k[len(pref) + 1 :])
                    break
        joint_names = sorted(
            j for j in bodies if j in j2l and p10_for_joint(full_sip, j)
        )

    updated: Dict[str, str] = {}
    for jname in joint_names:
        link_name = j2l.get(jname)
        if link_name is None:
            logging.warning("No child link for joint %s — skip", jname)
            continue
        p10 = p10_for_joint(full_sip, jname)
        if p10 is None:
            logging.warning("Incomplete p10 for %s — skip", jname)
            continue
        mass, com, I_com = dynamic_params_to_urdf_inertial(
            p10["m"],
            p10["mx"],
            p10["my"],
            p10["mz"],
            p10["Ixx"],
            p10["Ixy"],
            p10["Iyy"],
            p10["Ixz"],
            p10["Iyz"],
            p10["Izz"],
        )
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            logging.warning("Link %s missing in URDF — skip", link_name)
            continue
        _set_link_inertial(link, mass, com, I_com)
        updated[jname] = link_name

    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    # ElementTree writes without XML declaration / pretty print; fine for URDF
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return updated


def active_torque_rmse(
    identif: KITT1_5Identification,
    phi_std_like: Mapping[str, float],
) -> float:
    """RMSE of ``W @ φ`` vs measured active-joint torques."""
    q = identif.processed_data["positions"]
    dq = identif.processed_data["velocities"]
    ddq = identif.processed_data["accelerations"]
    W = build_regressor_basic(identif.robot, q, dq, ddq, identif.identif_config)
    names = list(identif.standard_parameter.keys())
    phi = np.array([float(phi_std_like.get(k, identif.standard_parameter[k])) for k in names])
    tau_full = W @ phi
    act_idxv = identif.identif_config["act_idxv"]
    n = identif.num_samples
    tau_hat = np.concatenate([tau_full[i * n : (i + 1) * n] for i in act_idxv])
    tau_m = np.asarray(identif.result["torque processed"]).flatten()[: tau_hat.size]
    return float(np.sqrt(np.mean((tau_m - tau_hat) ** 2)))


def run_identification(args: argparse.Namespace) -> KITT1_5Identification:
    robot = load_robot(args.urdf, package_dirs="urdf", load_by_urdf=True)
    identif = KITT1_5Identification(robot=robot, config_file=args.config)
    ps = identif.identif_config
    aj = ps.get("active_joints", [])
    ps["active_joints"] = aj
    ps["act_Jid"] = [identif.model.getJointId(i) for i in aj]
    ps["act_J"] = [identif.model.joints[jid] for jid in ps["act_Jid"]]
    ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
    ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]
    identif.initialize()
    identif.solve(
        decimate=False,
        plotting=False,
        save_results=False,
        html_report=False,
        wls=False,
    )
    return identif


def main(args: argparse.Namespace) -> None:
    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        print(f"Error: URDF not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    print("Running identification + soft-SDP reconstruction ...")
    identif = run_identification(args)
    recon = identif.result.get("reconstruction") or {}
    if recon.get("status") != "ok":
        print(
            f"Error: reconstruction status={recon.get('status')!r}; "
            "enable reconstruction.method=sdp with base_equality=soft",
            file=sys.stderr,
        )
        sys.exit(1)

    theta_r = recon.get("theta_r_dict") or {}
    full_sip = merge_full_sip(identif.standard_parameter, theta_r)

    out_urdf = Path(args.output)
    updated = write_identified_urdf(urdf_path, full_sip, out_urdf)

    params_csv = Path(args.params_csv)
    params_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(params_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["parameter", "value", "source"])
        for k, v in full_sip.items():
            src = "sdp" if k in theta_r else "urdf_prior"
            w.writerow([k, f"{v:.10g}", src])

    rmse_id = float(identif.result.get("rmse norm (N/m)", float("nan")))
    rmse_urdf = active_torque_rmse(identif, identif.standard_parameter)
    rmse_recon = active_torque_rmse(identif, full_sip)

    # Round-trip: reload written URDF and compare masses for updated links
    robot2 = load_robot(str(out_urdf), package_dirs="urdf", load_by_urdf=True)
    sp2 = get_standard_parameters(robot2.model, identif.identif_config)
    # Build φ in original name order where possible
    rmse_reload = active_torque_rmse(identif, sp2)

    print("\n" + "=" * 60)
    print("UPDATE MODEL (identified URDF)")
    print("=" * 60)
    print(f"  Output URDF : {out_urdf}")
    print(f"  Params CSV  : {params_csv}")
    print(f"  Links updated ({len(updated)}):")
    for j, link in updated.items():
        m = full_sip.get(f"m_{j}", float("nan"))
        print(f"    {j} → {link}  (m={m:.4f})")
    print(f"  reconstruction: {recon.get('status')}  "
          f"base_equality={recon.get('base_equality')}  "
          f"‖Mθ−φ_b‖={recon.get('base_residual_norm'):.4g}")
    print(f"  RMSE URDF nominal     : {rmse_urdf:.6f} N·m")
    print(f"  RMSE identified base  : {rmse_id:.6f} N·m")
    print(f"  RMSE SDP θ_r (in-mem) : {rmse_recon:.6f} N·m")
    print(f"  RMSE reloaded URDF    : {rmse_reload:.6f} N·m")
    print("=" * 60)
    print(
        "Note: to refresh MJCF, point urdf_to_mjcf.py at the new URDF "
        "(or copy it over urdf/right_arm_robot.urdf and re-run)."
    )


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s: %(message)s",
    )
    main(args)
