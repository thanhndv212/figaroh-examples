"""TIAGo mocap calibration: exported URDF and PAL file check (#28).

Fits the reference (``joint_offset``) and ``full_params`` on the training
session, as the held-out protocol does (#27), then:

- writes the URDF (``export_urdf`` with ``joint_corrections()``) and the PAL
  ``geometric_calibration`` (``build_geometric_calibration`` against the
  nominal URDF, figaroh-plus#123);
- reloads the URDF, and the nominal URDF with the PAL keys added to its
  origins (applied here, independently of the exporter), and with the
  metrology frames (``metrology_frames()``) applied outside them predicts
  the marker on every session's postures; the calibrated model must
  predict the same;
- checks that the exported URDF differs from the nominal one only in the
  corrected joints' ``<origin>``, so links, inertias, sensors and the
  nominal file are kept;
- records what is identified (the fitted parameters) and what is
  redistributed (the corrections written to the URDF/PAL file).

Results are in docs/development/tiago-calibration-export.md and checked by
tests/test_tiago_export_check.py. Nothing is applied to a robot. Run from
examples/tiago.
"""

from __future__ import annotations

import hashlib
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.tools.geometric_calibration_export import (  # noqa: E402
    build_geometric_calibration,
)
from figaroh.tools.urdf_exporter import export_urdf  # noqa: E402

from examples.tiago.heldout_protocol import (  # noqa: E402
    JOINTS,
    MOCAP,
    SETS,
    URDF,
    fit,
)

LEVELS = ["joint_offset", "full_params"]
PAL_AXES = ["dx", "dy", "dz", "droll", "dpitch", "dyaw"]


def apply_pal(nominal: Path, geometric_calibration: dict, output: Path) -> Path:
    """Add PAL ``<joint>_<axis>`` deltas to the URDF origins' xyz/rpy.

    What PAL's robot_state_publisher is taken to do with
    ``master_calibration.yaml``; written here, not taken from the exporter.
    """
    tree = ET.parse(str(nominal))
    joints = {j.get("name"): j for j in tree.getroot().findall("joint")}
    for key, value in geometric_calibration.items():
        joint, axis = key.rsplit("_", 1)
        elem = joints[f"{joint}_joint"] if f"{joint}_joint" in joints else joints[joint]
        origin = elem.find("origin")
        if origin is None:
            origin = ET.SubElement(elem, "origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        i = PAL_AXES.index(axis)
        if i < 3:
            xyz[i] += value
        else:
            rpy[i - 3] += value
        origin.set("xyz", " ".join(repr(v) for v in xyz))
        origin.set("rpy", " ".join(repr(v) for v in rpy))
    tree.write(str(output))
    return output


def session_postures(model, path: Path) -> np.ndarray:
    df = pd.read_csv(path)
    q = np.tile(pin.neutral(model), (len(df), 1))
    for j in JOINTS:
        q[:, model.joints[model.getJointId(j)].idx_q] = df[j].to_numpy()
    return q


def changed_elements(nominal: Path, exported: Path) -> dict:
    """``{"joints": [...], "other": [...]}``: what differs between the files.

    Joints whose ``<origin>`` changed numerically (the exporter rewrites the
    numbers of every joint it touches, including zero corrections), and any
    other difference (an element added, removed or changed outside a joint
    origin).
    """

    def signature(elem):
        attrib = dict(elem.attrib)
        if elem.tag == "origin":  # compare numbers, not their formatting
            for k in ("xyz", "rpy"):
                attrib[k] = tuple(
                    np.round([float(v) for v in attrib.get(k, "0 0 0").split()], 12)
                )
        return (elem.tag, tuple(sorted(attrib.items())), (elem.text or "").strip())

    a, b = ET.parse(str(nominal)).getroot(), ET.parse(str(exported)).getroot()
    joints, other = [], []
    if len(a) != len(b):
        other.append("number of top-level elements")
    for ea, eb in zip(a, b):
        name = ea.get("name")
        if ea.tag != eb.tag or name != eb.get("name"):
            other.append(f"{ea.tag} {name} vs {eb.tag} {eb.get('name')}")
            continue
        origin_changed = False
        sa, sb = list(ea.iter()), list(eb.iter())
        if len(sa) != len(sb):
            other.append(f"{ea.tag} {name}: children")
            continue
        for xa, xb in zip(sa, sb):
            if signature(xa) == signature(xb):
                continue
            if ea.tag == "joint" and xa.tag == "origin" and xb.tag == "origin":
                origin_changed = True
            else:
                other.append(f"{ea.tag} {name}: {xa.tag}")
        if origin_changed:
            joints.append(name)
    return {"joints": joints, "other": other}


def check(level: str) -> dict:
    calib = fit(level)
    cfg = calib.calib_config
    corrections = calib.joint_corrections()
    frames = calib.metrology_frames()
    gc = build_geometric_calibration(calib, nominal_urdf=str(URDF))[
        "robot_state_publisher"
    ]["geometric_calibration"]
    digest = hashlib.sha256(URDF.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory() as tmp:
        urdf_out = Path(tmp) / "exported.urdf"
        export_urdf(str(URDF), corrections, output_path=str(urdf_out))
        pal_out = apply_pal(URDF, gc, Path(tmp) / "pal.urdf")
        models = {
            "urdf": pin.buildModelFromUrdf(str(urdf_out)),
            "pal": pin.buildModelFromUrdf(str(pal_out)),
        }
        changed = changed_elements(URDF, urdf_out)
    parity = {}
    for _, f in SETS:
        q = session_postures(calib.model, MOCAP / f)
        c = dict(cfg, NbSample=len(q))
        calibrated = calc_updated_fkm(
            calib.model, calib.model.createData(), calib.LM_result.x, q, c
        )
        fc = dict(c, param_name=list(frames))
        fv = np.array(list(frames.values()))
        parity[f] = {
            k: float(
                np.abs(
                    calc_updated_fkm(m, m.createData(), fv, q, fc) - calibrated
                ).max()
            )
            for k, m in models.items()
        }
    fitted = {
        n: (float(v), float(s))
        for n, v, s in zip(cfg["param_name"], calib.LM_result.x, calib.std_dev)
        if n not in frames
    }
    return {
        "level": level,
        "fitted": fitted,
        "absorbed": list(cfg.get("absorbed_param_name", [])),
        "corrections": corrections,
        "frames": frames,
        "pal": gc,
        "parity": parity,
        "changed": changed,
        "nominal_unchanged": hashlib.sha256(URDF.read_bytes()).hexdigest() == digest,
    }


def _short(name: str) -> str:
    return name.removeprefix("qualisys_").removesuffix("_static_postures.csv")


def main() -> None:
    for level in LEVELS:
        r = check(level)
        print(f"\n== {level}")
        print(f"fitted joint parameters ({len(r['fitted'])}), mrad or mm:")
        for n, (v, s) in r["fitted"].items():
            print(f"  {n:28s} {v * 1e3:8.3f} ± {s * 1e3:.3f}")
        print(f"absorbed by the frames, not fitted: {r['absorbed']}")
        nonzero = {n: v for n, v in r["corrections"].items() if v != 0.0}
        print(f"written to URDF ({len(nonzero)} of {len(r['corrections'])} non-zero):")
        for n, v in nonzero.items():
            print(f"  {n:28s} {v * 1e3:8.3f}")
        print(f"PAL keys: {len(r['pal'])}")
        print("metrology frames (not in the URDF):")
        for n, v in r["frames"].items():
            print(f"  {n:12s} {v:9.5f}")
        for f, p in r["parity"].items():
            print(
                f"  parity {_short(f):15s} URDF {p['urdf']:.1e} m  PAL {p['pal']:.1e} m"
            )
        print(f"URDF changes: origins of {r['changed']['joints']}")
        print(f"  other differences: {r['changed']['other'] or 'none'}")
        print(f"  nominal URDF unchanged: {r['nominal_unchanged']}")


if __name__ == "__main__":
    main()
