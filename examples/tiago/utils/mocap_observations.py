"""TIAGo Qualisys postures in the data contract form (examples#17).

The shipped session files (``data/calibration/mocap/``, extracted by
:mod:`mocap_extraction`) become
:class:`~figaroh.data.observations.PoseObservations` with what was implicit:
- all four tracked points, named BL, BR, TR, TL;
- the frame they are expressed in (the Qualisys ``base_frame`` body, fixed
  to the robot base);
- the session and the source file with its sha256;
- each posture's row in the file.

The roles of the sessions come from the protocol manifest
(``data/calibration/mocap/protocol.yaml``), not from the data.

Calibration fits one point today (``x1..z1`` = BL); select it with
``obs.select_points(["BL"])``. figaroh-plus#119 would use more.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd

from figaroh.data import DataSource, PoseObservations, Protocol, Session

MOCAP = Path(__file__).resolve().parents[1] / "data" / "calibration" / "mocap"
PROTOCOL = MOCAP / "protocol.yaml"
POINTS = ("BL", "BR", "TR", "TL")  # x1..z1 ... x4..z4
FRAME = "qualisys:base_frame"


def session_date(session_id: str) -> str:
    """``2021-11-30-1544`` -> ``2021-11-30``."""
    return session_id[:10]


def mocap_observations(
    path: Path, model, calib_config: dict, session_id: str
) -> PoseObservations:
    """One session file as PoseObservations, all four points, positions only.

    Joint columns follow the calibration's active joints
    (``calib_config["actJoint_idx"]``); ``sample_index`` is the file row.
    """
    path = Path(path)
    df = pd.read_csv(path)
    joints = [model.names[i] for i in calib_config["actJoint_idx"]]
    columns = [f"{a}{k}" for k in range(1, 5) for a in "xyz"]
    missing = [c for c in columns + joints if c not in df.columns]
    if missing:
        raise KeyError(f"{path.name}: missing columns {missing}")
    values = np.full((len(df), len(POINTS), 6), np.nan)
    for k in range(len(POINTS)):
        for d, axis in enumerate("xyz"):
            values[:, k, d] = df[f"{axis}{k + 1}"].to_numpy(dtype=float)
    measured = np.tile([True] * 3 + [False] * 3, (len(POINTS), 1))
    session = Session(id=session_id, date=session_date(session_id))
    return PoseObservations(
        joint_names=joints,
        q=df[joints].to_numpy(dtype=float),
        values=values,
        point_names=POINTS,
        measurability=measured,
        frame=FRAME,
        registered_to=calib_config.get("start_frame", "universe"),
        orientation="none (positions only)",
        sample_index=df.index.to_numpy(),
        source=DataSource.from_files(
            [path],
            adapter="examples.tiago.mocap_observations",
            session=session,
            notes=(
                "static plateaus, robot/mocap clocks aligned (#67, "
                "mocap_extraction.py); the file also holds shipped_row, "
                "t_start_robot, t_end_robot (s, robot clock) and marker_std_mm"
            ),
        ),
    )


def protocol_observations(
    model, calib_config: dict, protocol_path: Path = PROTOCOL
) -> Dict[str, Tuple[str, PoseObservations]]:
    """``{session_id: (role, observations)}`` for every protocol session.

    The manifest's sha256 are checked first: a changed file is refused.
    """
    protocol = Protocol.load(protocol_path)
    protocol.verify(root=Path(protocol_path).parent)
    out = {}
    for s in protocol.sessions:
        (name,) = s.files
        out[s.id] = (
            s.role,
            mocap_observations(
                Path(protocol_path).parent / name, model, calib_config, s.id
            ),
        )
    return out
