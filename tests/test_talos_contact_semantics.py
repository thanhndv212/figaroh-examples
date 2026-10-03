"""Observation semantics of the TALOS table-contact calibration (C1, #25).

Pins what the real dataset contains and what the calibration measures, so a
change to either is visible. Numerical behaviour on real data is covered by
test_talos_table_contact_real_data.py. See
docs/development/talos-contact-calibration-audit-2026-10-04.md.
"""

import sys
from pathlib import Path

import pandas as pd
import pytest
import yaml

PROJECT_ROOT = Path(__file__).parent.parent
EXAMPLE_DIR = PROJECT_ROOT / "examples" / "talos_table_contact"
if str(EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_DIR))

from run_calibration_real_data import DATA_DIR, _load_real_touches  # noqa: E402

FILES = {
    "left_train.csv": ("left", 21),
    "left_validation.csv": ("left", 9),
    "right_train.csv": ("right", 29),
    "right_validation.csv": ("right", 9),
}


@pytest.mark.parametrize("side", ["left", "right"])
def test_chain_measures_contact_height_roll_pitch_only(side):
    cfg = yaml.safe_load(open(EXAMPLE_DIR / f"config/talos_table_{side}_config.yaml"))
    c = cfg["calibration"]
    # z, roll, pitch of the contact frame relative to the table; x, y, yaw
    # of the contact point carry no information.
    assert c["markers"][0]["measure"] == [False, False, True, True, True, False]
    assert c["base_frame"] == f"{side}_sole_link"
    assert c["tool_frame"] == f"gripper_{side}_base_link"
    assert c["free_flyer"] is False  # the sole is the fixed root
    assert c["coeff_regularize"] is None


@pytest.mark.parametrize("name", sorted(FILES))
def test_real_rows_are_untimed_contact_postures(name):
    side, n_rows = FILES[name]
    raw = pd.read_csv(DATA_DIR / name, header=None, skiprows=1)
    assert len(raw) == n_rows
    # 5 metadata fields then 32 joint angles; no timestamp column.
    assert raw.shape[1] == 37
    assert (raw.iloc[:, 0] == "gripper").all()
    assert (raw.iloc[:, 1] == f"talos/{side}_gripper").all()
    assert (raw.iloc[:, 2] == "handle").all()
    assert raw.iloc[:, 3].str.startswith("table/contact_").all()
    assert (raw.iloc[:, 4] == "joint_states").all()

    touches = _load_real_touches(DATA_DIR / name)
    joints = [c for c in touches.columns if c.endswith("_joint")]
    assert len(joints) == 32
    assert touches[joints].notna().all().all()
    # One physical table setup per file: there is no session column.
    assert (touches["session_id"] == 0).all()
