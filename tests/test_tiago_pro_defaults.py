"""TIAGo Pro calibration: the script's default input files exist.

The default --data named an undated file while the shipped sessions are
dated, so `python run_calibration.py --urdf ...` failed out of the box.
"""

import sys
from pathlib import Path

import pytest

pytest.importorskip("pinocchio")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples" / "tiago_pro"))

import run_calibration  # noqa: E402


@pytest.mark.parametrize("name", ["_DATA_DEFAULT", "_CONFIG"])
def test_default_inputs_exist(name):
    assert getattr(run_calibration, name).exists()
