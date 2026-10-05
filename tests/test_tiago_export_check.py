"""TIAGo exported URDF and PAL file (#28).

Runs examples/tiago/export_check.py on the shipped mocap sessions and checks
what docs/development/tiago-calibration-export.md records: the reloaded URDF
and the PAL-corrected URDF, with the metrology frames applied outside them,
predict what the calibrated model predicts on every session; only the
corrected joints' origins change; the nominal URDF is untouched.
"""

import sys
from pathlib import Path

import pytest

TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"
PARITY_M = 1e-9  # the exporter writes 12 significant digits


@pytest.fixture(scope="module", params=["joint_offset", "full_params"])
def result(request, monkeypatch_module):
    monkeypatch_module.chdir(TIAGO)
    sys.path.insert(0, str(TIAGO.parents[1]))
    from examples.tiago import export_check

    return export_check.check(request.param)


@pytest.fixture(scope="module")
def monkeypatch_module():
    mp = pytest.MonkeyPatch()
    yield mp
    mp.undo()


def test_urdf_and_pal_reproduce_the_calibrated_model(result):
    assert len(result["parity"]) == 4  # training, validation, 2 confirmation
    for session, parity in result["parity"].items():
        assert parity["urdf"] < PARITY_M, session
        assert parity["pal"] < PARITY_M, session


def test_only_corrected_joint_origins_change(result):
    assert result["changed"]["other"] == []
    corrected = {
        n.split("_", 1)[1] if n.startswith("offset") else n.split("_", 2)[2]
        for n, v in result["corrections"].items()
        if v != 0.0
    }
    assert set(result["changed"]["joints"]) == corrected
    assert result["nominal_unchanged"]


def test_identified_vs_written(result):
    """Frames stay out of the corrections; parameters the frames absorb are
    written as zero; the PAL file carries the same joints."""
    corrections, frames = result["corrections"], result["frames"]
    assert set(frames) == {
        "base_px",
        "base_py",
        "base_pz",
        "base_phix",
        "base_phiy",
        "base_phiz",
        "pEEx_1",
        "pEEy_1",
        "pEEz_1",
    }
    assert not set(frames) & set(corrections)
    for name in result["absorbed"]:
        assert abs(corrections.get(name, 0.0)) < 1e-9
    pal_joints = {k.rsplit("_", 1)[0] + "_joint" for k in result["pal"]}
    assert pal_joints == set(result["changed"]["joints"])
    if result["level"] == "joint_offset":
        # one offset per joint: identified values are written unchanged
        for name, (value, _) in result["fitted"].items():
            assert corrections[name] == pytest.approx(value, abs=1e-12)
        assert result["fitted"]["offsetRZ_arm_5_joint"][0] == pytest.approx(
            -49.8e-3, abs=0.1e-3
        )
