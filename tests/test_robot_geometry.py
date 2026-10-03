"""Every example robot loads its geometry the way its scripts do (#15).

A missing mesh fails here instead of in the middle of an example. Meshes that
are not vendored under ``models/`` come from ``scripts/fetch_models.py`` at
pinned revisions; see ``models/README.md``.
"""

import os
from pathlib import Path

import pytest

from figaroh.tools.robot import load_robot

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

FETCH_HINT = (
    "Fetch the pinned mesh packages and point ROS_PACKAGE_PATH at them:\n"
    "  python scripts/fetch_models.py\n"
    '  export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py '
    '--print-ros-package-path)"\n'
    "See models/README.md."
)

# (robot dir, URDF, load_robot kwargs) exactly as the example scripts call it.
CASES = [
    ("ur10", "urdf/ur10_robot.urdf", {"package_dirs": "../../models"}),
    ("tiago", "urdf/tiago_48_schunk.urdf", {"robot_pkg": "tiago_description"}),
    ("tiago", "urdf/tiago_48_hey5.urdf", {"robot_pkg": "tiago_description"}),
    ("talos", "urdf/talos_full_v2.urdf", {"package_dirs": "../../models"}),
    (
        "staubli_tx40",
        "urdf/tx40_mdh_modified.urdf",
        {"package_dirs": "../../models"},
    ),
]


def _mesh_paths(geometry_model):
    """File-backed mesh paths; primitives (BOX, CYLINDER, ...) have none."""
    return [
        g.meshPath
        for g in geometry_model.geometryObjects
        if g.meshPath and os.sep in g.meshPath
    ]


@pytest.mark.parametrize(
    "robot_dir, urdf, kwargs",
    CASES,
    ids=[f"{d}/{Path(u).stem}" for d, u, _ in CASES],
)
def test_example_robot_geometry_resolves(monkeypatch, robot_dir, urdf, kwargs):
    monkeypatch.chdir(EXAMPLES / robot_dir)

    try:
        robot = load_robot(urdf, load_by_urdf=True, **kwargs)
    except ValueError as e:
        pytest.fail(f"{robot_dir}/{urdf}: {e}\n{FETCH_HINT}")

    for kind in ("collision_model", "visual_model"):
        meshes = _mesh_paths(getattr(robot, kind))
        assert meshes, f"{robot_dir}/{urdf}: no mesh geometry in {kind}"
        missing = [m for m in meshes if not os.path.isfile(m)]
        assert not missing, f"{kind} meshes not found: {missing}\n{FETCH_HINT}"


def test_tiago_simplified_collision_model_builds(monkeypatch):
    """The primitive collision model used by tiago/optimal_trajectory.py.

    Pinocchio 4 accepts only GeometryObject(name, joint, placement, geometry);
    the old (geometry, placement) order failed there before any test ran (#59).
    """
    from examples.tiago.utils.simplified_collision_model import (
        build_tiago_simplified,
    )

    monkeypatch.chdir(EXAMPLES / "tiago")
    robot = load_robot(
        "urdf/tiago_48_schunk.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )

    robot = build_tiago_simplified(robot)

    names = {g.name for g in robot.geom_model.geometryObjects}
    assert {
        "torso_up_box",
        "torso_low_box",
        "base_cap",
        "forearm_cap",
        "head_cap",
    } <= names
    assert len(robot.geom_model.collisionPairs) == 4
