"""TX40 physically consistent fit and URDF export (single recording)."""

import numpy as np
import pinocchio as pin
import pytest

from examples.staubli_tx40 import update_model as um


@pytest.fixture(scope="module")
def fitted(tmp_path_factory):
    import os

    out = tmp_path_factory.mktemp("tx40") / "tx40_identified.urdf"
    cwd = os.getcwd()
    os.chdir(os.path.join(os.path.dirname(um.__file__)))
    try:
        args = um.parse_args(["--output", str(out), "--second-solver", "none"])
        robot = um.load_robot(args.urdf, package_dirs="../../models")
        iden = um.physical_fit(robot, args)
        assert iden.result["selected"]["status"] == "accepted"
        path = iden.export_urdf(args.urdf, output_path=str(out))
    finally:
        os.chdir(cwd)
    return robot, iden, path


def test_export_matches_estimate(fitted):
    _, iden, path = fitted
    assert um.export_parity(iden, path) < 1e-8


def test_exported_inertias_are_physical(fitted):
    # Core's pseudo-inertia feasibility tolerance. Link 6 is a point mass in
    # the CAD URDF, so its tensor sits on the boundary at round-off level.
    _, _, path = fitted
    model = pin.buildModelFromUrdf(path)
    for j in range(1, model.njoints):
        inertia = model.inertias[j]
        assert inertia.mass > 0
        assert np.linalg.eigvalsh(inertia.inertia).min() > -1e-8


def test_masses_stay_near_cad(fitted):
    robot, iden, _ = fitted
    values = dict(zip(iden.selected.names, iden.selected.values))
    for j in range(1, robot.model.njoints):
        name = robot.model.names[j]
        cad = robot.model.inertias[j].mass
        assert abs(values[f"m_{name}"] / cad - 1) < 0.2, name


def test_physical_fit_costs_little_torque_error(fitted):
    _, iden, _ = fitted
    per_joint = iden.result["selected"]["effort_rmse_fit_per_joint"]
    rmse_pf = [per_joint[j] for j in iden.fit_joints()]
    rmse_base = um.base_fit_rmse(iden)
    assert np.all(np.array(rmse_pf) <= 1.1 * np.array(rmse_base))
