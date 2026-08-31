"""
End-to-end smoke tests for example scripts.

Verifies that example scripts can be imported and run without crashing.
Import tests run by default; initialization (subprocess) tests are marked
``slow`` and ``integration`` so they can be excluded with::

    pytest tests/test_smoke.py -m 'not slow'
"""

from __future__ import annotations

import importlib
import os
import subprocess
import sys
from pathlib import Path

import pytest

# Path to the examples directory (repo_root / examples)
EXAMPLES_ROOT = Path(__file__).resolve().parent.parent / "examples"


# ===========================================================================
# Import tests  (fast — no special markers)
# ===========================================================================

SCRIPTS_TO_IMPORT = [
    "examples.ur10.calibration",
    "examples.ur10.identification",
    "examples.ur10.optimal_config",
    "examples.ur10.optimal_trajectory",
    "examples.tiago.calibration",
    "examples.tiago.identification",
    "examples.tiago.optimal_config",
    "examples.tiago.optimal_trajectory",
    "examples.tiago.suspension_identification",
    "examples.tiago.backlash_empirical_surface",
    "examples.talos.calibration_upperbody",
    "examples.talos.update_model",
    "examples.staubli_tx40.identification",
]


@pytest.mark.parametrize(
    "module_name",
    SCRIPTS_TO_IMPORT,
    ids=[m.rpartition(".")[-1] for m in SCRIPTS_TO_IMPORT],
)
def test_script_imports(module_name: str) -> None:
    """Verify each example script can be imported without errors.

    This tests import-time correctness — syntax errors, missing
    dependencies, and path issues.  The actual script logic does **not**
    execute (all scripts have ``if __name__ == "__main__"`` guards).
    """
    # Clear any cached imports so each test starts fresh
    if module_name in sys.modules:
        del sys.modules[module_name]
    try:
        importlib.import_module(module_name)
    except SystemExit:
        # argparse --help or similar can raise SystemExit(0); harmless
        pass


# ===========================================================================
# Subprocess smoke tests  (slow, integration)
# ===========================================================================
# Each entry: (robot_directory, script_name, timeout_seconds)
# These are run as isolated subprocesses from the robot's working directory
# so that relative paths (urdf/, config/, data/) resolve correctly.

SUBPROCESS_SCRIPTS: list[tuple[str, str, int]] = [
    ("ur10", "calibration.py", 120),
    ("ur10", "identification.py", 120),
    ("tiago", "calibration.py", 120),
    ("tiago", "identification.py", 120),
    ("tiago", "optimal_config.py", 120),
    ("talos", "calibration_upperbody.py", 120),
    ("talos", "update_model.py", 120),
    ("staubli_tx40", "identification.py", 120),
]


@pytest.mark.slow
@pytest.mark.integration
@pytest.mark.parametrize(
    "robot_dir,script_name,timeout",
    SUBPROCESS_SCRIPTS,
    ids=[f"{d}/{s.replace('.py', '')}" for d, s, _ in SUBPROCESS_SCRIPTS],
)
def test_script_subprocess(robot_dir: str, script_name: str, timeout: int) -> None:
    """Verify each example script runs without crashing via subprocess.

    Runs the script in a subprocess from its robot directory so that
    relative paths (``urdf/``, ``config/``, ``data/``) resolve correctly.
    Sets ``MPLBACKEND=Agg`` to prevent matplotlib from opening interactive
    windows.
    """
    script_path = EXAMPLES_ROOT / robot_dir / script_name
    cwd = EXAMPLES_ROOT / robot_dir

    if not script_path.exists():
        pytest.skip(f"Script not found: {script_path}")

    # talos/update_model.py requires calibration_results.npz produced by
    # calibration_upperbody.py.  If the file is missing, the script will
    # fail with FileNotFoundError, so we skip gracefully.
    if robot_dir == "talos" and script_name == "update_model.py":
        calib_file = EXAMPLES_ROOT / robot_dir / "data" / "calibration_results.npz"
        if not calib_file.exists():
            pytest.skip(
                f"calibration_results.npz not found at {calib_file} — "
                f"run calibration_upperbody.py first"
            )

    env = {**os.environ, "MPLBACKEND": "Agg"}
    try:
        result = subprocess.run(
            [sys.executable, str(script_path)],
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
        )
    except subprocess.TimeoutExpired:
        pytest.fail(f"Script {robot_dir}/{script_name} timed out after {timeout}s")

    assert result.returncode == 0, (
        f"Script {robot_dir}/{script_name} failed "
        f"(exit code {result.returncode}):\n"
        f"--- stdout (last 30 lines) ---\n"
        + "\n".join(result.stdout.strip().splitlines()[-30:])
        + f"\n--- stderr (last 30 lines) ---\n"
        + "\n".join(result.stderr.strip().splitlines()[-30:])
    )
