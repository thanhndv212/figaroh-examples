"""
Pytest configuration file for the unified config parser tests.
"""

import sys
from pathlib import Path

import pytest

# Add src directory to Python path for figaroh imports
project_root = Path(__file__).parent.parent
figaroh_src = project_root / "figaroh" / "src"
examples_root = project_root / "figaroh-examples"

if str(figaroh_src) not in sys.path:
    sys.path.insert(0, str(figaroh_src))

if str(examples_root) not in sys.path:
    sys.path.insert(0, str(examples_root))


# Impact map: robot markers and the collection guard (#141)
sys.path.insert(0, str(Path(__file__).parent))
import impact  # noqa: E402

IMPACT_MAP = impact.load_impact_map()
GOLDEN_FILE = "tests/test_golden_outputs.py"


# Configure pytest
def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line("markers", "integration: marks tests as integration tests")
    config.addinivalue_line(
        "markers", "real_config: marks tests that use real config files"
    )
    for robot in IMPACT_MAP["robots"]:
        config.addinivalue_line("markers", f"{robot}: tests of the {robot} example")
    config.addinivalue_line("markers", "shared: tests of code shared by robots")
    config.addinivalue_line(
        "markers", "cross_robot: tests spanning robots (smoke, configs, goldens)"
    )


# Markers come from tests/impact.yaml, so ``pytest -m ur10`` needs no edits
# to test files.


def pytest_collection_modifyitems(config, items):
    """Mark items by robot; fail on test files the impact map does not cover."""
    robot_files = impact.robot_test_files(IMPACT_MAP)
    shared_files = impact.shared_test_files(IMPACT_MAP)
    cross = set(IMPACT_MAP.get("always", [])) | set(IMPACT_MAP.get("cross_robot", []))
    unmapped = set()
    for item in items:
        try:
            rel = Path(str(item.fspath)).resolve().relative_to(project_root)
        except ValueError:
            continue
        rel = rel.as_posix()
        if not (rel.startswith("tests/") and Path(rel).name.startswith("test_")):
            continue
        if rel == GOLDEN_FILE:
            callspec = getattr(item, "callspec", None)
            robot = (
                impact.golden_case_for(callspec.id, IMPACT_MAP) if callspec else None
            )
            item.add_marker(
                getattr(pytest.mark, robot) if robot else pytest.mark.cross_robot
            )
            continue
        known = False
        for robot in robot_files.get(rel, []):
            item.add_marker(getattr(pytest.mark, robot))
            known = True
        if rel in shared_files:
            item.add_marker(pytest.mark.shared)
            known = True
        if rel in cross:
            item.add_marker(pytest.mark.cross_robot)
            known = True
        if not known:
            unmapped.add(rel)
    # Every slow item must land in exactly one job of
    # .github/workflows/slow.yml (one per robot, plus cross_robot/shared).
    job_markers = set(IMPACT_MAP["robots"]) | {"shared", "cross_robot"}
    for item in items:
        if item.get_closest_marker("slow") is None:
            continue
        jobs = {m.name for m in item.iter_markers() if m.name in job_markers}
        if len(jobs) != 1 and str(item.fspath).startswith(str(project_root / "tests")):
            raise pytest.UsageError(
                f"slow test {item.nodeid} lands in {len(jobs)} slow-CI jobs "
                f"({sorted(jobs)}); it must be in exactly one (tests/impact.yaml)"
            )
    if unmapped:
        raise pytest.UsageError(
            "test file(s) missing from tests/impact.yaml: "
            + ", ".join(sorted(unmapped))
            + " (list each under a robot, shared entry, always or cross_robot)"
        )
