"""Selector for change-scoped testing (#141), run on fake diffs.

No git and no fits: ``impact.select`` is pure, and the map is read from
``tests/impact.yaml``.
"""

from pathlib import Path

import pytest

import impact

ROOT = Path(__file__).resolve().parent.parent
IMPACT_MAP = impact.load_impact_map()


def pick(*paths):
    return impact.select(list(paths), IMPACT_MAP)


def test_ur10_only_diff_selects_ur10():
    sel = pick("examples/ur10/calibration.py", "tests/test_ur10_signal_processing.py")
    assert not sel.full
    assert sel.robots == ["ur10"]
    assert sel.golden_ids == ["ur10/"]
    assert "tests/test_ur10_identification_truth.py" in sel.test_files
    assert "tests/test_tiago_calibration_truth.py" not in sel.test_files
    assert impact.script_keys(sel.robots, IMPACT_MAP) == ["ur10"]
    for always in IMPACT_MAP["always"]:
        assert always in sel.test_files


def test_calibration_harness_selects_its_users():
    sel = pick("examples/calibration_harness/protocol.py")
    assert not sel.full
    assert set(sel.robots) == {"tiago", "tiago_pro", "talos_table_contact", "ur10"}
    assert "tests/test_calibration_harness_simulation.py" in sel.test_files


def test_robot_test_file_reruns_only_itself():
    sel = pick("tests/test_ur10_signal_processing.py")
    assert sel.robots == []
    assert "tests/test_ur10_signal_processing.py" in sel.test_files
    assert "tests/test_ur10_identification_truth.py" not in sel.test_files


@pytest.mark.parametrize(
    "path",
    [
        "tests/conftest.py",
        "validate.py",
        "models/ur10/ur10.urdf",
        "tests/golden/golden_outputs.json",
        "tests/test_golden_outputs.py",
        "tests/test_contract_adapters.py",
        "pyproject.toml",
        "brand_new_dir/thing.py",
    ],
)
def test_full_suite_triggers(path):
    sel = pick("examples/ur10/calibration.py", path)
    assert sel.full
    assert any(path in r for r in sel.reasons)


def test_docs_only_selects_always_set_only():
    sel = pick("docs/development/environment.md", "README.md")
    assert not sel.full
    assert sel.robots == []
    assert sel.test_files == sorted(IMPACT_MAP["always"])
    assert sel.golden_ids == []


def test_data_inventory_runs_always_set_only():
    sel = pick("docs/data-inventory.json")
    assert not sel.full
    assert sel.robots == []
    assert "tests/test_data_inventory.py" in sel.test_files


def test_empty_diff_runs_always_set():
    sel = pick()
    assert not sel.full and sel.robots == []
    assert sel.reasons


@pytest.mark.parametrize(
    "paths",
    [
        ["examples/ur10/x.py"],
        ["examples/run_record.py"],
        ["docs/foo.md"],
        ["tests/conftest.py"],
        ["unmapped.bin"],
        [],
    ],
)
def test_reasons_never_empty(paths):
    assert pick(*paths).reasons


def test_glob_semantics():
    assert impact.matches("examples/ur10/a/b.py", ["examples/ur10/**"])
    assert not impact.matches("examples/ur10_extra/a.py", ["examples/ur10/**"])
    assert impact.matches("docs/sub/README.md", ["*.md"])
    assert impact.matches("docs/data-inventory.json", ["docs/data-inventory.*"])


def test_every_test_file_is_mapped():
    mapped = impact.mapped_test_files(IMPACT_MAP)
    on_disk = {p.relative_to(ROOT).as_posix() for p in (ROOT / "tests").glob("test_*.py")}
    assert on_disk <= mapped, sorted(on_disk - mapped)
    assert mapped <= on_disk, f"map lists missing files: {sorted(mapped - on_disk)}"


def test_golden_ids_cover_every_case():
    import sys

    sys.path.insert(0, str(ROOT / "tests" / "golden"))
    import golden_outputs

    for case in golden_outputs.CASES:
        assert impact.golden_case_for(case, IMPACT_MAP), case


def test_slow_workflow_matrix_matches_impact_map():
    """.github/workflows/slow.yml runs one job per impact.yaml robot plus a
    cross_robot/shared job; the hard-coded matrix must not drift."""
    import re

    text = (ROOT / ".github" / "workflows" / "slow.yml").read_text()
    match = re.search(r"^\s*robot:\s*\[([^\]]*)\]", text, re.MULTILINE)
    assert match, "no `robot: [...]` matrix in slow.yml"
    matrix = [name.strip() for name in match.group(1).split(",") if name.strip()]
    assert sorted(matrix) == sorted([*IMPACT_MAP["robots"], "cross_robot_shared"])
    assert "slow and (cross_robot or shared)" in text
    assert 'MARKER="slow and ${{ matrix.robot }}"' in text


def test_no_test_file_belongs_to_two_slow_jobs():
    """A slow test must run in exactly one slow-CI job."""
    robot_files = impact.robot_test_files(IMPACT_MAP)
    shared = impact.shared_test_files(IMPACT_MAP)
    cross = set(IMPACT_MAP.get("always", [])) | set(IMPACT_MAP.get("cross_robot", []))
    for rel in impact.mapped_test_files(IMPACT_MAP):
        if rel == "tests/test_golden_outputs.py":
            continue  # one robot or cross_robot per case (conftest)
        jobs = list(robot_files.get(rel, [])) + ["shared"] * (rel in shared)
        jobs += ["cross_robot"] * (rel in cross)
        assert len(jobs) == 1, f"{rel} is in several jobs: {jobs}"


def test_impact_map_edit_selects_always_set_only():
    """Adding a test file to the map must not escalate a PR to the full suite."""
    sel = pick("tests/impact.yaml")
    assert not sel.full
    assert not sel.robots
