"""scripts/data_inventory.py: drift in the data inventory is detected."""

import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "data_inventory.py"
spec = importlib.util.spec_from_file_location("data_inventory", SCRIPT)
di = importlib.util.module_from_spec(spec)
spec.loader.exec_module(di)


@pytest.fixture
def repo(tmp_path):
    data = tmp_path / "examples" / "bot" / "data"
    data.mkdir(parents=True)
    (data / "run.csv").write_text("t,q\n0,1\n1,2\n2,3\n")
    (data / "README.md").write_text("ignored\n")
    (tmp_path / "docs").mkdir()
    inventory = {
        "schema": 1,
        "ignore": ["examples/*/data/README.md"],
        "datasets": [
            {
                "id": "bot-run",
                "robot": "Bot",
                "study": "Test",
                "kind": "simulated",
                "role": "training",
                "description": "A run.",
                "paths": ["examples/bot/data/run.csv"],
            }
        ],
        "files": {},
    }
    (tmp_path / "docs" / "data-inventory.json").write_text(json.dumps(inventory))
    assert di.main(["update", "--root", str(tmp_path)]) == 0
    return tmp_path


def _check(root):
    return di.main(["check", "--root", str(root)])


def test_clean_inventory_passes_and_counts_rows(repo):
    assert _check(repo) == 0
    files = json.loads((repo / "docs" / "data-inventory.json").read_text())["files"]
    assert files["examples/bot/data/run.csv"]["rows"] == 3
    assert "README.md" not in "".join(files)


def test_changed_file_is_detected(repo, capsys):
    (repo / "examples/bot/data/run.csv").write_text("t,q\n0,1\n1,9\n2,3\n")
    assert _check(repo) == 1
    assert "sha256 changed" in capsys.readouterr().err


def test_file_without_a_dataset_is_detected(repo, capsys):
    (repo / "examples/bot/data/extra.csv").write_text("a\n1\n")
    assert _check(repo) == 1
    assert "belongs to no dataset" in capsys.readouterr().err


def test_removed_file_is_detected(repo, capsys):
    (repo / "examples/bot/data/run.csv").unlink()
    assert _check(repo) == 1
    assert "missing or unowned" in capsys.readouterr().err


def test_stale_markdown_is_detected(repo, capsys):
    page = repo / "docs" / "data-inventory.md"
    page.write_text(page.read_text() + "edited by hand\n")
    assert _check(repo) == 1
    assert "out of date" in capsys.readouterr().err


def test_file_in_two_datasets_is_rejected(repo, capsys):
    path = repo / "docs" / "data-inventory.json"
    inv = json.loads(path.read_text())
    inv["datasets"].append(dict(inv["datasets"][0], id="dup"))
    path.write_text(json.dumps(inv))
    assert _check(repo) == 1
    assert "several datasets" in capsys.readouterr().err
