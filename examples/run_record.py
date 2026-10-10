# Copyright [2021-2026] Thanh Nguyen

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Reproduction record for an archived run (examples#30).

The core archive (``figaroh.tools.run_archive``) stores provenance, the
config snapshot, stages, parameters and, when the script writes them, the
report and verdict. What only the example script knows is not there: how it
was invoked, the processing it applied outside the config (truncation,
decimation, velocity lag), extra inputs such as a protocol manifest, and
generated files it wrote elsewhere (results ``.npz``, exported URDF).

:func:`write_reproduction_record` writes those to ``reproduction.json`` in
the run directory, then checks the whole directory against the reproduction
checklist and records what is missing by name. Generated files are
referenced by path and sha256, not copied. ``python -m examples.run_record
RUN_DIR`` audits any archive, including ones written before this record
existed; it exits 1 when an item is missing or incomplete.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

SCHEMA_VERSION = 1
RECORD = "reproduction.json"
UNKNOWN = ("unknown", "unavailable", None, "")

# item -> what it has to show
CHECKLIST = {
    "invocation": "command line and working directory",
    "revisions": "examples and core commits, both clean",
    "config": "config snapshot and the config file sha256",
    "model": "nominal URDF sha256",
    "inputs": "sha256 of every input file",
    "processing": "processing applied outside the config",
    "splits": "training/validation data and the validation source",
    "solver": "fit stage status and solver message",
    "selected_stage": "the stage the reported parameters come from",
    "report": "report.html",
    "export": "parameters.csv and every generated file, hashes matching",
}


def sha256(path: Path) -> Optional[str]:
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def describe_file(path, **extra) -> Dict[str, Any]:
    """``{"path", "sha256", **extra}``; sha256 is None for a missing file."""
    return {"path": str(path), "sha256": sha256(path), **extra}


def _load_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def _item(status: str, detail: str) -> Dict[str, str]:
    return {"status": status, "detail": detail}


def _check_files(files: Dict[str, Dict[str, Any]], root: Path) -> List[str]:
    """Names of declared files whose sha256 is unknown or no longer matches."""
    bad = []
    for name, spec in files.items():
        recorded = spec.get("sha256")
        path = Path(spec.get("path", ""))
        path = path if path.is_absolute() else root / path
        if recorded in UNKNOWN or sha256(path) != recorded:
            bad.append(name)
    return bad


def _check_validation_directory(spec, splits, cwd: Path) -> bool:
    """A directory input is covered by the loader's explicit source files.

    Directories have no file sha256. The data-contract validation split
    records which files were actually consumed; require all of them to be
    inside the declared directory and still match their recorded hashes.
    """
    if spec.get("path") in UNKNOWN:
        return False
    directory = Path(spec["path"])
    directory = (directory if directory.is_absolute() else cwd / directory).resolve()
    validation = (splits or {}).get("validation") or {}
    files = validation.get("files", {}) if isinstance(validation, dict) else {}
    if not directory.is_dir() or not files:
        return False
    for name, recorded in files.items():
        path = Path(name)
        path = (path if path.is_absolute() else cwd / path).resolve()
        if (
            not path.is_relative_to(directory)
            or recorded in UNKNOWN
            or sha256(path) != recorded
        ):
            return False
    return True


def audit(run_dir) -> Dict[str, Dict[str, str]]:
    """Check one run directory; ``{item: {"status", "detail"}}``.

    Status is ``ok``, ``incomplete`` (present but not reproducible as
    recorded) or ``missing``. Paths in the archive are relative to the
    directory the script ran in, recorded as ``invocation.cwd``.
    """
    run_dir = Path(run_dir)
    prov = _load_json(run_dir / "provenance.json") or {}
    verdict = _load_json(run_dir / "verdict.json")
    stages = _load_json(run_dir / "stages.json") or {}
    record = _load_json(run_dir / RECORD) or {}
    cwd = Path(record.get("invocation", {}).get("cwd", "."))
    out: Dict[str, Dict[str, str]] = {}

    argv = record.get("invocation", {}).get("argv")
    out["invocation"] = (
        _item("ok", " ".join(argv)) if argv else _item("missing", f"no {RECORD}")
    )

    sw = prov.get("software", {})
    core = sw.get("figaroh_revision", {}) or {}
    commits = (sw.get("git_commit"), core.get("commit"))
    if not prov:
        out["revisions"] = _item("missing", "no provenance.json")
    elif any(c in UNKNOWN for c in commits):
        out["revisions"] = _item(
            "incomplete", f"examples {commits[0]}, core {commits[1]}"
        )
    else:
        dirty = [
            name
            for name, flag in (
                ("examples", sw.get("git_dirty")),
                ("core", core.get("dirty")),
            )
            if flag is not False
        ]
        detail = f"examples {commits[0][:8]}, core {commits[1][:8]}"
        out["revisions"] = (
            _item("incomplete", f"{detail}; uncommitted changes: {', '.join(dirty)}")
            if dirty
            else _item("ok", detail)
        )

    config = prov.get("config", {})
    if not (run_dir / "config.snapshot.yaml").exists():
        out["config"] = _item("missing", "no config.snapshot.yaml")
    elif config.get("sha256") in UNKNOWN:
        out["config"] = _item("incomplete", "config file sha256 unknown")
    else:
        out["config"] = _item("ok", f"{config.get('path')} {config['sha256'][:12]}")

    model = prov.get("model", {})
    out["model"] = (
        _item("ok", f"{model.get('urdf_path')} {model['urdf_sha256'][:12]}")
        if model.get("urdf_sha256") not in UNKNOWN
        else _item("missing", "URDF sha256 unknown")
    )

    data = prov.get("data", {})
    extra = record.get("inputs", {})
    bad = [
        k
        for k, v in data.items()
        if v.get("sha256") in UNKNOWN
        and not (
            k == "validation_data_file"
            and _check_validation_directory(v, (verdict or {}).get("splits"), cwd)
        )
    ]
    bad += _check_files(extra, cwd)
    if not data and not extra:
        out["inputs"] = _item("missing", "no input files recorded")
    elif bad:
        out["inputs"] = _item("incomplete", f"no matching sha256: {', '.join(bad)}")
    else:
        out["inputs"] = _item("ok", ", ".join([*data, *extra]))

    processing = record.get("processing")
    out["processing"] = (
        _item("ok", ", ".join(processing))
        if processing
        else _item("missing", f"no processing in {RECORD}")
    )

    splits = (verdict or {}).get("splits")
    out["splits"] = (
        _item("ok", f"validation source {splits.get('validation_source')}")
        if splits
        else _item("missing", "no splits in verdict.json")
    )

    fit = next((s for s in stages.get("stages", []) if s.get("stage") == "fit"), None)
    out["solver"] = (
        _item(
            "ok" if fit["status"] == "ok" else "incomplete",
            f"{fit['status']}: {fit.get('reason', '')}",
        )
        if fit
        else _item("missing", "no fit stage in stages.json")
    )

    selected = (verdict or {}).get("selected_stage")
    out["selected_stage"] = (
        _item("ok", selected) if selected else _item("missing", "no verdict.json")
    )

    out["report"] = (
        _item("ok", "report.html")
        if (run_dir / "report.html").exists()
        else _item("missing", "no report.html")
    )

    artifacts = record.get("artifacts", {})
    if not (run_dir / "parameters.csv").exists():
        out["export"] = _item("missing", "no parameters.csv")
    else:
        bad = _check_files(artifacts, cwd)
        out["export"] = (
            _item("incomplete", f"no matching sha256: {', '.join(bad)}")
            if bad
            else _item("ok", ", ".join(["parameters.csv", *artifacts]))
        )
    return out


def missing(checks: Dict[str, Dict[str, str]]) -> List[str]:
    return [item for item, c in checks.items() if c["status"] != "ok"]


def _write(run_dir: Path, record: Dict[str, Any]) -> Dict[str, Any]:
    path = run_dir / RECORD
    path.write_text(json.dumps(record, indent=2, default=str))
    checks = audit(run_dir)
    record["checklist"] = checks
    record["missing"] = missing(checks)
    path.write_text(json.dumps(record, indent=2, default=str))
    return record


def write_reproduction_record(
    run_dir,
    *,
    processing: Dict[str, Any],
    inputs: Optional[Dict[str, Dict[str, Any]]] = None,
    artifacts: Optional[Dict[str, Dict[str, Any]]] = None,
    argv: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Write ``reproduction.json`` and audit the run directory.

    Call after :func:`~figaroh.tools.run_archive.archive_run`.

    Args:
        run_dir: The archived run directory.
        processing: Settings applied outside the config (JSON-serializable).
        inputs: Input files not in provenance ``data``, as
            :func:`describe_file` entries keyed by name.
        artifacts: Generated files written outside the run directory, as
            :func:`describe_file` entries keyed by name.
        argv: Command line; defaults to ``sys.argv``.

    Returns:
        The record, with ``checklist`` and ``missing``.
    """
    run_dir = Path(run_dir)
    record = {
        "schema_version": SCHEMA_VERSION,
        "invocation": {
            "argv": list(argv if argv is not None else sys.argv),
            "cwd": str(Path.cwd()),
            "python": sys.executable,
        },
        "processing": processing,
        "inputs": inputs or {},
        "artifacts": artifacts or {},
    }
    record = _write(run_dir, record)
    print_audit(run_dir, record["checklist"])
    return record


def add_artifacts(run_dir, artifacts: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Attach generated files written after the record (e.g. an exported URDF)."""
    run_dir = Path(run_dir)
    record = _load_json(run_dir / RECORD)
    if record is None:
        raise FileNotFoundError(f"{run_dir / RECORD}: write the record first")
    record["artifacts"] = {**record.get("artifacts", {}), **artifacts}
    return _write(run_dir, record)


def print_audit(run_dir, checks: Dict[str, Dict[str, str]]) -> None:
    print(f"\nReproduction checklist: {run_dir}")
    for item, c in checks.items():
        print(f"  [{c['status'].upper():>10}] {item}: {c['detail']}")
    gaps = missing(checks)
    print(f"  Missing or incomplete: {', '.join(gaps) if gaps else 'none'}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit archived runs against the reproduction checklist."
    )
    parser.add_argument("run_dirs", nargs="+", type=Path)
    args = parser.parse_args(argv)
    code = 0
    for run_dir in args.run_dirs:
        checks = audit(run_dir)
        print_audit(run_dir, checks)
        code |= bool(missing(checks))
    return code


if __name__ == "__main__":
    sys.exit(main())
