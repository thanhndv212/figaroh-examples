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

"""Scoped identification verification shared by the example CLIs.

``execution`` checks that the fit produced finite, consistent numerical
outputs. ``prediction`` additionally needs separately loaded validation data
and an explicit per-joint error-limit profile. Neither scope certifies
physical feasibility or export; those stages stay NOT_EVALUATED unless the
caller records them (e.g. ``tiago/reference_run.py`` records the export).
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

SCOPES = ("execution", "prediction")


@dataclass
class AcceptanceProfile:
    """Explicit metric limits read from a JSON file, kept verbatim for archiving."""

    path: Path
    text: str
    thresholds: Dict[str, Dict[str, Any]]


def load_acceptance_profile(path: str) -> AcceptanceProfile:
    """Read and shape-check a limits file (raises ValueError on bad input)."""
    profile_path = Path(path)
    try:
        text = profile_path.read_text()
        thresholds = json.loads(text)
    except (OSError, ValueError) as exc:
        raise ValueError(f"acceptance profile {profile_path}: {exc}") from exc
    if not isinstance(thresholds, dict) or not thresholds:
        raise ValueError(
            f"acceptance profile {profile_path}: expected a non-empty mapping "
            "of metric names to limits"
        )
    for name, spec in thresholds.items():
        if not isinstance(spec, dict) or not {"threshold", "comparison"} <= set(spec):
            raise ValueError(
                f"acceptance profile {profile_path}: {name!r} needs "
                "'threshold' and 'comparison' fields"
            )
    return AcceptanceProfile(profile_path, text, thresholds)


def _profile_arg(path: str) -> AcceptanceProfile:
    try:
        return load_acceptance_profile(path)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def add_verification_args(parser: argparse.ArgumentParser) -> None:
    """Add ``--verification-scope`` and ``--acceptance-profile``."""
    parser.add_argument(
        "--verification-scope",
        choices=SCOPES,
        default="execution",
        help=(
            "execution: finite, consistent fit outputs only. prediction: also "
            "requires separate validation data and --acceptance-profile limits."
        ),
    )
    parser.add_argument(
        "--acceptance-profile",
        type=_profile_arg,
        default=None,
        metavar="JSON",
        help=(
            "JSON mapping of explicit metric limits, e.g. "
            '{"validation_rmse:<joint>": {"threshold": 0.5, "comparison": "max"}}; '
            "copied into the run directory."
        ),
    )


def _format_value(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.4g}"


def run_verification(
    iden,
    run_dir: Path,
    scope: str,
    profile: Optional[AcceptanceProfile] = None,
):
    """Verify ``iden`` in ``scope``, write ``verdict.json`` and print the checks.

    Returns the core ``VerificationVerdict``; ``verdict.passed`` is True only
    when every required check in the scope passed.
    """
    thresholds = profile.thresholds if profile else None
    if profile:
        (run_dir / "acceptance-profile.json").write_text(profile.text)
    verdict = iden.verify(scope=scope, thresholds=thresholds)
    iden.export_verification_report(
        output_path=str(run_dir / "verdict.json"),
        scope=scope,
        thresholds=thresholds,
    )
    print_verdict(verdict)
    return verdict


def print_verdict(verdict) -> None:
    """Print each check, the scope status and the unevaluated stages."""
    for check in verdict.checks:
        line = (
            f"  [{check.status.upper()}] {check.name}: {_format_value(check.value)} "
            f"({check.comparison} {check.threshold:.4g})"
        )
        if check.status != "pass" and check.reason:
            line += f" — {check.reason}"
        if not check.required:
            line += " [advisory]"
        print(line)
    print(f"Verification scope: {verdict.scope}; status: {verdict.status.upper()}")
    print(
        "Prediction acceptance (declared validation split): "
        f"{verdict.stages['prediction'].upper()}"
    )
    stages = verdict.stages
    print(
        f"Physical model: {stages.get('physical', 'not_evaluated').upper()}; "
        f"export: {stages.get('export', 'not_evaluated').upper()}"
    )
