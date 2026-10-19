"""Held-out evaluation plans from a frozen protocol (#96).

``load_plan`` turns a protocol.yaml (sha256-verified) into folds: either the
protocol's own sessions (train on the training session, evaluate on the
rest) or k row folds within one session. ``run`` fits and evaluates every
model on every fold; ``select`` picks a model from a role that is allowed to
influence the choice, never from ``confirmation``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Tuple

import numpy as np
import pandas as pd

from examples.calibration_harness.protocol import (
    HeldoutSpec,
    read_heldout,
)

KFOLD_NOTE = "weaker than held-out sessions"


@dataclass(frozen=True)
class HeldoutSet:
    """A file (or a subset of its rows) playing one role."""

    name: str
    role: str
    session_id: str
    path: Path
    rows: Tuple[int, ...]  # row indices into the file


@dataclass(frozen=True)
class Fold:
    training: Tuple[HeldoutSet, ...]
    evaluated: Tuple[HeldoutSet, ...]


@dataclass(frozen=True)
class HeldoutPlan:
    protocol: str  # "<name> v<version>"
    split: str  # sessions | kfold
    label: str
    folds: Tuple[Fold, ...]
    spec: HeldoutSpec


class HeldoutResults(dict):
    """``{model: {fold: {set: metrics}}}`` plus the plan's label and roles.

    Each metrics dict carries the set's ``role``; ``label`` states how
    strong the evidence is (printed and written with the results).
    """

    label: str = ""

    def as_json(self) -> dict:
        return {"label": self.label, "results": dict(self)}


def _n_rows(path: Path) -> int:
    return len(pd.read_csv(path))


def load_plan(path, verify: bool = True) -> HeldoutPlan:
    """Build a plan from a protocol.yaml; fold indices are fixed here.

    ``verify`` checks every session file against its sha256 (paths relative
    to the yaml's directory) and raises ``ValueError`` on a mismatch.
    """
    path = Path(path)
    protocol, spec = read_heldout(path)
    root = path.parent
    if verify:
        protocol.verify(root=root)
    label_proto = f"{protocol.name} v{protocol.version}"

    if spec.split == "sessions":
        sets = [
            HeldoutSet(
                Path(rel).name,
                s.role,
                s.id,
                root / rel,
                tuple(range(_n_rows(root / rel))),
            )
            for s in protocol.sessions
            if s.id not in spec.not_evaluated
            for rel in s.files
        ]
        train = tuple(x for x in sets if x.role == "training")
        rest = tuple(x for x in sets if x.role != "training")
        sid = train[0].session_id
        label = f"held-out sessions (training session {sid})"
        return HeldoutPlan(label_proto, "sessions", label, (Fold(train, rest),), spec)

    session = protocol.session(spec.kfold.session)
    if len(session.files) != 1:
        raise ValueError(f"k-fold session {session.id} must have exactly one file")
    (rel,) = session.files
    n = _n_rows(root / rel)
    k = spec.kfold.k
    if k > n:
        raise ValueError(f"k={k} exceeds the {n} rows of session {session.id}")
    perm = np.random.default_rng(spec.kfold.seed).permutation(n)
    parts = [tuple(sorted(int(i) for i in p)) for p in np.array_split(perm, k)]
    name = Path(rel).name
    folds = []
    for i, held in enumerate(parts):
        rest = tuple(sorted(j for m, p in enumerate(parts) if m != i for j in p))
        folds.append(
            Fold(
                (
                    HeldoutSet(
                        f"{name}[train {i}]", "training", session.id, root / rel, rest
                    ),
                ),
                (
                    HeldoutSet(
                        f"{name}[fold {i}]", "validation", session.id, root / rel, held
                    ),
                ),
            )
        )
    label = f"k-fold within session {session.id} (single session; {KFOLD_NOTE})"
    return HeldoutPlan(label_proto, "kfold", label, tuple(folds), spec)


def run(
    plan: HeldoutPlan,
    models: Mapping[str, Callable[[Fold], Any]],
    evaluate: Callable[[Any, HeldoutSet], Mapping[str, Any]],
) -> HeldoutResults:
    """Fit each model on each fold's training sets and evaluate it.

    ``models[name](fold)`` returns a fitted object; ``evaluate(fitted, set)``
    returns a metrics mapping. Nothing here looks at the results.
    """
    out = HeldoutResults()
    out.label = plan.label
    for name, fit in models.items():
        out[name] = {}
        for i, fold in enumerate(plan.folds):
            fitted = fit(fold)
            out[name][i] = {
                s.name: {**evaluate(fitted, s), "role": s.role} for s in fold.evaluated
            }
    return out


def select(
    results: Mapping, metric: str, role: str = "validation"
) -> Tuple[str, Dict[str, float]]:
    """Best model (lowest mean ``metric``) over the sets of ``role``.

    Returns ``(best, {model: score})``. Selecting on ``confirmation`` is the
    leak the protocol exists to prevent, so it raises ``ValueError``.
    """
    if role == "confirmation":
        raise ValueError("model selection must not look at confirmation sets")
    scores = {}
    for model, folds in results.items():
        vals = [
            m[metric]
            for sets in folds.values()
            for m in sets.values()
            if m.get("role") == role
        ]
        if not vals:
            raise ValueError(f"no {role!r} sets with metric {metric!r} for {model}")
        scores[model] = float(np.mean(vals))
    return min(scores, key=scores.get), scores


def posture_strata(
    q: np.ndarray, q_train: np.ndarray, tol: float = 0.01, margin: float = 0.05
) -> dict:
    """Split postures by their relation to the training postures.

    - ``repeated``: within ``tol`` (rad or m) of a training posture on every
      joint, i.e. the same configuration recorded on another occasion;
    - ``out_of_range``: some joint outside the training range by > ``margin``;
    - ``new``: everything else (new configurations inside the training range).
    """
    nearest = np.array([np.min(np.max(np.abs(q_train - x), axis=1)) for x in q])
    repeated = nearest < tol
    outside = np.any(
        (q < q_train.min(0) - margin) | (q > q_train.max(0) + margin), axis=1
    )
    return {
        "repeated": repeated,
        "new": ~repeated & ~outside,
        "out_of_range": outside & ~repeated,
    }
