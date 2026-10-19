"""The optional ``heldout:`` block of a protocol.yaml.

Core's ``figaroh.data.protocol.Protocol`` reads ``name``, ``version`` and
``sessions`` and ignores every other key, so this block rides in the same
file without changing the data contract::

    heldout:
      split: sessions            # or kfold; default sessions
      selection_role: validation # role model selection may look at
      kfold:                     # required iff split is kfold
        session: 2021-11-30-1544
        k: 5
        seed: 0
      strata:                    # optional, posture strata of held-out sets
        joints: [torso_lift_joint, arm_1_joint]
        repeated_tol: 0.01
        out_of_range_margin: 0.05
      not_evaluated: [2021-10-26-1051]  # optional: manifest only (hashes, roles),
                                        # not in the plan (other frame)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import yaml
from figaroh.data.protocol import Protocol

ROLES = ("training", "validation", "confirmation")
SPLITS = ("sessions", "kfold")
_BLOCK_KEYS = {"split", "selection_role", "kfold", "strata", "not_evaluated"}
_KFOLD_KEYS = {"session", "k", "seed"}
_STRATA_KEYS = {"joints", "repeated_tol", "out_of_range_margin"}


@dataclass(frozen=True)
class KFoldSpec:
    session: str
    k: int
    seed: int = 0


@dataclass(frozen=True)
class StrataSpec:
    joints: Tuple[str, ...]
    repeated_tol: float = 0.01
    out_of_range_margin: float = 0.05


@dataclass(frozen=True)
class HeldoutSpec:
    split: str = "sessions"
    selection_role: str = "validation"
    kfold: Optional[KFoldSpec] = None
    strata: Optional[StrataSpec] = None
    not_evaluated: Tuple[str, ...] = ()


def _unknown(where: str, got: dict, allowed: set) -> None:
    extra = sorted(set(got) - allowed)
    if extra:
        raise ValueError(f"{where}: unknown keys {extra}; allowed {sorted(allowed)}")


def parse_heldout(raw: Optional[dict], protocol: Protocol) -> HeldoutSpec:
    """Validate a ``heldout:`` mapping against ``protocol``.

    A missing block (``None``) means the sessions split, so protocols that
    predate the block keep working.
    """
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("heldout: must be a mapping")
    _unknown("heldout", raw, _BLOCK_KEYS)
    for s in protocol.sessions:
        if s.role not in ROLES:
            raise ValueError(f"session {s.id}: role {s.role!r} not in {ROLES}")

    split = raw.get("split", "sessions")
    if split not in SPLITS:
        raise ValueError(f"heldout.split {split!r} not in {SPLITS}")
    role = raw.get("selection_role", "validation")
    if role not in ROLES:
        raise ValueError(f"heldout.selection_role {role!r} not in {ROLES}")
    if role == "confirmation":
        raise ValueError("heldout.selection_role cannot be 'confirmation'")

    kfold = None
    if raw.get("kfold") is not None:
        if split != "kfold":
            raise ValueError("heldout.kfold is only valid with split: kfold")
        k = raw["kfold"]
        if not isinstance(k, dict):
            raise ValueError("heldout.kfold must be a mapping")
        _unknown("heldout.kfold", k, _KFOLD_KEYS)
        if "session" not in k or "k" not in k:
            raise ValueError("heldout.kfold needs session and k")
        kfold = KFoldSpec(str(k["session"]), int(k["k"]), int(k.get("seed", 0)))
        if kfold.k < 2:
            raise ValueError("heldout.kfold.k must be >= 2")
        protocol.session(kfold.session)  # KeyError if unknown
    if split == "kfold":
        if kfold is None:
            raise ValueError("split: kfold needs a heldout.kfold block")
        if len(protocol.sessions) != 1:
            raise ValueError(
                "split: kfold needs exactly one session, "
                f"got {len(protocol.sessions)}"
            )
    else:
        n = len(protocol.with_role("training"))
        if n != 1:
            raise ValueError(
                f"split: sessions needs exactly one training session, got {n}"
            )

    strata = None
    if raw.get("strata") is not None:
        s = raw["strata"]
        if not isinstance(s, dict):
            raise ValueError("heldout.strata must be a mapping")
        _unknown("heldout.strata", s, _STRATA_KEYS)
        if not s.get("joints"):
            raise ValueError("heldout.strata needs joints")
        strata = StrataSpec(
            tuple(str(j) for j in s["joints"]),
            float(s.get("repeated_tol", 0.01)),
            float(s.get("out_of_range_margin", 0.05)),
        )
    skipped = raw.get("not_evaluated") or []
    if not isinstance(skipped, list):
        raise ValueError("heldout.not_evaluated must be a list of session ids")
    skipped = tuple(str(i) for i in skipped)
    for sid in skipped:
        protocol.session(sid)  # KeyError if unknown
    if split == "kfold" and skipped:
        raise ValueError("heldout.not_evaluated is only valid with split: sessions")
    return HeldoutSpec(split, role, kfold, strata, skipped)


def read_heldout(path) -> Tuple[Protocol, HeldoutSpec]:
    """Load a protocol.yaml and its validated ``heldout:`` block."""
    path = Path(path)
    protocol = Protocol.load(path)
    raw = yaml.safe_load(path.read_text()).get("heldout")
    return protocol, parse_heldout(raw, protocol)
