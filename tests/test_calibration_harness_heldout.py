"""Held-out plans of the calibration harness (#96): schema, sha256, folds."""

import hashlib
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples.calibration_harness.heldout import (  # noqa: E402
    load_plan,
    posture_strata,
    run,
    select,
)


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(tmp_path, heldout=None, sessions=("train", "val", "conf")):
    roles = {"train": "training", "val": "validation", "conf": "confirmation"}
    entries = []
    for i, sid in enumerate(sessions):
        f = tmp_path / f"{sid}.csv"
        pd.DataFrame({"q": np.arange(10 + i, dtype=float)}).to_csv(f, index=False)
        entries.append({"id": sid, "role": roles[sid], "files": {f.name: _sha(f)}})
    doc = {"name": "toy", "version": 1, "sessions": entries}
    if heldout is not None:
        doc["heldout"] = heldout
    p = tmp_path / "protocol.yaml"
    p.write_text(yaml.safe_dump(doc))
    return p


KFOLD = {"split": "kfold", "kfold": {"session": "train", "k": 5, "seed": 3}}


def test_missing_block_is_sessions_split(tmp_path):
    plan = load_plan(_write(tmp_path))
    assert plan.split == "sessions" and len(plan.folds) == 1
    fold = plan.folds[0]
    assert [s.role for s in fold.training] == ["training"]
    assert [s.name for s in fold.evaluated] == ["val.csv", "conf.csv"]
    assert "held-out sessions" in plan.label


@pytest.mark.parametrize(
    "block",
    [
        {"split": "bogus"},
        {"selection_role": "nonsense"},
        {"selection_role": "confirmation"},
        {"unknown": 1},
        {"split": "kfold"},  # no kfold block
        {"kfold": {"session": "train", "k": 5}},  # kfold without split: kfold
    ],
)
def test_schema_errors(tmp_path, block):
    with pytest.raises(ValueError):
        load_plan(_write(tmp_path, block))


def test_kfold_needs_exactly_one_session(tmp_path):
    with pytest.raises(ValueError, match="exactly one session"):
        load_plan(_write(tmp_path, KFOLD))
    with pytest.raises(ValueError, match="k must|k>=|>= 2"):
        bad = {"split": "kfold", "kfold": {"session": "train", "k": 1}}
        load_plan(_write(tmp_path, bad, sessions=("train",)))


def test_sessions_needs_one_training(tmp_path):
    with pytest.raises(ValueError, match="one training"):
        load_plan(_write(tmp_path, sessions=("val", "conf")))


def test_sha256_mismatch_raises(tmp_path):
    p = _write(tmp_path)
    (tmp_path / "val.csv").write_text("q\n99\n")
    with pytest.raises(ValueError, match="sha256"):
        load_plan(p)
    assert load_plan(p, verify=False).split == "sessions"


def test_kfold_disjoint_complete_deterministic(tmp_path):
    p = _write(tmp_path, KFOLD, sessions=("train",))
    plan = load_plan(p)
    again = load_plan(p)
    assert plan.folds == again.folds
    assert len(plan.folds) == 5
    held = [r for f in plan.folds for r in f.evaluated[0].rows]
    assert sorted(held) == list(range(10))
    for f in plan.folds:
        ev, tr = set(f.evaluated[0].rows), set(f.training[0].rows)
        assert not ev & tr and ev | tr == set(range(10))
    other = dict(KFOLD, kfold={"session": "train", "k": 5, "seed": 4})
    assert load_plan(_write(tmp_path, other, sessions=("train",))).folds != plan.folds


def test_kfold_label_in_plan_and_results(tmp_path):
    plan = load_plan(_write(tmp_path, KFOLD, sessions=("train",)))
    assert "k-fold within session train" in plan.label
    assert "weaker than held-out sessions" in plan.label
    res = run(plan, {"m": lambda fold: None}, lambda fitted, s: {"rmse": 1.0})
    assert res.label == plan.label and res.as_json()["label"] == plan.label


def test_select_by_role_and_confirmation_raises(tmp_path):
    plan = load_plan(_write(tmp_path))
    scores = {
        "good": {"val.csv": 1.0, "conf.csv": 9.0},
        "bad": {"val.csv": 2.0, "conf.csv": 0.1},
    }
    res = run(
        plan,
        {m: (lambda fold, m=m: m) for m in scores},
        lambda fitted, s: {"rmse": scores[fitted][s.name]},
    )
    best, table = select(res, "rmse")
    assert best == "good" and table == {"good": 1.0, "bad": 2.0}
    with pytest.raises(ValueError, match="confirmation"):
        select(res, "rmse", role="confirmation")


def test_posture_strata():
    q_train = np.array([[0.0], [1.0]])
    q = np.array([[0.005], [0.5], [1.2]])
    s = posture_strata(q, q_train)
    assert s["repeated"].tolist() == [True, False, False]
    assert s["new"].tolist() == [False, True, False]
    assert s["out_of_range"].tolist() == [False, False, True]
