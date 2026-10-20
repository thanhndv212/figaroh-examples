"""Impact map and test selector for change-scoped testing (#141).

``select`` turns a list of changed paths into the robots, test files and
golden cases that a change can affect, following ``tests/impact.yaml``.
It is pure (no git, no subprocess) so it can be tested with fake diffs.
``tests/conftest.py`` reuses the same map for robot markers and the
collection guard.
"""

import re
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import yaml

IMPACT_FILE = Path(__file__).resolve().parent / "impact.yaml"


@dataclass
class Selection:
    robots: list = field(default_factory=list)
    test_files: list = field(default_factory=list)
    golden_ids: list = field(default_factory=list)
    full: bool = False
    reasons: list = field(default_factory=list)

    def to_dict(self):
        return {
            "robots": self.robots,
            "test_files": self.test_files,
            "golden_ids": self.golden_ids,
            "full": self.full,
            "reasons": self.reasons,
        }


def load_impact_map(path=IMPACT_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


@lru_cache(maxsize=None)
def _glob_regex(pattern):
    """Translate a glob (``**`` crosses ``/``) to a compiled regex."""
    out = []
    i = 0
    while i < len(pattern):
        if pattern.startswith("**/", i):
            out.append("(?:.*/)?")
            i += 3
        elif pattern.startswith("**", i):
            out.append(".*")
            i += 2
        elif pattern[i] == "*":
            out.append("[^/]*")
            i += 1
        elif pattern[i] == "?":
            out.append("[^/]")
            i += 1
        else:
            out.append(re.escape(pattern[i]))
            i += 1
    body = "".join(out)
    if "/" not in pattern:
        body = "(?:.*/)?" + body  # bare pattern: match the name anywhere
    return re.compile(body + r"\Z")


def matches(path, patterns):
    return any(_glob_regex(p).match(path) for p in patterns or [])


def robot_test_files(impact_map):
    """Map every test file the robots list to its robots."""
    owners = {}
    for robot, spec in impact_map["robots"].items():
        for test in spec.get("tests", []):
            owners.setdefault(test, []).append(robot)
    return owners


def shared_test_files(impact_map):
    owners = {}
    for name, spec in impact_map.get("shared", {}).items():
        for test in spec.get("tests", []):
            owners.setdefault(test, []).append(name)
    return owners


def mapped_test_files(impact_map):
    """Every test file the map covers (robot, shared, always, cross_robot)."""
    files = set(robot_test_files(impact_map)) | set(shared_test_files(impact_map))
    files |= set(impact_map.get("always", [])) | set(impact_map.get("cross_robot", []))
    return files


def select(changed_paths, impact_map=None):
    """Select what to run for ``changed_paths`` (repo-relative, ``/`` separated)."""
    m = impact_map if impact_map is not None else load_impact_map()
    sel = Selection()
    robots = set()
    tests = set(m.get("always", []))
    full_reasons = []
    reasons = []

    r_tests = robot_test_files(m)
    s_tests = shared_test_files(m)

    for path in sorted({p.replace("\\", "/") for p in changed_paths}):
        if matches(path, m.get("full_suite")):
            full_reasons.append(f"{path}: full-suite file")
            continue
        if matches(path, m.get("always_paths")):
            reasons.append(f"{path}: always-run checks only")
            continue
        if matches(path, m.get("ignore")):
            reasons.append(f"{path}: ignored (no tests)")
            continue
        hit = False
        for robot, spec in m["robots"].items():
            if matches(path, spec.get("paths")):
                robots.add(robot)
                reasons.append(f"{path}: robot {robot}")
                hit = True
        for name, spec in m.get("shared", {}).items():
            if matches(path, spec.get("paths")):
                robots.update(spec["robots"])
                tests.update(spec.get("tests", []))
                reasons.append(f"{path}: shared {name} -> {', '.join(spec['robots'])}")
                hit = True
        if hit:
            continue
        if path in r_tests or path in s_tests:
            tests.add(path)
            reasons.append(f"{path}: test file, rerun itself")
        elif path in m.get("always", []):
            reasons.append(f"{path}: always-run test")
        elif path in m.get("cross_robot", []):
            full_reasons.append(f"{path}: cross-robot test file")
        else:
            full_reasons.append(f"{path}: not in the impact map (safe default)")

    if full_reasons:
        sel.full = True
        sel.reasons = full_reasons + reasons
        sel.robots = list(m["robots"])
        return sel

    for robot in robots:
        tests.update(m["robots"][robot].get("tests", []))
    sel.robots = [r for r in m["robots"] if r in robots]
    sel.test_files = sorted(tests)
    sel.golden_ids = [
        gid for r in sel.robots for gid in m["robots"][r].get("golden_ids", [])
    ]
    if not changed_paths:
        reasons.append("no changed paths: always-run checks only")
    sel.reasons = reasons
    return sel


def script_keys(robots, impact_map=None):
    """validate.py EXAMPLE_SCRIPTS keys for ``robots`` (skips robots without one)."""
    m = impact_map if impact_map is not None else load_impact_map()
    return [
        m["robots"][r]["script_key"] for r in robots if m["robots"][r].get("script_key")
    ]


def golden_case_for(case_id, impact_map):
    """Robot owning a test_golden_outputs parametrize id, or None."""
    for robot, spec in impact_map["robots"].items():
        if any(g in case_id for g in spec.get("golden_ids", [])):
            return robot
    return None
