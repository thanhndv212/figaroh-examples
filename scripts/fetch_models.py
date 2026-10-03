#!/usr/bin/env python3
"""Fetch the third-party mesh packages the example URDFs need, at pinned revisions.

Most geometry is vendored under ``models/``. The packages below are not: they
are fetched from their upstream repositories at an exact commit and never
copied into this repository (see ``models/README.md`` for licences). After
fetching, point ``ROS_PACKAGE_PATH`` at them so Pinocchio resolves
``package://`` URIs:

    python scripts/fetch_models.py
    export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py --print-ros-package-path)"

Only the standard library and ``git`` are required, so CI can run this before
any environment is set up.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DEST = REPO_ROOT / "models" / "external"

# (checkout dir, GitHub repository, pinned commit, sparse path,
#  ROS package dir relative to the checkout, licence, needed by)
SOURCES = [
    (
        "agimus-demos",
        "agimus/agimus-demos",
        "7858fd3da9b31b4a17cc265722a4647567cda8cd",
        "meshes",
        "..",  # the checkout itself is the `agimus-demos` package
        "BSD-2-Clause",
        "ur10 (RealSense D435 mount)",
    ),
    (
        "tiago_robot",
        "pal-robotics/tiago_robot",
        "234b653b2a063e89bcdf7c9b2c419272531eab5a",
        "tiago_description/meshes",
        ".",
        "Apache-2.0",
        "tiago (arm_5 wrist-2010 meshes)",
    ),
    (
        "pmb2_robot",
        "pal-robotics/pmb2_robot",
        "90b419d64a786cbcf3ba22623b68259f354698aa",
        "pmb2_description/meshes",
        ".",
        "Apache-2.0 (package.xml; no LICENSE file)",
        "tiago (PMB2 caster wheels)",
    ),
    (
        "pal_wsg_gripper",
        "pal-robotics/pal_wsg_gripper",
        "ee542262a264f220142c787c0e005b5664bffa66",
        "pal_wsg_gripper_description/meshes",
        ".",
        "Proprietary (package.xml) - fetch only, do not redistribute",
        "tiago_48_schunk (WSG gripper)",
    ),
]


def _git(*args: str, cwd: Path) -> None:
    subprocess.run(["git", *args], cwd=cwd, check=True)


def _current_commit(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    out = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=path, capture_output=True, text=True
    )
    return out.stdout.strip() if out.returncode == 0 else None


def fetch(dest: Path) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    for name, repo, commit, sparse, _, _, _ in SOURCES:
        path = dest / name
        if _current_commit(path) == commit:
            print(f"{name}: already at {commit[:12]}")
            continue
        print(f"{name}: fetching {repo}@{commit[:12]} ({sparse})")
        path.mkdir(parents=True, exist_ok=True)
        if not (path / ".git").exists():
            _git("init", "-q", cwd=path)
            _git("remote", "add", "origin", f"https://github.com/{repo}.git", cwd=path)
        _git("sparse-checkout", "set", "--no-cone", sparse, cwd=path)
        _git("fetch", "-q", "--depth", "1", "origin", commit, cwd=path)
        _git("-c", "advice.detachedHead=false", "checkout", "-q", commit, cwd=path)


def ros_package_path(dest: Path) -> str:
    dirs = []
    for name, _, _, _, package_parent, _, _ in SOURCES:
        d = (dest / name / package_parent).resolve()
        if str(d) not in dirs:
            dirs.append(str(d))
    return ":".join(dirs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--dest",
        type=Path,
        default=DEFAULT_DEST,
        help=f"Checkout directory (default: {DEFAULT_DEST.relative_to(REPO_ROOT)})",
    )
    parser.add_argument(
        "--print-ros-package-path",
        action="store_true",
        help="Print the ROS_PACKAGE_PATH entries for --dest and exit",
    )
    args = parser.parse_args()
    dest = args.dest.resolve()
    if args.print_ros_package_path:
        print(ros_package_path(dest))
        return 0
    fetch(dest)
    print("\nexport ROS_PACKAGE_PATH=" + ros_package_path(dest))
    return 0


if __name__ == "__main__":
    sys.exit(main())
