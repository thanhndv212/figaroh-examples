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

"""Candidate postures for optimal calibration design (Phase 7).

For robots without a recorded candidate pool, the posture design draws its
candidates here: uniform samples over the calibrated chain, inside the URDF
joint limits less a margin in radians, every other joint held at ``q0``.

With ``collision="srdf"`` or ``collision="non_adjacent"``, self-colliding
samples are rejected and drawn again. Only the robot itself is checked: the
models carry no floor, table or fixture, so a selected posture can still be
unreachable on the real cell.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np
import pinocchio as pin

from figaroh.optimal.base_optimal_calibration import BaseOptimalCalibration
from figaroh.tools.robotcollisions import add_non_adjacent_pairs

COLLISION_MODES = ("none", "srdf", "non_adjacent")


@dataclass
class CandidatePool:
    """Sampled candidates, in the format ``BaseOptimalCalibration`` reads."""

    q: np.ndarray  # (n, nq) full configurations
    joint_names: List[str]  # sampled joints, in ``idx_q`` order
    idx_q: List[int]
    n_rejected: int  # samples dropped for self-collision

    def as_configs(self) -> dict:
        return {
            "calibration_joint_names": list(self.joint_names),
            "calibration_joint_configurations": self.q[:, self.idx_q].tolist(),
        }


def chain_joint_names(model, idx_q: Sequence[int]) -> List[str]:
    """Names of the one-DOF joints at the given ``idx_q``, in that order."""
    by_idx = {
        model.joints[j].idx_q: model.names[j]
        for j in range(1, model.njoints)
        if model.joints[j].nq == 1
    }
    missing = [i for i in idx_q if i not in by_idx]
    if missing:
        raise ValueError(f"idx_q {missing} are not one-DOF joints of the model")
    return [by_idx[i] for i in idx_q]


def _collision_checker(robot, mode: str, srdf: Optional[str]):
    if mode == "none":
        return None
    geom = robot.collision_model.copy()
    if mode == "srdf":
        if not srdf:
            raise ValueError('collision="srdf" needs an SRDF path')
        geom.addAllCollisionPairs()
        pin.removeCollisionPairs(robot.model, geom, srdf)
    else:
        add_non_adjacent_pairs(robot.model, geom)
    data, gdata = robot.model.createData(), geom.createData()

    def collides(q: np.ndarray) -> bool:
        return bool(pin.computeCollisions(robot.model, data, geom, gdata, q, True))

    return collides


def sample_chain_postures(
    robot,
    idx_q: Sequence[int],
    n: int,
    *,
    seed: int = 0,
    margin: float = 0.0,
    collision: str = "none",
    srdf: Optional[str] = None,
    max_draws: Optional[int] = None,
) -> CandidatePool:
    """Draw ``n`` postures of the chain ``idx_q``, seeded and reproducible.

    Args:
        margin: radians kept away from each limit
            (``tasks.optimal_configuration.constraints.joint_limit_margin``).
        collision: one of ``COLLISION_MODES``.
        max_draws: stop with an error after this many samples (default 20 n),
            so a model that collides almost everywhere fails loudly.
    """
    if collision not in COLLISION_MODES:
        raise ValueError(f"collision must be one of {COLLISION_MODES}")
    if margin < 0.0:
        raise ValueError("margin must be >= 0")
    model = robot.model
    idx_q = [int(i) for i in idx_q]
    names = chain_joint_names(model, idx_q)
    lo = model.lowerPositionLimit[idx_q].astype(float)
    hi = model.upperPositionLimit[idx_q].astype(float)
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and np.all(hi > lo)):
        raise ValueError(f"joints {names} need finite URDF limits with upper > lower")
    lo, hi = lo + margin, hi - margin
    if np.any(hi <= lo):
        raise ValueError(f"margin {margin} rad leaves no range on some of {names}")

    rng = np.random.default_rng(seed)
    collides = _collision_checker(robot, collision, srdf)
    max_draws = max_draws or 20 * n
    q0 = np.asarray(robot.q0, dtype=float)
    kept, draws = [], 0
    while len(kept) < n:
        if draws >= max_draws:
            raise RuntimeError(
                f"only {len(kept)}/{n} collision-free postures in {draws} draws"
            )
        q = q0.copy()
        q[idx_q] = rng.uniform(lo, hi)
        draws += 1
        if collides is None or not collides(q):
            kept.append(q)
    return CandidatePool(np.array(kept), names, idx_q, draws - n)


def configured_margin(config_file: str) -> float:
    """``tasks.optimal_configuration.constraints.joint_limit_margin`` (rad), 0 if unset."""
    from figaroh.utils.config_parser import parse_configuration

    tasks = parse_configuration(config_file).get("tasks", {})
    constraints = (tasks.get("optimal_configuration") or {}).get("constraints") or {}
    return float(constraints.get("joint_limit_margin") or 0.0)


class SampledOptimalCalibration(BaseOptimalCalibration):
    """Optimal posture design over sampled candidates.

    Subclasses set ``collision`` (and ``srdf`` for ``"srdf"``). The sampled
    chain is the calibration chain, ``calib_config["config_idx"]``.
    """

    collision = "none"
    srdf: Optional[str] = None

    def __init__(
        self, robot, config_file: str, n_candidates: int = 1000, seed: int = 0
    ):
        self.n_candidates = int(n_candidates)
        self.seed = int(seed)
        self.margin = configured_margin(config_file)
        self.candidate_pool: Optional[CandidatePool] = None
        super().__init__(robot, config_file)

    def load_candidate_configurations(self) -> None:
        pool = sample_chain_postures(
            self.robot,
            self.calib_config["config_idx"],
            self.n_candidates,
            seed=self.seed,
            margin=self.margin,
            collision=self.collision,
            srdf=self.srdf,
        )
        self.candidate_pool = pool
        self.q_measured = pool.q
        self._configs = pool.as_configs()
        self.calib_config["NbSample"] = len(pool.q)
