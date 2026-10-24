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

"""
TALOS optimal calibration postures (D-optimal design).

Selects, from seeded random candidates over the calibrated chain, the
postures that maximise the information of the kinematic calibration
(``TALOSOptimalCalibration``). The rest of the body is held at q0, and self-colliding candidates (TALOS SRDF) are drawn again.

This is a design: nothing here is measured. The selected postures still
have to be checked on the real cell before recording, since only
self-collision is modelled.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.talos.utils.talos_tools import TALOSOptimalCalibration  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="TALOS optimal calibration postures")
    parser.add_argument(
        "--config",
        type=str,
        default="config/talos_unified_config.yaml",
        help="Path to unified config YAML file",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/talos_full_v2.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--candidates", type=int, default=1000, help="Number of sampled candidates"
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="Seed of the candidate sampling"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose (INFO) logging"
    )
    return parser.parse_args(argv)


def main(args: argparse.Namespace) -> TALOSOptimalCalibration:
    """Sample candidates, select the D-optimal subset, save it."""
    for label, path in (("URDF", args.urdf), ("Config", args.config)):
        if not Path(path).exists():
            print(f"Error: {label} file not found: {path}", file=sys.stderr)
            sys.exit(1)

    print("TALOS Optimal Calibration Postures")
    print("=" * 50)
    robot = load_robot(args.urdf, package_dirs="../../models", load_by_urdf=True)
    opt = TALOSOptimalCalibration(
        robot, args.config, n_candidates=args.candidates, seed=args.seed
    )
    print(f"Calibration model: {opt.calib_config['calib_model']}")
    print(f"Minimum configurations required: {opt.minNbChosen}")

    start = time.time()
    opt.solve(save_file=True)
    pool = opt.candidate_pool
    print(f"Solved in {time.time() - start:.1f} s")
    print(f"Sampled joints: {', '.join(pool.joint_names)}")
    print(
        f"Candidates: {len(pool.q)} (seed {args.seed}, margin {opt.margin} rad, "
        f"{pool.n_rejected} self-colliding draws rejected)"
    )
    n_sel = opt.count_optimal_configurations()
    print(f"Selected {n_sel} postures")
    print(f"Information matrix determinant root: {opt.detroot_whole:.4e}")
    print(f"Results saved to '{opt.get_optimal_output_dir()}/'")
    return opt


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    main(args)
