# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2024] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils.tiago_tools import TiagoOptimalCalibration  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="TIAGo optimal configuration generation for calibration"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/tiago_unified_config_hey5.yaml",
        help="Path to config YAML file (unified format)",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/tiago_48_hey5.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--end-effector",
        "-e",
        type=str,
        default="hey5",
        dest="end_effector",
        help="End-effector type (hey5, schunk, etc.)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose (INFO) logging",
    )
    return parser.parse_args()


def main() -> None:
    """Run TIAGo optimal configuration generation."""
    args = parse_args()

    # Configure logging after parsing args
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Validate files exist
    config_path = Path(args.config)
    urdf_path = Path(args.urdf)
    if not config_path.exists():
        print(f"Error: Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    if not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # Load robot model
        tiago = load_robot(
            str(urdf_path),
            load_by_urdf=True,
            robot_pkg="tiago_description",
        )

        # Create optimal calibration object
        tiago_optcalib = TiagoOptimalCalibration(tiago, str(config_path))

        # Solve for optimal configurations
        print(
            f"Generating optimal configurations for TIAGo "
            f"with {args.end_effector} end effector..."
        )
        tiago_optcalib.solve(save_file=False)  # Skip writing to file for now
    except Exception as e:
        print(
            f"Error during optimal configuration generation: {e}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
