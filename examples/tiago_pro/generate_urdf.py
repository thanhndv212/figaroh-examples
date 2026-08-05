#!/usr/bin/env python3
"""
Post-process a URDF generated in the container to make mesh paths portable.

Replaces absolute container paths like:
  /home/gepetto/ros2_ws/install/<pkg>/share/<pkg>/...
  /opt/ros/humble/share/<pkg>/...
with:
  package://<pkg>/...

Usage:
    python3 generate_urdf.py --input urdf/tiago_pro_container.urdf --output urdf/tiago_pro.urdf
    python3 generate_urdf.py  # uses urdf/tiago_pro_container.urdf -> urdf/tiago_pro.urdf
"""

import argparse
import re
from pathlib import Path

_HERE = Path(__file__).parent
_INPUT = _HERE / "urdf" / "tiago_pro_container.urdf"
_OUTPUT = _HERE / "urdf" / "tiago_pro.urdf"

_EXT = r"(?:stl|dae|obj|STL|DAE|OBJ)"

# Workspace-installed packages: /home/gepetto/ros2_ws/install/<pkg>/share/<pkg>/...
_CONTAINER_INSTALL = re.compile(
    rf"/home/gepetto/ros2_ws/install/[^/]+/share/([^/]+)/(.*?\.{_EXT})"
)

# System APT packages: [file://]/opt/ros/<distro>/share/<pkg>/...
_SYSTEM_INSTALL = re.compile(
    rf"(?:file://)?/opt/ros/[^/]+/share/([^/]+)/(.*?\.{_EXT})"
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=str(_INPUT))
    parser.add_argument("--output", default=str(_OUTPUT))
    args = parser.parse_args()

    text = Path(args.input).read_text()

    def replace_path(m):
        return f"package://{m.group(1)}/{m.group(2)}"

    cleaned, n1 = _CONTAINER_INSTALL.subn(replace_path, text)
    cleaned, n2 = _SYSTEM_INSTALL.subn(replace_path, cleaned)
    print(f"Replaced {n1} workspace path(s) and {n2} system path(s).")

    Path(args.output).write_text(cleaned)
    print(f"Written to {args.output}")


if __name__ == "__main__":
    main()
