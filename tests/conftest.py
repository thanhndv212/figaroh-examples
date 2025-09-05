"""
Pytest configuration file for the unified config parser tests.
"""

import sys
from pathlib import Path

# Add src directory to Python path for figaroh imports
project_root = Path(__file__).parent.parent
figaroh_src = project_root / "figaroh" / "src"
examples_root = project_root / "figaroh-examples"

if str(figaroh_src) not in sys.path:
    sys.path.insert(0, str(figaroh_src))

if str(examples_root) not in sys.path:
    sys.path.insert(0, str(examples_root))

# Configure pytest
def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", 
        "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers",
        "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers",
        "real_config: marks tests that use real config files"
    )
