"""Example scripts still produce their recorded outputs (#76).

Each case runs one example script unchanged and compares what it produces
with ``tests/golden/golden_outputs.json``. When a core or example change
moves an output on purpose, regenerate the reference in the same PR:

    python tests/golden/golden_outputs.py --update
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "golden"))

import golden_outputs as golden  # noqa: E402

REFERENCE = golden.load_reference()


@pytest.mark.parametrize("case", list(golden.CASES))
def test_example_outputs_match_reference(case):
    assert case in REFERENCE, f"no reference for {case}; run with --update"
    got = golden.summarize(case, golden.run_case(case))
    diffs = golden.compare(case, REFERENCE[case], got)
    assert not diffs, f"{case} changed:\n  " + "\n  ".join(diffs)
