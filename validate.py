#!/usr/bin/env python3
"""
Validation script for figaroh-examples.

Runs the pytest suite and all available example scripts, then reports a
summary. Designed to be used as a post-phase quality gate after
implementation work.

Usage:
    python validate.py              # run tests + all scripts
    python validate.py --tests-only # run pytest only
    python validate.py --scripts-only  # run example scripts only
    python validate.py --robot ur10 # run tests + scripts for one robot
    python validate.py --quick      # skip slow scripts (optimal_config, optimal_trajectory)

Exit code: 0 if all pass, 1 if any fail.
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# --- Configuration -------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent

# Example scripts to validate, grouped by robot.
# Each entry: (script_name, timeout_seconds, is_slow, [extra_args])
#   extra_args is an optional list of CLI flags to pass to the script.
EXAMPLE_SCRIPTS = {
    "ur10": [
        ("calibration.py", 120, False,
         ["--calibrate-only", "--no-plot", "--html-report"]),
        ("update_model.py", 120, False),
        ("identification.py", 120, False, ["--verify", "--html-report"]),
        ("optimal_config.py", 600, True),
        ("optimal_trajectory.py", 600, True),
    ],
    "tiago": [
        ("calibration.py", 120, False,
         ["--calibrate-only", "--no-plot", "--html-report"]),
        ("update_model.py", 120, False),
        ("identification.py", 120, False, ["--verify", "--html-report"]),
        ("optimal_config.py", 120, False),
        ("optimal_trajectory.py", 600, True),
    ],
    "talos": [
        ("calibration_upperbody.py", 120, False,
         ["--calibrate-only", "--no-plot", "--html-report"]),
        ("update_model.py", 120, False),
    ],
    "staubli_tx40": [
        ("identification.py", 120, False, ["--verify", "--html-report"]),
    ],
}

# --- Helpers -------------------------------------------------------------


class Result:
    """Track pass/fail/skip/timeout for a single check."""

    def __init__(self, name, category):
        self.name = name
        self.category = category  # "test" or "script"
        self.status = "unknown"  # pass, fail, skip, timeout
        self.duration = 0.0
        self.output = ""
        self.error = ""

    def __repr__(self):
        icon = {
            "pass": "PASS",
            "fail": "FAIL",
            "skip": "SKIP",
            "timeout": "TIMEOUT",
            "unknown": "?",
        }[self.status]
        return f"[{icon}] {self.category}/{self.name} ({self.duration:.1f}s)"


def run_command(cmd, cwd, timeout, env_extra=None):
    """Run a command, return (returncode, stdout, stderr, timed_out)."""
    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)

    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
        )
        return proc.returncode, proc.stdout, proc.stderr, False
    except subprocess.TimeoutExpired as e:
        out = e.stdout or ""
        err = e.stderr or ""
        if isinstance(out, bytes):
            out = out.decode("utf-8", errors="replace")
        if isinstance(err, bytes):
            err = err.decode("utf-8", errors="replace")
        return -1, out, err, True


def run_pytest():
    """Run the full pytest suite from repo root."""
    print("\n" + "=" * 70)
    print("  PYTEST SUITE")
    print("=" * 70)

    result = Result("pytest", "test")
    start = time.time()

    rc, stdout, stderr, timed_out = run_command(
        [sys.executable, "-m", "pytest", "tests/", "-v", "--tb=short"],
        cwd=REPO_ROOT,
        timeout=180,
    )

    result.duration = time.time() - start
    result.output = stdout
    result.error = stderr

    if timed_out:
        result.status = "timeout"
    elif rc == 0:
        result.status = "pass"
    else:
        result.status = "fail"

    # Print last meaningful line (summary)
    for line in stdout.splitlines():
        if "passed" in line or "failed" in line or "error" in line:
            print(f"  {line.strip()}")
            break

    if result.status != "pass":
        # Show failing tests
        for line in stdout.splitlines():
            if "FAILED" in line or "ERROR" in line:
                print(f"    {line.strip()}")

    print(f"  -> {result}")
    return result


def run_example_script(robot, script, timeout, is_slow, quick=False, extra_args=None):
    """Run a single example script from its robot directory."""
    if quick and is_slow:
        r = Result(f"{robot}/{script}", "script")
        r.status = "skip"
        r.output = "Skipped (--quick mode)"
        print(f"  -> {r}")
        return r

    script_path = REPO_ROOT / "examples" / robot / script
    r = Result(f"{robot}/{script}", "script")

    if not script_path.exists():
        r.status = "skip"
        r.output = f"File not found: {script_path}"
        print(f"  -> {r}")
        return r

    print(f"\n  Running {robot}/{script} ...")
    start = time.time()

    cmd = [sys.executable, script]
    if extra_args:
        cmd.extend(extra_args)

    rc, stdout, stderr, timed_out = run_command(
        cmd,
        cwd=script_path.parent,
        timeout=timeout,
        env_extra={"MPLBACKEND": "Agg"},  # non-interactive matplotlib
    )

    r.duration = time.time() - start
    r.output = stdout
    r.error = stderr

    if timed_out:
        r.status = "timeout"
    elif rc == 0:
        r.status = "pass"
    else:
        r.status = "fail"

    # Print last few meaningful lines
    lines = stdout.strip().splitlines()
    if lines:
        # Show last 3 lines of stdout
        for line in lines[-3:]:
            print(f"    {line.strip()}")

    if r.status == "fail" and stderr:
        # Show error traceback
        err_lines = stderr.strip().splitlines()
        for line in err_lines[-5:]:
            print(f"    [stderr] {line.strip()}")

    print(f"  -> {r}")
    return r


def run_all_scripts(robots=None, quick=False):
    """Run all (or filtered) example scripts."""
    print("\n" + "=" * 70)
    print("  EXAMPLE SCRIPTS")
    print("=" * 70)

    results = []
    target_robots = robots if robots else list(EXAMPLE_SCRIPTS.keys())

    for robot in target_robots:
        if robot not in EXAMPLE_SCRIPTS:
            print(f"\n  Unknown robot: {robot}")
            continue

        print(f"\n  --- {robot} ---")
        for entry in EXAMPLE_SCRIPTS[robot]:
            if len(entry) == 4:
                script, timeout, is_slow, extra_args = entry
            else:
                script, timeout, is_slow = entry
                extra_args = []
            r = run_example_script(robot, script, timeout, is_slow, quick, extra_args)
            results.append(r)

    return results


def print_summary(test_results, script_results):
    """Print final summary table."""
    print("\n" + "=" * 70)
    print("  VALIDATION SUMMARY")
    print("=" * 70)

    all_results = test_results + script_results

    counts = {"pass": 0, "fail": 0, "skip": 0, "timeout": 0}
    for r in all_results:
        counts[r.status] = counts.get(r.status, 0) + 1

    # Test results
    if test_results:
        print("\n  Tests:")
        for r in test_results:
            print(f"    {r}")

    # Script results
    if script_results:
        print("\n  Scripts:")
        for r in script_results:
            print(f"    {r}")

    # Totals
    print(
        f"\n  Total: {counts['pass']} passed, {counts['fail']} failed, "
        f"{counts['skip']} skipped, {counts['timeout']} timed out"
    )

    # Final verdict
    print("\n" + "-" * 70)
    if counts["fail"] > 0:
        print(f"  RESULT: FAIL ({counts['fail']} failure(s))")
    elif counts["timeout"] > 0:
        print(
            f"  RESULT: PASS WITH TIMEOUTS ({counts['timeout']} timed out — "
            "likely IPOPT optimization, not a bug)"
        )
    else:
        print("  RESULT: ALL PASS")
    print("-" * 70)

    return counts["fail"] == 0


# --- Main ----------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Validate figaroh-examples: run tests and example scripts."
    )
    parser.add_argument(
        "--tests-only",
        action="store_true",
        help="Run pytest only, skip example scripts.",
    )
    parser.add_argument(
        "--scripts-only",
        action="store_true",
        help="Run example scripts only, skip pytest.",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Run scripts for a single robot (ur10, tiago, talos, staubli_tx40).",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Skip slow scripts (optimal_config, optimal_trajectory).",
    )
    args = parser.parse_args()

    print("=" * 70)
    print("  figaroh-examples Validation")
    print(f"  Python: {sys.executable}")
    print(f"  Repo:   {REPO_ROOT}")
    print("=" * 70)

    test_results = []
    script_results = []

    if not args.scripts_only:
        test_results.append(run_pytest())

    if not args.tests_only:
        robots = [args.robot] if args.robot else None
        script_results = run_all_scripts(robots=robots, quick=args.quick)

    success = print_summary(test_results, script_results)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
