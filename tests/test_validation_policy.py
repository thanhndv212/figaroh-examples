"""Required timeout and diagnostic retention regressions."""

import json
import sys

import validate


def test_required_timeout_is_not_a_success(capsys):
    check = validate.Result("optimizer", "script")
    check.status = "timeout"
    assert not validate.print_summary([], [check])
    assert "INCOMPLETE" in capsys.readouterr().out


def test_complete_success_is_success():
    check = validate.Result("execution", "script")
    check.status = "pass"
    assert validate.print_summary([], [check])


def test_complete_child_output_and_exit_code_are_retained(tmp_path, monkeypatch):
    monkeypatch.setattr(validate, "REPO_ROOT", tmp_path)
    rc, out, err, timed_out = validate.run_command(
        [
            sys.executable,
            "-c",
            "import sys; print('full stdout'); print('full stderr', file=sys.stderr); sys.exit(7)",
        ],
        tmp_path,
        5,
        label="ur10/child",
    )
    assert rc == 7 and not timed_out
    directory = tmp_path / "validation_logs"
    meta = json.loads(next(directory.glob("ur10-child-*.json")).read_text())
    assert meta["returncode"] == 7 and not meta["timed_out"]
    assert next(directory.glob("*.stdout.log")).read_text() == out
    assert next(directory.glob("*.stderr.log")).read_text() == err


def test_timeout_preserves_partial_output_without_inventing_exit(tmp_path, monkeypatch):
    monkeypatch.setattr(validate, "REPO_ROOT", tmp_path)
    rc, out, _, timed_out = validate.run_command(
        [
            sys.executable,
            "-c",
            "import time; print('started', flush=True); time.sleep(2)",
        ],
        tmp_path,
        0.2,
        label="slow",
    )
    assert timed_out and "started" in out
    meta = json.loads(next((tmp_path / "validation_logs").glob("*.json")).read_text())
    assert meta["returncode"] is None and meta["timed_out"]


def test_empty_validation_cannot_pass():
    assert not validate.print_summary([], [])


def test_unknown_required_result_cannot_pass():
    check = validate.Result("missing-evidence", "script")
    assert not validate.print_summary([], [check])


def test_skipped_check_is_reported_not_established(capsys):
    done = validate.Result("execution", "script")
    done.status = "pass"
    skipped = validate.Result("optimizer", "script")
    skipped.status = "skip"
    assert validate.print_summary([], [done, skipped])
    assert "1 skipped check(s) not established" in capsys.readouterr().out


def test_acceptance_profile_is_checked_before_the_fit(tmp_path):
    import argparse

    import pytest

    from examples.verification import add_verification_args

    parser = argparse.ArgumentParser()
    add_verification_args(parser)
    good = tmp_path / "limits.json"
    good.write_text('{"validation_rmse:j1": {"threshold": 0.5, "comparison": "max"}}')
    args = parser.parse_args(["--acceptance-profile", str(good)])
    assert args.verification_scope == "execution"
    assert args.acceptance_profile.thresholds["validation_rmse:j1"]["threshold"] == 0.5

    for bad in ("[]", "{}", '{"validation_rmse:j1": 0.5}', "not json"):
        good.write_text(bad)
        with pytest.raises(SystemExit):
            parser.parse_args(["--acceptance-profile", str(good)])
