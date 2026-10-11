# Acceptance policy

What an identification `--verify` PASS means in these examples, and how to
request prediction acceptance. Summarised in the [README](../README.md#verification).

Identification `--verify` runs with `--verification-scope execution` by default:
it checks that the fit produced finite, consistent numerical outputs. CLI output
and `verdict.json` name the scope and per-stage status. An execution PASS is
**not** independent prediction acceptance, physical feasibility or export
approval; those stages print NOT_EVALUATED. Improvement over nominal, correlation
and raw condition number remain reported diagnostics, without the former
universal 50% / 0.9 / 1000 gates. Existing CSVs and historical verdicts are
unchanged; a status that changed under this policy is a policy change, not an
improvement in the fit.

Prediction acceptance needs separately loaded validation data **and** a JSON
profile with an explicit limit for every active joint, chosen from measurement
uncertainty and the application before you look at the result:

```json
{
  "validation_rmse:<joint>": {"threshold": 0.05, "comparison": "max", "rationale": "..."}
}
```

```bash
cd examples/so101   # ships a separate simulated validation run
python identification.py --verification-scope prediction --acceptance-profile limits.json
```

Thresholds are in the joint's effort unit (N·m for revolute joints). Optional
`required: false` marks a check advisory. The profile is copied into the run
directory. Missing limits or missing independent data give NOT_EVALUATED and a
nonzero exit; no example-specific limits are shipped or guessed. A separate file
alone does not prove an experimentally independent split or adequate coverage.
`so101/update_model.py` takes the same flags and records the scoped verdict in
the written YAML; its legacy `verification_passed` field is true only for a
passed prediction stage.
