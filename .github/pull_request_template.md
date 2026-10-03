<!-- Base: `main`. Title: Conventional Commits, e.g. "fix(ur10): align configured timing".
     PRs are merged with a merge commit after maintainer approval. -->

Closes #
<!-- Paired core change? Link thanhndv212/figaroh-plus#… and give the core commit tested with. -->

## Why

<!-- The problem, with evidence (numbers, logs, a failing check). -->

## What

<!-- The change, by robot/module. Call out data, config or default changes explicitly; original data stays unmodified. -->

## Validation

<!-- Paste the commands and results. For a level you skipped, say which and why. -->

- [ ] Focused tests for the changed example(s)
- [ ] Full `python validate.py` (required for implementation changes): pass/fail/skip/timeout counts,
      each known failure classified against the baseline, `validation_logs/` cited for failures
- [ ] Tested revision pair: figaroh-examples `…` + figaroh-plus `…`, `figaroh-dev` environment
- [ ] Numerical changes: before/after metrics on the same data; status changes from policy labelled as such
- [ ] CI green on the latest commit

## Docs

- [ ] Robot README / data notes / top-level README updated, or not needed

## Found along the way / limits

<!-- Issues noticed but not fixed here (open issues for them), and what this does not establish
     (e.g. simulation vs. hardware, training vs. held-out evidence). -->

- [ ] Maintainer approval received before merging
