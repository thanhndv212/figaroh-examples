# Dataset validation audit — 2026-10-02

Related: examples issue #12 / PR #13, core #22 / PR #31.
This audit reports empirical failures; it does not approve a production solver.

## Local runs

All six documented dataset commands (three robots × Pinocchio 3.7/4.1)
complete and pass native regressor/order/RNEA assertions. Machine-readable
records are committed under `results/`; interpretation is in `README.md`.
Both profiles use Python 3.12.11, NumPy 2.3.2, SciPy 1.16.1,
PICOS 2.6.1 and CVXOPT 1.3.2. All nonlinear fits exhaust 200 evaluations.
The TX40 alignment regression passes separately on each profile. Changed-file
hooks pass. Source datasets are unchanged.

The initial examples CI exposed hidden geometry dependencies on the local
`ROS_PACKAGE_PATH=/Users/thanhndv212/Develop/:/Users/thanhndv212/Develop/robot_models/`.
UR10 needs Agimus mount/tip meshes; TIAGo needs older wrist, WSG gripper and
PMB2 base/wheel meshes absent from this checkout. CI now fetches the precise
upstream repositories/revisions listed in `dataset-validation.yml`, supplies
its own ROS package search path, and pins the scientific versions above.
Assets are fetched into the runner, not copied into this repository.

## Full repository validation

Executed `MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
conda run -n figaroh-dev python validate.py` in an isolated examples worktree
at base `3a2c8e9`, linked to core `b134888`, with the TX40 correction applied
before its script run. Existing pytest suite passed (211.3 seconds). The
new TX40 regression was run separately on both profiles; it was added after
that baseline suite run. No new failing regression is being hidden.

Full summary: **11 checks passed, 4 failed, 1 timed out** (no validator-level
skips). The validator summarizes pytest as one check; it does not retain
pytest's complete summary. Its exit status is **1**, not a green full workflow.

| Check | Observed result |
| --- | --- |
| UR10 identification verification | Condition 20617.78 > 1000; improvement 8.03% < 50%; correlation passes |
| TIAGo identification verification | Condition 3617.75 > 1000; improvement/correlation pass |
| TX40 identification verification after alignment | Improvement 13.91% < 50%; condition 966.88 and correlation pass |
| UR10 optimal trajectory | 600-second timeout |
| TIAGo optimal trajectory | Nonzero exit after 200.9 seconds; the validator's retained stderr contains warnings, no diagnostic traceback |

Identification verification in these default scripts falls back to training
data because separate validation data is not configured. The private benchmark
uses its explicit splits, so its metrics differ. Do not call the default
verification an independent held-out pass.

TX40 verification was repeated on the original unmodified loader: improvement
12.74% < 50%, with condition/correlation passing. Thus the quality gate failure
predates the alignment fix. UR10/TIAGo/optimal source was unchanged. Their
failures remain visible and are outside this loader correction. Thresholds
were not loosened. Other calibration, model-update, optimal-configuration and
SO-101 checks passed. This audit is not a claim that all existing workflows pass.

Local logs: `/tmp/figaroh-f22-examples-validate.log`,
`/tmp/figaroh-tx40-before-alignment.log`, and
`/tmp/figaroh-tx40-alignment-test{,41}.log`. Generated reports/models stayed
in the isolated worktree; no original robot dataset/model was overwritten.
Hosted CI publishes fresh offline comparison artifacts and constraining pins.

## Acceleration and sampling blockers

The native regressor check proves matching state/parameter order, not measured
acceleration correctness. Core helper
`calculate_first_second_order_differentiation` loops over `range(nq-1)` and
leaves the final acceleration zero on a fixed-base manipulator. Independent
reproduction: `q_last = 0.5*t**2`, `dt=0.01` returns changing velocity and
zero acceleration instead of 1 rad/s². UR10's last joint moves (raw position
range 0.2271 rad), and its loader uses this helper. This is tracked in
[core #32](https://github.com/thanhndv212/figaroh-plus/issues/32).
UR10 results here are preprocessing-limited, not an accepted scientific
validation of the proposed method. The core synthetic spike uses independent
analytic derivatives and is unaffected.

UR10's 100 Hz loader timestamp/filter path conflicts with the 500 Hz config.
TIAGo's recorded median timestamp spacing is 0.009997 s, while configured
`ts=0.0002` and filter `f_sample=500` disagree. TIAGo differentiation uses
recorded timestamps, but filter calibration still needs an explicit audit.
These are preserved in this protocol, not silently compensated. Before a
production go decision or closure of examples #11, fix/audit acceleration
provenance and rates and repeat the relevant benchmark with a newly frozen
protocol. Real-data temporal blocks do not replace independent experiments.

## Hosted CI status

Run [37026334637](https://github.com/thanhndv212/figaroh-examples/actions/runs/37026334637)
passed the new TX40 regression and UR10 smoke tests on both profiles, but
reported 4 failures / 129 passes / 9 skips on Pinocchio 3.7 and
3 failures / 130 passes / 9 skips on 4.1. Three TIAGo smoke failures required
additional PMB2 base geometry; CI now fetches the complete PMB2/TIAGo mesh
folders. The additional 3.7 failure is the existing TALOS held-out multichain
regression (right z RMSE 2.9431 -> 2.5258 mm, below the required improvement).
It remains enabled and is tracked in
[examples #14](https://github.com/thanhndv212/figaroh-examples/issues/14).
Scientific version pins did not resolve it.

Dataset comparisons run after successful dependency installation even if
pytest fails, retaining their artifacts while preserving the job's failed
test status. These results distinguish dataset execution from a green
repository regression suite. The companion PR remains draft while these
hosted validation issues are unresolved.
