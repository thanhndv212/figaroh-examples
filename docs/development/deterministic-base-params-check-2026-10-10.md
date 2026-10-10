# Deterministic base parameters: effect on the identification examples (2026-10-10)

figaroh-plus#169 makes QR column pivoting deterministic. Columns whose norms
match to within a relative 1e-6 count as tied, and a tie goes to the
smallest parameter name. This can change which parameter stands for a group
of linearly dependent columns. It does not change the rank, the span or the
fitted torques. This check records what that means for each identification
example (#116).

## Method

Each example's `identification.py --verify` ran from its own directory on
two cores: the commit before #169 merged (figaroh-plus `d4e8c0c`) and
`devel` after it. The runs set
`MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1`.
A `sitecustomize.py` on `PYTHONPATH` wrapped `BaseIdentification.solve` and
logged `params_base`, `phi_base`, `result["condition number"]` and the
torque RMSE. This is the same wrapping as `tests/golden/hook/`.

## Results

| Example | Base params | Torque RMSE | Condition number | Base-parameter names |
|---|---|---|---|---|
| staubli_tx40 | 58 → 58 | 2.99224494, unchanged | 966.9, unchanged | 4 replaced, 42 reordered |
| tiago | 73 → 73 | 0.665947359, unchanged | 2585 → 1782 | 12 replaced, 41 reordered |
| so101 | 23 → 23 | 0.00822355681, unchanged | 429.2, unchanged | unchanged |
| ur10 (truth fixture) | 36 → 36 | unchanged | train 61.7 → 92.7, validation 198.8 → 162.5 | 6 replaced |

Each replaced name is a different representative of the same dependent
group. The grouped expressions span the same space.

- **staubli_tx40:** before #169 the replaced representatives were
  `Ixx_joint_2`, `Iyy_joint_2`, `Izz_joint_2` and `mz_joint_5`. After it they
  are `Ia_joint_1`, `Ia_joint_2`, `Iyy_joint_2` and `my_joint_4`.
- **tiago:** the changes are in the arm_2 to arm_5 and arm_7 inertias, the
  arm_1 and arm_2 armatures (`Ia`) and the gripper finger.

## Consequences

- **Golden outputs** (`tests/golden/`) pin the number of base parameters and
  the torque RMSE. Neither changed, so all golden cases pass on `devel`.
- **UR10 truth fixture:** this fixture pins base-parameter names and the
  condition numbers, and it is the only example that broke. It was re-pinned
  in #116 (see [ur10-dynamic-truth-fixture.md](ur10-dynamic-truth-fixture.md)).
- **staubli_tx40:** the verification checks the condition number against a
  limit of 1000. The condition number did not change and the check still
  passes. Nothing compares base-parameter names or values with
  `TX40_bp.csv`.
- **tiago:** the condition number fell to 1782. No doc or test records the
  old 2585. The 3617.75 in
  [ur10-signal-audit-2026-10-02.md](ur10-signal-audit-2026-10-02.md) is a
  dated record of an earlier configuration and stays as written.
- **so101:** the "condition number 429" in its README still holds. Its
  friction and offset columns are not tied to any other column, so the
  lookup by name in `so101_tools.py` is unaffected.

**Rule:** a result that names a base parameter, or quotes a base-regressor
condition number, depends on the core version. Record the core version
alongside such a result, or record quantities that do not depend on the
choice of columns: the rank, the fitted torques and the RMSE.
