# Worked experiment briefs

Each brief fills in the [experiment brief template](../experiment-brief-template.md)
section by section for a task that already runs in this repository, then
records what the commands produced next to the accepted evidence. Read them
as examples of what a brief forces you to write down, not as a second copy of
the reports they link.

| Brief | Task | Shows |
|---|---|---|
| [UR10 truth identification](ur10-truth-identification.md) | Dynamic identification, simulated known truth (D7) | A gate set by a noise floor; the exported URDF checked against the truth |
| [TIAGo identification](tiago-identification.md) | Dynamic identification, real logs (D7) | An accepted `physical_fit`, a deliberately rejected `reconstruction`, the lumped `arm_7` body, ungated held-out numbers |
| [TIAGo mocap calibration](tiago-mocap-calibration.md) | Geometric calibration, real mocap (C4) | Frozen sessions with roles, gauge frames, parity of the export |
| [TX40 without validation data](tx40-no-validation-data.md) | Dynamic identification, no held-out recording | The fallback warning, and which claims are therefore unsupported |

All commands are headless. Set `MPLBACKEND=Agg` before any script that might
open a window, and run inside the `figaroh-dev` environment. The commands
below write archives under `results/runs/` (git-ignored) unless `--root` is
given.

Numbers in the "obtained" columns were measured on figaroh-examples
`b167298` and figaroh-plus (core) `779e830` (`devel`, v0.7.0), Python 3.12.11,
Pinocchio 3.7.0, macOS arm64. The "accepted" columns are the values in the
accepted evidence named in each brief.
