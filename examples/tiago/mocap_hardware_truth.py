"""TIAGo mocap across hardware (#100): what measurement hardware alone explains.

A simulated fixture on ``examples/calibration_harness``. One truth per seed
(``calibration_truth.draw_truth``, joint_offset class) is shared by every
system; each session is simulated at its real postures with its own frames (the
registration-only fit of the real session, nominal joints) and its own noise
stream. The same fixed model as the study (``mocap_hardware.py``) is fitted on
each system's training session, evaluated on the 2023 sessions by LOO
re-registration, and the *simulated* differences between systems give the
hardware-only thresholds that the real differences will be read against:

- transfer band(A -> E) = |mean_s G_s| + 2 sd_s(G_s), where
  ``G = e(M_A, E) - e(M_same, E)`` is the LOO re-registered RMSE of system A's
  model on session E minus that of E's own system, over seeds ``s``;
- parameter band_j(A, B) = 2 RMS_s(theta_jA - theta_jB), for the comparable
  joints ``arm_2 .. arm_6``.

Noise: isotropic ``level`` mm per axis (grid ``NOISE_GRID_MM``), a stream
``default_rng(10_000 + 100 k + seed)`` per system ``k`` so systems are never
correlated; orientation noise (6D run) is the median ``pose_std_deg`` of the
system's files. The level used for the pre-registration is the smallest grid
level at or above each system's effective noise (``noise_estimates``).

Only simulated measurements are fitted here; the real sessions enter through
their postures and their registration-only frames.

    python mocap_hardware_truth.py --seeds 0 1 2 3 4 --out bands.json
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.calibration_harness import simulation as sim  # noqa: E402
from examples.tiago import calibration_truth as ct  # noqa: E402
from examples.tiago import mocap_hardware as mh  # noqa: E402

SYSTEMS = ("qualisys", "optitrack", "vicon")  # k = 0, 1, 2
EVALUATED = {
    "optitrack": ["2023-09-12-1658", "2023-09-12-1552"],
    "vicon": ["2023-09-20-1508", "2023-09-20-1607"],
}
SEEDS = (0, 1, 2, 3, 4)
MODES = ("3d", "6d")
# A -> E pairs of the transfer bands: every model on every evaluated session of
# another system (the model of the session's own system is the reference)
MODELS = ("qualisys", "optitrack", "vicon")


def stream(system: str, seed: int) -> np.random.Generator:
    """The noise stream of one system: independent across systems."""
    return np.random.default_rng(10_000 + 100 * SYSTEMS.index(system) + seed)


class Fixture:
    """Real postures and registration-only frames of every simulated session."""

    def __init__(self):
        self.sessions = mh.load_sessions()
        train = {s: mh.training_session(s, self.sessions) for s in SYSTEMS}
        self.train = train
        self.order = {s: [train[s].id] for s in SYSTEMS}
        for s, ids in EVALUATED.items():
            self.order[s] += [i for i in ids if i != train[s].id]
        self.chain = mh.Chain(mh.make_calib(train["qualisys"].path, solve=False))
        self.real = {}
        self.frames = {}
        self.rot_sigma = {}
        for s in SYSTEMS:
            for sid in self.order[s]:
                path = self.sessions[f"{s}/{sid}"].path
                six_d = s != "qualisys"
                data = mh.read_data(path, six_d=six_d)
                R, p = self.chain.tool_pose(data.q, np.zeros(8))
                self.real[(s, sid)] = data
                # registration only: nominal joints, base and tool frames
                self.frames[(s, sid)] = mh.reregister(R, p, data.pos, data.rot)
            if s != "qualisys":
                std = pd.read_csv(self.sessions[f"{s}/{self.order[s][0]}"].path)[
                    "pose_std_deg"
                ]
                self.rot_sigma[s] = float(np.deg2rad(np.median(std)))

    def truth(self, seed: int) -> np.ndarray:
        """(8,) true joint offsets, drawn from ``default_rng(seed)``."""
        t = ct.draw_truth("joint_offset", np.random.default_rng(seed), self.chain.model)
        return np.array([t[n] for n in mh.OFFSET_NAMES])

    def simulate(self, system: str, seed: int, level: float, theta: np.ndarray):
        """``{session: (clean, noisy)}`` of one system; the stream is per system."""
        rng = stream(system, seed)
        out = {}
        for sid in self.order[system]:
            real = self.real[(system, sid)]
            six_d = real.rot is not None
            R, p = self.chain.tool_pose(real.q, theta)
            pos, rot = mh._predict(R, p, self.frames[(system, sid)], six_d)
            noisy = sim.gaussian_mm(pos, level, rng)
            noisy_rot = None
            if six_d:
                n = rng.normal(0.0, self.rot_sigma[system], (len(pos), 3))
                noisy_rot = np.einsum(
                    "nij,njk->nik", Rotation.from_rotvec(n).as_matrix(), rot
                )
            out[sid] = (
                mh.Data(real.q, pos, rot),
                mh.Data(real.q, noisy, noisy_rot),
            )
        return out


def write_csv(data: mh.Data, path: Path) -> Path:
    cols = {"x1": data.pos[:, 0], "y1": data.pos[:, 1], "z1": data.pos[:, 2]}
    if data.rot is not None:
        rpy = np.array([mh.matrix_to_rpy(r) for r in data.rot])
        cols.update(phix1=rpy[:, 0], phiy1=rpy[:, 1], phiz1=rpy[:, 2])
    df = pd.DataFrame(cols)
    for k, j in enumerate(mh.JOINTS):
        df[j] = data.q[:, k]
    df.to_csv(path, index=False)
    return path


def fit_models(fx: Fixture, seed: int, levels: Dict[str, float], workdir: Path, truth):
    """Structural fit of every system's training session (and 6D for 2023)."""
    sims = {s: fx.simulate(s, seed, levels[s], truth) for s in SYSTEMS}
    fits = {}
    for s in SYSTEMS:
        sid = fx.train[s].id
        noisy = sims[s][sid][1]
        for mode in MODES:
            if mode == "6d" and s == "qualisys":
                continue
            six_d = mode == "6d"
            path = write_csv(
                noisy if six_d else mh.Data(noisy.q, noisy.pos),
                workdir / f"{s}_{seed}_{mode}.csv",
            )
            params, std, calib = mh.fit_session(path, six_d=six_d)
            fits[(s, mode)] = {
                "params": params,
                "std": std,
                "status": int(calib.LM_result.status),
                "absorbed": list(calib.calib_config.get("absorbed_param_name", [])),
            }
    return sims, fits


def run_cell(fx: Fixture, seed: int, levels: Dict[str, float], workdir: Path) -> dict:
    """One seed at per-system noise levels: fits, recoveries, LOO matrices."""
    truth = fx.truth(seed)
    sims, fits = fit_models(fx, seed, levels, workdir, truth)
    for mode in MODES:
        names = {k: sorted(v["params"]) for k, v in fits.items() if k[1] == mode}
        if len({tuple(n) for n in names.values()}) != 1:
            raise RuntimeError(f"fits keep different parameter sets: {names}")
    z = []
    for f in fits.values():
        z += [
            (f["params"][n] - truth[mh.OFFSET_NAMES.index(n)]) / f["std"][n]
            for n in mh.COMPARABLE
            if f["std"][n] > 0
        ]
    theta = {
        mode: {
            s: mh.offsets_of(fits[(s, mode if (s, mode) in fits else "3d")]["params"])
            for s in SYSTEMS
        }
        for mode in MODES
    }
    theta["zero"] = np.zeros(8)
    matrix = {}
    for system, ids in EVALUATED.items():
        for sid in ids:
            clean, noisy = sims[system][sid]
            for model in (*MODELS, "registration only"):
                for mode in MODES:
                    off = (
                        theta["zero"]
                        if model == "registration only"
                        else theta[mode][model]
                    )
                    data = noisy if mode == "6d" else mh.Data(noisy.q, noisy.pos)
                    matrix[f"{model}|{system}/{sid}|{mode}"] = mh.score(
                        fx.chain, data, off, truth=clean.pos
                    )
    return {
        "seed": seed,
        "levels": dict(levels),
        "z": z,
        "params": {
            f"{s}|{mode}": {n: f["params"][n] for n in mh.COMPARABLE}
            for (s, mode), f in fits.items()
        },
        "status": {f"{s}|{mode}": f["status"] for (s, mode), f in fits.items()},
        "absorbed": sorted({tuple(f["absorbed"]) for f in fits.values()}),
        "matrix": matrix,
    }


def _band(g: np.ndarray) -> float:
    return float(abs(np.mean(g)) + 2.0 * np.std(g, ddof=1))


def summarize(cells: List[dict]) -> dict:
    """Bands, recovery and a descriptive table from the cells of one noise setting."""
    keys = cells[0]["matrix"].keys()
    sessions = [f"{s}/{i}" for s, ids in EVALUATED.items() for i in ids]
    transfer, judge, descriptive = {}, {}, {}
    for mode in MODES:
        metric = "loo_mm" if mode == "3d" else "loo_6d_mm"
        for sess in sessions:
            own = sess.split("/")[0]
            for model in MODELS:
                if model == own:
                    continue
                g = np.array(
                    [
                        c["matrix"][f"{model}|{sess}|{mode}"][metric]
                        - c["matrix"][f"{own}|{sess}|{mode}"][metric]
                        for c in cells
                    ]
                )
                transfer[f"{model}->{sess}|{mode}"] = {
                    "gap_mean": float(g.mean()),
                    "gap_sd": float(g.std(ddof=1)),
                    "band": _band(g),
                }
                gt = np.array(
                    [
                        c["matrix"][f"{model}|{sess}|{mode}"]["loo_vs_truth_mm"]
                        - c["matrix"][f"{own}|{sess}|{mode}"]["loo_vs_truth_mm"]
                        for c in cells
                    ]
                )
                judge[f"{model}->{sess}|{mode}"] = float(gt.mean())
    for k in keys:
        descriptive[k] = {
            m: float(np.mean([c["matrix"][k][m] for c in cells]))
            for m in cells[0]["matrix"][k]
            if m != "n"
        }
    parameter = {}
    for mode in MODES:
        systems = SYSTEMS if mode == "3d" else ("optitrack", "vicon")
        for i, a in enumerate(systems):
            for b in systems[i + 1 :]:
                ka = f"{a}|{mode}" if f"{a}|{mode}" in cells[0]["params"] else f"{a}|3d"
                kb = f"{b}|{mode}" if f"{b}|{mode}" in cells[0]["params"] else f"{b}|3d"
                parameter[f"{a}-{b}|{mode}"] = {
                    j: float(
                        2.0
                        * np.sqrt(
                            np.mean(
                                [
                                    (c["params"][ka][j] - c["params"][kb][j]) ** 2
                                    for c in cells
                                ]
                            )
                        )
                    )
                    for j in mh.COMPARABLE
                }
    z = np.array([v for c in cells for v in c["z"]])
    return {
        "levels": cells[0]["levels"],
        "seeds": [c["seed"] for c in cells],
        "transfer_band_mm": transfer,
        "judge_gap_mm": judge,
        "parameter_band_rad": parameter,
        "recovery_z": {
            "n": int(z.size),
            "mean": float(z.mean()),
            "rms": float(np.sqrt(np.mean(z**2))),
            "max_abs": float(np.abs(z).max()),
        },
        "mean": descriptive,
    }


def run(
    seeds=SEEDS,
    grid=mh.NOISE_GRID_MM,
    used: Optional[Dict[str, float]] = None,
    workdir=None,
) -> dict:
    """Every grid level (common to all systems) and the pre-registered levels."""
    fx = Fixture()
    if used is None:
        est = mh.noise_estimates(fx.sessions)
        used = {s: est[s]["noise_level_mm"] for s in SYSTEMS}
    settings = {f"grid {lv} mm": {s: lv for s in SYSTEMS} for lv in grid}
    settings["used"] = used
    cache = {}
    out = {"noise_estimates_mm": used}
    tmp = tempfile.TemporaryDirectory() if workdir is None else None
    root = Path(workdir or tmp.name)
    for label, levels in settings.items():
        cells = []
        for seed in seeds:
            key = (seed, tuple(levels.values()))
            if key not in cache:
                cache[key] = run_cell(fx, seed, levels, root)
            cells.append(cache[key])
        out[label] = summarize(cells)
        if label == "used":
            out["seed0_used"] = cells[0]
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument(
        "--noise", type=float, nargs="+", default=list(mh.NOISE_GRID_MM)
    )
    parser.add_argument("--out", type=Path, help="write the summary as JSON")
    parser.add_argument(
        "--golden",
        type=Path,
        help="write the golden file (first seed at the used levels, and the "
        "pre-registered summary)",
    )
    args = parser.parse_args(argv)
    res = run(args.seeds, args.noise)
    if args.out:
        args.out.write_text(json.dumps(res, indent=1))
    if args.golden:
        golden = {
            "noise_estimates_mm": res["noise_estimates_mm"],
            "seed0": res["seed0_used"],
            "preregistered": res["used"],
        }
        args.golden.write_text(json.dumps(golden, indent=1, sort_keys=True) + "\n")
    for label, s in res.items():
        if not isinstance(s, dict) or "transfer_band_mm" not in s:
            continue
        z = s["recovery_z"]
        print(f"\n{label}: recovery z rms {z['rms']:.2f} (n={z['n']})")
        for k, v in s["transfer_band_mm"].items():
            print(
                f"  {k:48s} gap {v['gap_mean']:+.3f} sd {v['gap_sd']:.3f}  band {v['band']:.3f}"
            )
        for k, v in s["parameter_band_rad"].items():
            print(
                f"  {k:20s} "
                + "  ".join(f"{j[8:13]} {x*1e3:.2f}" for j, x in v.items())
                + " mrad"
            )


if __name__ == "__main__":
    main()
