"""TIAGo base suspension (#70), part 2: the real fits, scored by the frozen rules.

Part 1 (docs/development/tiago-suspension.md, sections 1 to 10) froze the
loader, the models, the selection rule and the verdict rules on a simulated
fixture. This stage applies them, unchanged, to the real runs (section 9):

1. training ``xyz_1640`` and the selection diagnostics (O3): wrench through
   ``read_target``; validation ``xyz_1628`` with ``unlock_validation``;
2. model selection by blocked CV (S1 < S2 < S3), then B0, B1 (status quo),
   B2 (cube), S1B2 and S1/S2/S3 fitted on training; bootstrap SE of the
   selected model and the calibrated SE ``sqrt(SE^2 + bias^2)`` (fixture bias);
3. the frozen fits scored on validation; the selected model refitted on
   validation for repeatability (band of the fixture);
4. section 8: "enough", then Q2; the confirmation runs are read only if
   validation says "enough" (once, frozen model);
5. ``Mz`` under both conventions (step 6, O7): it counts only if both give the
   same DOF verdict;
6. the vertical-mode cross-check of ``k_z`` (step 7, O8), descriptive.

No threshold is chosen here: every rule and number comes from the study
config or the fixture golden (``tests/golden/tiago_suspension_truth.json``).

    python suspension_study.py --out ../../tests/golden/tiago_suspension_study.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import suspension_model as sm  # noqa: E402
from examples.tiago.utils import suspension_data as sd  # noqa: E402

FIXTURE_GOLDEN = project_root / "tests/golden/tiago_suspension_truth.json"
RULES = sm.RULES
DOFS = sm.DOFS
MZ = DOFS.index("rz")
CONVENTIONS = ("centre", "cop")


def fixture_context(path=FIXTURE_GOLDEN) -> dict:
    """Answerable DOF, repeatability band and S1 bias of the frozen fixture."""
    return json.loads(Path(path).read_text())["context"]


def _r2(w, pred, valid) -> np.ndarray:
    return sm.score(w, pred, valid)["r2"]


def _resid(fit: sm.Fit, r: sm.Regressors, w: np.ndarray) -> np.ndarray:
    """In-sample residual including the offset (the nonlinearity flag input)."""
    full = sm._full_design(sm.design(fit.model, r, fit.h_e), fit.offset)
    return w - full @ fit.params


def flags(fit: sm.Fit, r: sm.Regressors, w: np.ndarray) -> np.ndarray:
    res = _resid(fit, r, w)
    return np.array([sm.nonlinear_flag(r.x[:, j], res[:, j], r.valid) for j in range(6)])


def load(mz_about: str) -> dict:
    """``(run, wrench)`` of the selection set (training first) and of validation."""
    sessions = sm.load_sessions(verify=True)
    selection = [
        (sm.read_regressors(s), sm.read_target(s, mz_about=mz_about))
        for s in sm.selection_sessions()
    ]
    val = sessions[sm.SETTINGS["sessions"]["validation"]]
    rv = sm.read_regressors(val)
    wv = sm.read_target(val, unlock_validation=True, mz_about=mz_about)
    return {"selection": selection, "training": selection[0], "validation": (rv, wv)}


def status_quo_b1() -> dict:
    """B1: the status-quo loader and S1 fit on training, scored on validation."""
    sessions = sm.load_sessions(verify=False)
    tr = sd.load_status_quo(sessions[sm.SETTINGS["sessions"]["training"]].path)
    va = sd.load_status_quo(sessions[sm.SETTINGS["sessions"]["validation"]].path)
    f = sm.fit_status_quo(tr.base_displacement, tr.base_velocity, tr.wrench)
    pred = sm.predict_status_quo(f.parameters, va.base_displacement, va.base_velocity)
    r2 = _r2(va.wrench, pred, np.ones(va.sample_count, bool))
    return {
        "r2": r2.tolist(),
        "k": f.parameters[0::2].tolist(),
        "c": f.parameters[1::2].tolist(),
        "neg_damping": int(np.sum(f.parameters[1::2] < 0.0)),
    }


def parameters(fit: sm.Fit, ctx: dict) -> dict:
    """Diagonal stiffness and damping with the bootstrap and the calibrated SE.

    The calibrated SE adds the fixture's S1 relative bias (F2 bias) in
    quadrature, ``sqrt(SE^2 + (bias |value|)^2)``; for S2 and S3 the S1 bias is
    the only one the fixture measured (stated in the note).
    """
    k, c = fit.k(), fit.c()
    out = {"k": k.tolist(), "c": c.tolist()}
    if fit.se is not None:
        k_se, c_se = fit.k_se(), fit.c_se()
        out.update(
            k_se=k_se.tolist(),
            c_se=c_se.tolist(),
            k_se_cal=np.hypot(k_se, np.asarray(ctx["k_bias"]) * np.abs(k)).tolist(),
            c_se_cal=np.hypot(c_se, np.asarray(ctx["c_bias"]) * np.abs(c)).tolist(),
        )
    if fit.model in ("S1", "S2", "S1B2"):
        out["c_unidentified"] = fit.damping_unidentified().tolist()
    if fit.model == "S3":
        kk, cc = fit.matrices()
        out["K_eig"] = np.linalg.eigvalsh(kk).tolist()
        out["C_eig"] = np.linalg.eigvalsh(cc).tolist()
    return out


def reasons(v: dict, rep, at_bound, nonlin) -> list:
    """Per DOF, the section 8 items that fail (empty: the DOF is ok)."""
    out = []
    for j in range(6):
        r = []
        if not v["explained"][j]:
            r.append("not explained")
        if not v["beats_b2"][j]:
            r.append("does not beat B2")
        if at_bound[j]:
            r.append("stiffness at bound")
        if not rep[j]:
            r.append("not repeatable")
        if nonlin[j]:
            r.append("nonlinear")
        out.append(r)
    return out


def evaluate(mz_about: str, ctx: dict, bootstrap: bool = True) -> dict:
    """Steps 1 to 4 of section 9 under one ``Mz`` convention (no confirmation)."""
    d = load(mz_about)
    rt, wt = d["training"]
    rv, wv = d["validation"]
    choice = sm.select_model(d["selection"])
    model, h_e = choice["model"], choice["h_e"]

    fits = {
        "S1": sm.fit_model("S1", rt, wt, bootstrap=bootstrap and model != "S1"),
        "S2": sm.fit_model("S2", rt, wt, h_e=choice["h_e_cv"]),
        "S3": sm.fit_model("S3", rt, wt),
        "B2": sm.fit_model("B2", rt, wt),
        "S1B2": sm.fit_model("S1B2", rt, wt),
    }
    sel = sm.fit_model(model, rt, wt, h_e=h_e, bootstrap=bootstrap)
    fits[model] = sel
    r2 = {n: _r2(wv, sm.predict(f, rv), rv.valid) for n, f in fits.items()}
    r2_train = {n: _r2(wt, sm.predict(f, rt), rt.valid) for n, f in fits.items()}
    r2["B0"] = np.zeros(6)

    sel_v = sm.fit_model(model, rv, wv, h_e=h_e)
    kv = np.abs(sel_v.k())
    k_rel_diff = np.abs(sel.k() - sel_v.k()) / np.where(kv > 0, kv, np.nan)
    rep = k_rel_diff <= ctx["band"]
    at_bound = sel.stiffness_bound_dofs()
    corr = flags(sel, rt, wt)
    nonlin = np.abs(corr) > RULES["nonlinear_corr"]
    b2 = fits["B2"]
    v = sm.verdict(
        r2[model], r2["B2"], at_bound, rep, nonlin, ctx["answerable"], b2.b2_at_bound()
    )
    q2 = sm.suspension_adds(r2["S1B2"], r2["B2"], fits["S1B2"].stiffness_bound_dofs())

    out = {
        "mz_about": mz_about,
        "selection": choice,
        "model": model,
        "h_e": h_e,
        "r2_validation": {n: x.tolist() for n, x in r2.items()},
        "r2_training": {n: x.tolist() for n, x in r2_train.items()},
        "selected": parameters(sel, ctx),
        "selected_validation_k": sel_v.k().tolist(),
        "k_rel_diff": np.nan_to_num(k_rel_diff, nan=np.inf).tolist(),
        "repeatable": rep.tolist(),
        "at_bound": at_bound.tolist(),
        "any_bound": sel.bound_dofs().tolist(),
        "nonlinear_corr": corr.tolist(),
        "nonlinear": nonlin.tolist(),
        "verdict": v,
        "reasons": reasons(v, rep, at_bound, nonlin),
        "q2": q2,
        "b2": {
            "theta": b2.theta.tolist(),
            "at_bound": b2.b2_at_bound(),
            "mass": float(b2.theta[0]),
        },
        "s1b2": {
            "k": fits["S1B2"].k().tolist(),
            "c": fits["S1B2"].c().tolist(),
            "mass": float(fits["S1B2"].theta[12]),
            "k_at_bound": fits["S1B2"].stiffness_bound_dofs().tolist(),
        },
        "fits": {n: parameters(f, ctx) for n, f in fits.items() if n in ("S1", "S2")},
        "cond": {n: f.cond for n, f in fits.items()},
        "_fits": fits,
        "_data": d,
    }
    if bootstrap:
        out["s1_eiv"] = sm.fit_s1_eiv(rt, wt).tolist()  # sensitivity row (F2)
    return out


def combine(res: Dict[str, dict], ctx: dict) -> dict:
    """Step 6 (O7): ``Mz`` enters the answerable set only if both conventions agree."""
    a, b = res["centre"], res["cop"]
    same = a["verdict"]["dof_ok"][MZ] == b["verdict"]["dof_ok"][MZ]
    answerable = np.asarray(ctx["answerable"], bool).copy()
    if not same:
        answerable[MZ] = False
    ok = np.asarray(a["verdict"]["dof_ok"], bool)
    enough = bool(answerable.any() and ok[answerable].all() and not a["b2"]["at_bound"])
    q2_adds = np.asarray(a["q2"]["adds"], bool)
    return {
        "mz_same_verdict": bool(same),
        "answerable": answerable.tolist(),
        "enough": enough,
        "q2_present": bool(q2_adds[answerable].any()),
        "explained_dofs": [DOFS[j] for j in range(6) if a["verdict"]["explained"][j]],
    }


def vertical_mode(k_z: float) -> dict:
    """Step 7 (O8): the fitted ``k_z`` against the ``Fz`` mode of training, and the
    10 to 45 Hz band on both posture runs (base at rest; description only)."""
    sessions = sm.load_sessions(verify=False)
    out = {"k_z_fit": float(k_z)}
    for sid in (sm.SETTINGS["sessions"]["training"], "postures_1720", "postures_1734"):
        _, v = sd.read_raw(sessions[sid].path)
        out[sid] = sm.fz_mode(v)
    out["k_z_over_mode"] = float(k_z / out[sm.SETTINGS["sessions"]["training"]]["k_z"])
    return out


def confirm(res: dict, ctx: dict) -> dict:
    """Section 8 on each confirmation run with the frozen model (only after "enough")."""
    fits = res["_fits"]
    model, h_e = res["model"], res["h_e"]
    sel, b2 = fits[model], fits["B2"]
    rt, wt = res["_data"]["training"]
    sessions = sm.load_sessions(verify=True)
    out = {}
    for sid in sm.SETTINGS["sessions"]["confirmation"]:
        s = sessions[sid]
        r = sm.read_regressors(s, unlock_confirmation=True)
        w = sm.read_target(s, unlock_confirmation=True, mz_about=res["mz_about"])
        r2 = _r2(w, sm.predict(sel, r), r.valid)
        r2_b2 = _r2(w, sm.predict(b2, r), r.valid)
        refit = sm.fit_model(model, r, w, h_e=h_e)
        kv = np.abs(refit.k())
        rep = np.abs(sel.k() - refit.k()) <= ctx["band"] * np.where(kv > 0, kv, np.nan)
        nonlin = np.abs(flags(sel, rt, wt)) > RULES["nonlinear_corr"]
        v = sm.verdict(
            r2, r2_b2, sel.stiffness_bound_dofs(), rep, nonlin,
            res["answerable"], b2.b2_at_bound(),
        )  # fmt: skip
        out[sid] = {"r2": r2.tolist(), "r2_b2": r2_b2.tolist(), "verdict": v}
    out["enough"] = bool(all(out[s]["verdict"]["enough"] for s in sm.SETTINGS["sessions"]["confirmation"]))
    return out


def run(verbose: bool = True) -> dict:
    ctx = fixture_context()
    res = {}
    for conv in CONVENTIONS:
        if verbose:
            print(f"[study] Mz about {conv}", file=sys.stderr)
        res[conv] = evaluate(conv, ctx, bootstrap=conv == "centre")
    final = combine(res, ctx)
    main = res["centre"]
    out = {
        "context": {k: ctx[k] for k in ("answerable", "band", "k_bias", "c_bias")},
        "b1": status_quo_b1(),
        "final": final,
        "vertical_mode": vertical_mode(main["selected"]["k"][2]),
    }
    if final["enough"]:
        main["answerable"] = final["answerable"]
        out["confirmation"] = confirm(main, ctx)
    else:
        out["confirmation"] = "not read: validation does not say enough"
    for conv in CONVENTIONS:
        res[conv].pop("_fits")
        res[conv].pop("_data")
    out.update(res)
    return out


def _clean(x):
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, (np.floating, float)):
        f = float(x)
        return f if np.isfinite(f) else str(f)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, help="write the result JSON here")
    args = parser.parse_args(argv)
    res = _clean(run())
    text = json.dumps(res, indent=1)
    if args.out:
        args.out.write_text(text + "\n")
    print(json.dumps(res["final"], indent=1))


if __name__ == "__main__":
    main()
