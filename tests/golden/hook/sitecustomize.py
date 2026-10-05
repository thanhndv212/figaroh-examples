"""Record what an example script's calibration/identification produces (#76).

Loaded through ``PYTHONPATH`` by ``tests/golden/golden_outputs.py`` only when
``FIGAROH_GOLDEN_LOG`` is set. It wraps ``BaseCalibration.solve`` and
``BaseIdentification.solve`` and appends one JSON line per call; the example
scripts run unchanged.
"""

import json
import os
import sys


def _install(log_path):
    import numpy as np
    from figaroh.calibration.base_calibration import BaseCalibration
    from figaroh.identification.base_identification import BaseIdentification

    def write(record):
        with open(log_path, "a") as f:
            f.write(json.dumps(record) + "\n")

    calib_solve = BaseCalibration.solve

    def solve_calibration(self, *args, **kwargs):
        out = calib_solve(self, *args, **kwargs)
        cfg = self.calib_config
        n_meas = self.PEE_measured.size
        residual = np.asarray(self.LM_result.fun[:n_meas])
        write(
            {
                "kind": "calibration",
                "calib_model": cfg["calib_model"],
                "param_name": list(cfg["param_name"]),
                "x": np.asarray(self.LM_result.x, float).tolist(),
                "fit_rms": float(np.sqrt(np.mean(residual**2))),
                "end_frame": cfg.get("end_frame"),
            }
        )
        return out

    ident_solve = BaseIdentification.solve

    def solve_identification(self, *args, **kwargs):
        out = ident_solve(self, *args, **kwargs)
        tau_meas = np.asarray(self.tau_noised, float).ravel()
        tau_est = np.asarray(self.tau_identif, float).ravel()
        write(
            {
                "kind": "identification",
                "phi_base": np.asarray(self.phi_base, float).ravel().tolist(),
                "tau_rmse": float(np.sqrt(np.mean((tau_meas - tau_est) ** 2))),
            }
        )
        return out

    BaseCalibration.solve = solve_calibration
    BaseIdentification.solve = solve_identification


if os.environ.get("FIGAROH_GOLDEN_LOG"):
    try:
        _install(os.environ["FIGAROH_GOLDEN_LOG"])
    except Exception as exc:  # the comparison then reports a missing record
        print(f"golden-output hook not installed: {exc}", file=sys.stderr)
