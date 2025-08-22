# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Base class for robot dynamic parameter identification.
This module provides a generalized framework for dynamic parameter identification
that can be inherited by any robot type (TIAGo, UR10, MATE, etc.).
"""

import yaml
import numpy as np
import pandas as pd
from abc import ABC, abstractmethod
from os.path import abspath

# FIGAROH imports
from figaroh.identification.identification_tools import (
    get_param_from_yaml as get_identification_param_from_yaml,
)
from figaroh.tools.regressor import (
    build_regressor_basic,
    get_index_eliminate,
    build_regressor_reduced,
)
from figaroh.identification.identification_tools import get_standard_parameters


class BaseIdentification(ABC):
    """
    Base class for robot dynamic parameter identification.
    
    Provides common functionality for all robots while allowing
    robot-specific implementations of key methods.
    """
    
    def __init__(self, robot, config_file="config/robot_config.yaml"):
        """Initialize base identification with robot model and configuration.
        
        Args:
            robot: Robot model loaded with FIGAROH
            config_file: Path to robot configuration YAML file
        """
        self._robot = robot
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration using the TIAGo-style approach
        self.load_param(config_file)
        
        # Initialize attributes for identification results (TIAGo-style)
        self.W = None
        self.standard_parameter = None
        self.params_base = None
        self.idx_base = None
        self.idx_e = None
        self.params_r = None
        self.W_base = None
        self.phi_base = None
        self.phi_standard = None
        self.phi_ref = None
        self.rms_error = None
        self.correlation = None
        self.processed_data = None
        self.result = None
        self.Nsample_ = None
        self.tau_ref = None
        self.tau_identif = None
        self.tau_noised = None
        
        print(f"{self.__class__.__name__} initialized")
    
    def load_param(self, config_file, setting_type="identification"):
        """Load the identification parameters from the yaml file."""
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
        self.params_settings = get_identification_param_from_yaml(
            self._robot, config[setting_type]
        )

    
    def load_csv_data(self):
        """Load and process CSV data (generic implementation)."""
        ts = pd.read_csv(
            abspath(self.params_settings["pos_data"]), usecols=[0]
        ).to_numpy()
        pos = pd.read_csv(abspath(self.params_settings["pos_data"]))
        vel = pd.read_csv(abspath(self.params_settings["vel_data"]))
        eff = pd.read_csv(abspath(self.params_settings["torque_data"]))

        cols = {"pos": [], "vel": [], "eff": []}
        for jn in self.params_settings["active_joints"]:
            cols["pos"].extend([col for col in pos.columns if jn in col])
            cols["vel"].extend([col for col in vel.columns if jn in col])
            cols["eff"].extend([col for col in eff.columns if jn in col])

        q = pos[cols["pos"]].to_numpy()
        dq = vel[cols["vel"]].to_numpy()
        tau = eff[cols["eff"]].to_numpy()
        return ts, q, dq, tau
    
    def apply_filters(self, t, q, dq, nbutter=4, f_butter=2, med_fil=5, f_sample=100):
        """Apply median and lowpass filters to position and velocity data."""
        from scipy import signal
        b1, b2 = signal.butter(nbutter, f_butter / (f_sample / 2), "low")

        q_filtered = np.zeros(q.shape)
        dq_filtered = np.zeros(dq.shape)

        for j in range(dq.shape[1]):
            q_med = signal.medfilt(q[:, j], med_fil)
            q_filtered[:, j] = signal.filtfilt(
                b1, b2, q_med, padtype="odd", padlen=3 * (max(len(b1), len(b2)) - 1)
            )
            dq_med = signal.medfilt(dq[:, j], med_fil)
            dq_filtered[:, j] = signal.filtfilt(
                b1, b2, dq_med, padtype="odd", padlen=3 * (max(len(b1), len(b2)) - 1)
            )
        return q_filtered, dq_filtered

    def estimate_acceleration(self, t, dq_filtered):
        """Estimate acceleration from filtered velocity."""
        return np.array([
            np.gradient(dq_filtered[:, j]) / np.gradient(t[:, 0])
            for j in range(dq_filtered.shape[1])
        ]).T

    def build_full_configuration(self, q_f, dq_f, ddq_f, N_):
        """Build full configuration arrays for position, velocity, and acceleration."""
        p = np.tile(self._robot.q0, (N_, 1))
        v = np.tile(self._robot.v0, (N_, 1))
        a = np.tile(self._robot.v0, (N_, 1))

        p[:, self.params_settings["act_idxq"]] = q_f
        v[:, self.params_settings["act_idxv"]] = dq_f
        a[:, self.params_settings["act_idxv"]] = ddq_f
        return p, v, a
    
    def process_torque_data(self, tau):
        """Process torque data (generic implementation, should be overridden for robot-specific processing)."""
        # Generic torque processing - robots should override this method
        return tau

    def process_data(self, truncate=None):
        """Load and process data"""
        t_, q_, dq_, tau_ = self.load_csv_data()

        # Truncate data if truncation indices are provided
        if truncate is not None:
            if isinstance(truncate, (list, tuple)) and len(truncate) == 2:
                # Use provided truncation indices
                n_i, n_f = truncate
                t_ = t_[n_i:n_f]
                q_ = q_[n_i:n_f]
                dq_ = dq_[n_i:n_f]
                tau_ = tau_[n_i:n_f]

        # Apply filters and estimate acceleration
        q_filtered_, dq_filtered_ = self.apply_filters(t_, q_, dq_)
        ddq_filtered_ = self.estimate_acceleration(t_, dq_filtered_)

        # after processing, total number of samples
        self.Nsample_ = q_filtered_.shape[0]

        # Build full configuration
        p_, v_, a_ = self.build_full_configuration(
            q_filtered_, dq_filtered_, ddq_filtered_, self.Nsample_
        )

        # Process torque data
        tau_prcd = self.process_torque_data(tau_)
        self.processed_data = {
            "t": t_,
            "p": p_,
            "v": v_,
            "a": a_,
            "tau": tau_prcd,
        }

    def calc_full_regressor(self):
        """Build regressor matrix."""
        self.W = build_regressor_basic(
            self._robot,
            self.processed_data["p"],
            self.processed_data["v"],
            self.processed_data["a"],
            self.params_settings,
        )
        self.standard_parameter = self.get_standard_parameters()
        # joint torque estimated from p,v,a with std params
        phi_ref = np.array(list(self.standard_parameter.values()))
        tau_ref = np.dot(self.W, phi_ref)
        self.tau_ref = tau_ref[range(len(self.params_settings["act_idxv"]) * self.Nsample_)]

    def calc_baseparam(self, decimate=True, plotting=True, save_params=False):
        """Calculate base parameters."""
        from figaroh.tools.qrdecomposition import double_QR
        from figaroh.identification.identification_tools import relative_stdev
        
        # Eliminate zero columns
        idx_e_, active_parameter_ = get_index_eliminate(
            self.W, self.standard_parameter, tol_e=0.001
        )
        W_e_ = build_regressor_reduced(self.W, idx_e_)

        # remove zero-crossing data
        if decimate:
            from scipy import signal
            tau_dec = []
            for i in range(len(self.params_settings["act_idxv"])):
                tau_dec.append(signal.decimate(self.processed_data["tau"][:, i], q=10, zero_phase=True))

            tau_rf = tau_dec[0]
            for i in range(1, len(tau_dec)):
                tau_rf = np.append(tau_rf, tau_dec[i])

            # Process regressor similarly
            W_list = []
            for i in range(len(self.params_settings["act_idxv"])):
                W_dec = []
                for j in range(W_e_.shape[1]):
                    W_dec.append(signal.decimate(
                        W_e_[range(self.params_settings["act_idxv"][i] * self.Nsample_,
                                  (self.params_settings["act_idxv"][i] + 1) * self.Nsample_), j],
                        q=10, zero_phase=True
                    ))

                W_temp = np.zeros((W_dec[0].shape[0], len(W_dec)))
                for k in range(len(W_dec)):
                    W_temp[:, k] = W_dec[k]
                W_list.append(W_temp)

            W_rf = np.zeros((tau_rf.shape[0], W_list[0].shape[1]))
            for i in range(len(W_list)):
                W_rf[range(i * W_list[i].shape[0], (i + 1) * W_list[i].shape[0]), :] = W_list[i]
        else:
            tau_rf = self.processed_data["tau"].flatten() if hasattr(self.processed_data["tau"], 'flatten') else self.processed_data["tau"]
            W_rf = W_e_

        # calculate base parameters
        W_b, bp_dict, base_parameter, phi_b, phi_std = double_QR(
            tau_rf, W_rf, active_parameter_, self.standard_parameter
        )
        rmse = np.linalg.norm(tau_rf - np.dot(W_b, phi_b)) / np.sqrt(tau_rf.shape[0])
        std_xr_ols = relative_stdev(W_b, phi_b, tau_rf)

        self.result = {
            "base regressor": W_b,
            "base parameters": bp_dict,
            "condition number": np.linalg.cond(W_b),
            "rmse norm (N/m)": rmse,
            "torque estimated": np.dot(W_b, phi_b),
            "std dev of estimated param": std_xr_ols,
        }
        
        # Store key results for backward compatibility
        self.W_base = W_b
        self.phi_base = phi_b
        self.params_base = list(bp_dict.keys())
        self.tau_identif = np.dot(W_b, phi_b)
        self.tau_noised = tau_rf
            
        # Calculate quality metrics
        residuals = self.tau_noised - self.tau_identif
        self.rms_error = np.sqrt(np.mean(residuals**2))
        if len(self.tau_noised) > 1 and len(self.tau_identif) > 1:
            try:
                correlation_matrix = np.corrcoef(self.tau_noised, self.tau_identif)
                self.correlation = correlation_matrix[0, 1]
            except:
                self.correlation = 1.0
        else:
            self.correlation = 1.0

    def solve(self, truncate=None, decimate=True, plotting=True, save_params=False):
        """Main solving method for dynamic parameter identification.
        
        Args:
            truncate: None for no truncation, or tuple/list (start, end) 
                     for custom truncation indices
            decimate: Whether to apply decimation to reduce data size
            plotting: Whether to show plots
            save_params: Whether to save parameters to file
        """
        print(f"Starting {self.__class__.__name__} dynamic parameter identification...")
        
        self.process_data(truncate=truncate)
        self.calc_full_regressor()
        self.calc_baseparam(decimate=decimate, plotting=plotting, save_params=save_params)
        
        print(f"Dynamic identification completed")
        print(f"RMS error: {self.rms_error:.6f}")
        print(f"Correlation: {self.correlation:.4f}")
        print(f"Condition number: {self.result['condition number']:.2e}")
        print(f"{len(self.params_base)} base parameters identified")
        
        return self.phi_base
    
    def plot_results(self):
        """Plot identification results."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to plot. Run solve() first.")
            return
        
        import matplotlib.pyplot as plt
        
        # Plot torque comparison
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(self.tau_noised, label="Measured (with noise)", alpha=0.7)
        plt.plot(self.tau_identif, label="Identified", alpha=0.7)
        plt.xlabel('Sample')
        plt.ylabel('Torque (Nm)')
        plt.title(f'{self.__class__.__name__} Torque Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot parameter values
        plt.subplot(2, 1, 2)
        plt.bar(range(len(self.phi_base)), self.phi_base, alpha=0.7, label="Base Parameters")
        plt.xlabel('Parameter Index')
        plt.ylabel('Parameter Value')
        plt.title('Identified Base Parameters')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save identification results to files."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to save. Run solve() first.")
            return
        
        import os
        import yaml
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save identified parameters
        results_dict = {
            'base_parameters': self.phi_base.tolist(),
            'parameter_names': [str(p) for p in self.params_base],
            'rms_error': float(self.rms_error),
            'correlation': float(self.correlation),
            'condition_number': float(self.result['condition number']),
            'standard_deviation': self.result['std dev of estimated param'].tolist() if hasattr(self.result['std dev of estimated param'], 'tolist') else self.result['std dev of estimated param']
        }
        
        robot_name = self.__class__.__name__.lower().replace('identification', '')
        filename = f"{robot_name}_identification_results.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")
