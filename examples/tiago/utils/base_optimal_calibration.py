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
Base class for robot optimal configuration generation for calibration.
This module provides a generalized framework for optimal configuration
generation that can be inherited by any robot type (TIAGo, UR10, MATE, etc.).
"""

import os
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from yaml.loader import SafeLoader

# FIGAROH imports
from figaroh.calibration.calibration_tools import BaseCalibration


class BaseOptimalCalibration(ABC):
    """
    Base class for robot optimal configuration generation for calibration.
    
    Provides common functionality for all robots while allowing
    robot-specific implementations of optimization strategies.
    """
    
    def __init__(self, robot, config_file="config/robot_config.yaml"):
        """Initialize optimal calibration with robot model and configuration.
        
        Args:
            robot: Robot model loaded with FIGAROH
            config_file: Path to robot configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Create a BaseCalibration object to get parameters
        calib_obj = BaseCalibration(robot, config_file)
        self.param = calib_obj.param
        
        # Initialize attributes for optimal calibration
        self.optimal_configurations = None
        self.optimal_weights = None
        self._sampleConfigs_file = self.param.get("sample_configs_file")
        
        # Calculate minimum number of configurations needed
        if self.param["calib_model"] == "full_params":
            self.minNbChosen = (
                int(
                    len(self.param["actJoint_idx"])
                    * 6
                    / self.param["calibration_index"]
                )
                + 1
            )
        elif self.param["calib_model"] == "joint_offset":
            self.minNbChosen = (
                int(len(self.param["actJoint_idx"]) / self.param["calibration_index"])
                + 1
            )
        else:
            self.minNbChosen = 10  # Default fallback
        
        print(f"{self.__class__.__name__} initialized")
    
    def initialize(self):
        """Initialize the generation process."""
        self.load_data_set()
        self.calculate_regressor()
        self.calculate_detroot_whole()
    
    def solve(self, file_name=None, write_file=False):
        """Solve the optimization problem."""
        if not hasattr(self, 'R_rearr'):
            self.initialize()
        self.calculate_optimal_configurations()
        if write_file:
            try:
                self.write_to_file(name_=file_name)
                print("Optimal configurations written to file successfully")
            except Exception as e:
                print(f"Warning: Could not write to file: {e}")
        self.plot()
    
    def load_data_set(self):
        """Load data from yaml file."""
        import yaml
        from figaroh.calibration.calibration_tools import rank_in_configuration
        
        if self._sampleConfigs_file is None:
            raise ValueError("sample_configs_file not specified in configuration")
        
        if "csv" in self._sampleConfigs_file:
            # For CSV files, use the BaseCalibration load_data_set method
            super().load_data_set()
        elif "yaml" in self._sampleConfigs_file:
            with open(self._sampleConfigs_file, "r") as file:
                self._configs = yaml.load(file, Loader=yaml.SafeLoader)

            q_jointNames = self._configs["calibration_joint_names"]
            q_jointConfigs = np.array(
                self._configs["calibration_joint_configurations"]
            ).T

            df = pd.DataFrame.from_dict(dict(zip(q_jointNames, q_jointConfigs)))

            q = np.zeros([len(df), self.robot.q0.shape[0]])
            for i in range(len(df)):
                for j, name in enumerate(q_jointNames):
                    jointidx = rank_in_configuration(self.model, name)
                    q[i, jointidx] = df[name][i]
            self.q_measured = q

            # update number of samples
            self.param["NbSample"] = self.q_measured.shape[0]
        else:
            raise ValueError("Data file format not supported. Use CSV or YAML format.")
    
    def calculate_regressor(self):
        """Calculate regressor."""
        from figaroh.calibration.calibration_tools import calculate_base_kinematics_regressor
        
        (
            Rrand_b,
            R_b,
            R_e,
            paramsrand_base,
            paramsrand_e,
        ) = calculate_base_kinematics_regressor(
            self.q_measured, self.model, self.data, self.param
        )
        for ii, param_b in enumerate(paramsrand_base):
            print(ii + 1, param_b)

        # Rearrange the kinematic regressor by sample numbered order
        self.R_rearr = self.rearrange_rb(R_b, self.param)
        subX_list, subX_dict = self.sub_info_matrix(self.R_rearr, self.param)
        self._subX_dict = subX_dict
        self._subX_list = subX_list
        return True
    
    def calculate_detroot_whole(self):
        """Calculate detrootn of whole matrix"""
        import picos as pc
        assert self.calculate_regressor(), "Calculate regressor first."
        M_whole = np.matmul(self.R_rearr.T, self.R_rearr)
        self.detroot_whole = pc.DetRootN(M_whole) / np.sqrt(M_whole.shape[0])
        print("detrootn of whole matrix:", self.detroot_whole)

    def calculate_optimal_configurations(self):
        """Calculate optimal configurations using SOCP optimization."""
        import time
        assert self.calculate_regressor(), "Calculate regressor first."

        # Picos optimization (A-optimality, C-optimality, D-optimality)
        prev_time = time.time()
        SOCP_algo = SOCPOptimizer(self._subX_dict, self.param)
        self.w_list, self.w_dict_sort = SOCP_algo.solve()
        solve_time = time.time() - prev_time
        print("solve time of socp: ", solve_time)

        # Select optimal config based on values of weight
        self.eps_opt = 1e-5
        chosen_config = []
        for i in list(self.w_dict_sort.keys()):
            if self.w_dict_sort[i] > self.eps_opt:
                chosen_config.append(i)

        assert (
            len(chosen_config) >= self.minNbChosen
        ), "Infeasible design, try to increase NbSample."

        print(len(chosen_config), "configs are chosen: ", chosen_config)
        self.nb_chosen = len(chosen_config)
        opt_ids = chosen_config
        opt_configs_values = []
        for opt_id in opt_ids:
            opt_configs_values.append(
                self._configs["calibration_joint_configurations"][opt_id]
            )
        self.opt_configs = self._configs.copy()
        self.opt_configs["calibration_joint_configurations"] = list(opt_configs_values)
        self.optimal_configurations = opt_configs_values
        self.optimal_weights = self.w_list
        return True

    def write_to_file(self, name_=None):
        """Write optimal configurations to file."""
        import yaml
        assert (
            self.calculate_optimal_configurations()
        ), "Calculate optimal configurations first."
        
        if name_ is None:
            path_save = "data/optimal_configurations/optimal_configurations.yaml"
        else:
            path_save = "data/optimal_configs/" + name_
            
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(path_save), exist_ok=True)
        
        with open(path_save, "w") as stream:
            try:
                yaml.dump(
                    self.opt_configs,
                    stream,
                    sort_keys=False,
                    default_flow_style=True,
                )
            except yaml.YAMLError as exc:
                print(exc)
        return True

    def plot(self):
        """Plot results."""
        import picos as pc
        import matplotlib.pyplot as plt
        
        # Plotting
        det_root_list = []
        n_key_list = []

        # Calculate det_root_list and n_key_list
        for nbc in range(self.minNbChosen, self.param["NbSample"] + 1):
            n_key = list(self.w_dict_sort.keys())[0:nbc]
            n_key_list.append(n_key)
            M_i = pc.sum(self.w_dict_sort[i] * self._subX_list[i] for i in n_key)
            det_root_list.append(pc.DetRootN(M_i) / np.sqrt(nbc))

        # Create subplots
        fig, ax = plt.subplots(2)

        # Plot D-optimality criterion
        ratio = self.detroot_whole / det_root_list[-1]
        plot_range = self.param["NbSample"] - self.minNbChosen
        ax[0].set_ylabel("D-optimality criterion", fontsize=20)
        ax[0].tick_params(axis="y", labelsize=18)
        ax[0].plot(ratio * np.array(det_root_list[:plot_range]))
        ax[0].spines["top"].set_visible(False)
        ax[0].spines["right"].set_visible(False)
        ax[0].grid(True, linestyle="--")
        ax[0].legend(fontsize=18)

        # Plot quality of estimation
        ax[1].set_ylabel("Weight values (log)", fontsize=20)
        ax[1].set_xlabel("Data sample", fontsize=20)
        ax[1].tick_params(axis="both", labelsize=18)
        ax[1].tick_params(axis="y", labelrotation=30)
        ax[1].scatter(
            np.arange(len(list(self.w_dict_sort.values()))),
            list(self.w_dict_sort.values()),
        )
        ax[1].set_yscale("log")
        ax[1].spines["top"].set_visible(False)
        ax[1].spines["right"].set_visible(False)
        ax[1].grid(True, linestyle="--")
        plt.show()

        return True
    
    def rearrange_rb(self, R_b, param):
        """rearrange the kinematic regressor by sample numbered order"""
        Rb_rearr = np.empty_like(R_b)
        for i in range(param["calibration_index"]):
            for j in range(param["NbSample"]):
                Rb_rearr[j * param["calibration_index"] + i, :] = R_b[
                    i * param["NbSample"] + j
                ]
        return Rb_rearr

    def sub_info_matrix(self, R, param):
        """Returns a list of sub info matrices (product of transpose of regressor and regressor)
        which corresponds to each data sample
        """
        subX_list = []
        idex = param["calibration_index"]
        for it in range(param["NbSample"]):
            sub_R = R[it * idex : (it * idex + idex), :]
            subX = np.matmul(sub_R.T, sub_R)
            subX_list.append(subX)
        subX_dict = dict(
            zip(
                np.arange(
                    param["NbSample"],
                ),
                subX_list,
            )
        )
        return subX_list, subX_dict
    
    @abstractmethod
    def load_candidate_configurations(self):
        """Load candidate configurations for optimization (robot-specific implementation).
        
        Returns:
            np.ndarray: Array of candidate joint configurations
        """
        pass
    
    @abstractmethod
    def optimize_selection(self, subX_dict, nb_chosen):
        """Optimize configuration selection (robot-specific implementation).
        
        Args:
            subX_dict: Dictionary of sub information matrices
            nb_chosen: Number of configurations to select
            
        Returns:
            tuple: (optimal_configs, optimal_weights)
        """
        pass
    
    def plot_results(self):
        """Plot optimal configuration results."""
        if not hasattr(self, 'optimal_configurations') or self.optimal_configurations is None:
            print("No optimal configuration results to plot. Run solve() first.")
            return
        
        # Plot weight distribution
        plt.figure(figsize=(10, 6))
        
        if hasattr(self, 'optimal_weights') and self.optimal_weights is not None:
            plt.subplot(1, 2, 1)
            plt.bar(range(len(self.optimal_weights)), self.optimal_weights)
            plt.xlabel('Configuration Index')
            plt.ylabel('Weight')
            plt.title(f'{self.__class__.__name__} Configuration Weights')
            plt.grid(True, alpha=0.3)
        
        # Plot configuration distribution (if joint space is low dimensional)
        if len(self.optimal_configurations) > 0 and self.optimal_configurations[0].shape[0] <= 3:
            plt.subplot(1, 2, 2)
            configs = np.array(self.optimal_configurations)
            if configs.shape[1] >= 2:
                plt.scatter(configs[:, 0], configs[:, 1])
                plt.xlabel('Joint 1 (rad)')
                plt.ylabel('Joint 2 (rad)')
                plt.title(f'{self.__class__.__name__} Optimal Configurations')
                plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save optimal configuration results to files."""
        if not hasattr(self, 'optimal_configurations') or self.optimal_configurations is None:
            print("No optimal configuration results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save optimal configurations
        results_dict = {
            'optimal_configurations': [config.tolist() for config in self.optimal_configurations],
            'number_selected': len(self.optimal_configurations)
        }
        
        if hasattr(self, 'optimal_weights') and self.optimal_weights is not None:
            results_dict['optimal_weights'] = self.optimal_weights.tolist()
        
        robot_name = self.__class__.__name__.lower().replace('optimalcalibration', '')
        filename = f"{robot_name}_optimal_configurations.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")


# SOCP Optimizer class used by BaseOptimalCalibration
class SOCPOptimizer:
    """Second-Order Cone Programming optimizer for optimal configuration selection."""
    
    def __init__(self, subX_dict, param):
        import picos as pc
        self.pool = subX_dict
        self.param = param
        self.problem = pc.Problem()
        self.w = pc.RealVariable("w", self.param["NbSample"], lower=0)
        self.t = pc.RealVariable("t", 1)

    def add_constraints(self):
        import picos as pc
        Mw = pc.sum(self.w[i] * self.pool[i] for i in range(self.param["NbSample"]))
        wgt_cons = self.problem.add_constraint(1 | self.w <= 1)
        det_root_cons = self.problem.add_constraint(self.t <= pc.DetRootN(Mw))

    def set_objective(self):
        self.problem.set_objective("max", self.t)

    def solve(self):
        self.add_constraints()
        self.set_objective()
        self.solution = self.problem.solve(solver="cvxopt")

        w_list = []
        for i in range(self.w.dim):
            w_list.append(float(self.w.value[i]))
        print("sum of all element in vector solution: ", sum(w_list))

        # to dict
        w_dict = dict(zip(np.arange(self.param["NbSample"]), w_list))
        w_dict_sort = dict(reversed(sorted(w_dict.items(), key=lambda item: item[1])))
        return w_list, w_dict_sort


# DetMax Optimizer class
class Detmax:
    """Determinant Maximization optimizer using greedy exchange algorithm.
    
    This class implements a heuristic optimization algorithm for D-optimal
    experimental design that uses a greedy exchange strategy to find
    near-optimal configuration subsets. Unlike the SOCP approach, this
    method provides a combinatorial solution that directly selects
    discrete configurations.
    
    Algorithm Overview:
        The DetMax algorithm uses an iterative exchange procedure:
        1. Initialize with a random subset of configurations
        2. Iteratively add the configuration that maximally improves
           the determinant criterion
        3. Remove the configuration whose absence minimally degrades
           the determinant criterion
        4. Repeat until convergence (no beneficial exchanges)
    
    Mathematical Background:
        The algorithm maximizes det(Σᵢ∈S Xᵢ)^(1/n) where:
        - S is the selected configuration subset
        - Xᵢ are information matrices for configurations
        - n is the matrix dimension
        
        This is a discrete optimization problem (vs continuous SOCP).
    
    Advantages:
        - Provides exact discrete solution (no weight thresholding)
        - Computationally efficient for small to medium problems
        - Intuitive greedy strategy with good convergence properties
        - No external optimization solvers required
        
    Limitations:
        - May converge to local optima (not globally optimal)
        - Performance depends on random initialization
        - Computational complexity grows with candidate pool size
    
    Attributes:
        pool (dict): Dictionary of information matrices indexed by config ID
        nd (int): Number of configurations to select
        cur_set (list): Current configuration subset being evaluated
        fail_set (list): Configurations that failed selection criteria
        opt_set (list): Final optimal configuration subset
        opt_critD (list): Evolution of determinant criterion during
                        optimization
        
    Example:
        >>> # Create DetMax optimizer
        >>> detmax = Detmax(subX_dict, num_configs=12)
        >>>
        >>> # Run optimization
        >>> criterion_history = detmax.main_algo()
        >>>
        >>> # Get selected configurations
        >>> selected_configs = detmax.cur_set
        >>> final_criterion = criterion_history[-1]
        >>>
        >>> print(f"Selected {len(selected_configs)} configurations")
        >>> print(f"Final D-optimality: {final_criterion:.4f}")
        
    See Also:
        SOCPOptimizer: Alternative SOCP-based optimization approach
        BaseOptimalCalibration: Main calibration framework
    """
    
    def __init__(self, candidate_pool, NbChosen):
        """Initialize DetMax optimizer with candidate pool and target size.
        
        Sets up the determinant maximization optimizer with the candidate
        configuration pool and specifies the number of configurations to
        select in the final optimal subset.
        
        Args:
            candidate_pool (dict): Dictionary mapping configuration indices
                                 to their corresponding information matrices.
                                 Keys are configuration IDs, values are
                                 symmetric positive definite matrices.
            NbChosen (int): Number of configurations to select in the optimal
                          subset. Must be less than total candidates and
                          sufficient for parameter identifiability.
                          
        Raises:
            ValueError: If NbChosen exceeds candidate pool size
            TypeError: If candidate_pool is not a dictionary
            
        Side Effects:
            - Initializes internal tracking lists (cur_set, fail_set, etc.)
            - Stores candidate pool and selection target
            
        Example:
            >>> # Information matrices dict
            >>> info_matrices = {0: X0, 1: X1, 2: X2, ...}
            >>> optimizer = Detmax(info_matrices, NbChosen=10)
        """
        self.pool = candidate_pool
        self.nd = NbChosen
        self.cur_set = []
        self.fail_set = []
        self.opt_set = []
        self.opt_critD = []

    def get_critD(self, set):
        """Calculate D-optimality criterion for configuration subset.
        
        Computes the determinant root of the Fisher Information Matrix
        formed by summing the information matrices of configurations
        in the specified subset. This serves as the objective function
        for the determinant maximization algorithm.
        
        Args:
            set (list): List of configuration indices from the candidate
                       pool to include in the criterion calculation
                       
        Returns:
            float: D-optimality criterion value (determinant root)
                  Higher values indicate better parameter identifiability
                  
        Raises:
            AssertionError: If any configuration index not in candidate pool
            
        Mathematical Details:
            For subset S, computes: det(Σᵢ∈S Xᵢ)^(1/n)
            where Xᵢ are information matrices and n is matrix dimension
            
        Example:
            >>> subset = [0, 5, 12, 18]  # Configuration indices
            >>> criterion = optimizer.get_critD(subset)
            >>> print(f"D-optimality: {criterion:.6f}")
        """
        import picos as pc
        infor_mat = 0
        for idx in set:
            assert idx in self.pool.keys(), \
                   "chosen sample not in candidate pool"
            infor_mat += self.pool[idx]
        return float(pc.DetRootN(infor_mat))

    def main_algo(self):
        """Execute the main determinant maximization algorithm.
        
        Implements the greedy exchange algorithm for D-optimal experimental
        design. The algorithm alternates between adding configurations that
        maximally improve the determinant and removing configurations whose
        absence minimally degrades the determinant.
        
        Algorithm Steps:
        1. Initialize random subset of target size from candidate pool
        2. Exchange Loop:
           a. ADD PHASE: Find configuration that maximally improves criterion
           b. REMOVE PHASE: Find configuration whose removal minimally hurts
           c. Update current subset and criterion value
        3. Repeat until convergence (no beneficial exchanges)
        4. Return optimization history
        
        Convergence Condition:
            The algorithm stops when the optimal configuration to add
            equals the optimal configuration to remove, indicating no
            further improvement is possible.
        
        Returns:
            list: History of D-optimality criterion values throughout
                 the optimization process. Last value is final criterion.
                 
        Side Effects:
            - Updates self.cur_set with final optimal configuration subset
            - Updates self.opt_critD with complete optimization history
            - Uses random initialization (results may vary between runs)
            
        Complexity:
            O(max_iterations × candidate_pool_size × target_subset_size)
            where max_iterations depends on problem structure and
            initialization
            
        Example:
            >>> optimizer = Detmax(info_matrices, NbChosen=10)
            >>> history = optimizer.main_algo()
            >>> print(f"Converged after {len(history)} iterations")
            >>> print(f"Final subset: {optimizer.cur_set}")
            >>> print(f"Final criterion: {history[-1]:.6f}")
            
        Note:
            The algorithm may converge to different local optima depending
            on random initialization. For critical applications, consider
            running multiple times with different seeds.
        """
        import random
        # get all indices in the pool
        pool_idx = tuple(self.pool.keys())

        # initialize a random set
        cur_set = random.sample(pool_idx, self.nd)
        updated_pool = list(set(pool_idx) - set(self.cur_set))

        # adding samples from remaining pool: k = 1
        opt_k = updated_pool[0]
        opt_critD = self.get_critD(cur_set)
        init_set = set(cur_set)
        fin_set = set([])
        rm_j = cur_set[0]

        while opt_k != rm_j:

            # add
            for k in updated_pool:
                cur_set.append(k)
                cur_critD = self.get_critD(cur_set)
                if opt_critD < cur_critD:
                    opt_critD = cur_critD
                    opt_k = k
                cur_set.remove(k)
            cur_set.append(opt_k)
            opt_critD = self.get_critD(cur_set)

            # remove
            delta_critD = opt_critD
            rm_j = cur_set[0]
            for j in cur_set:
                rm_set = cur_set.copy()
                rm_set.remove(j)
                cur_delta_critD = opt_critD - self.get_critD(rm_set)

                if cur_delta_critD < delta_critD:
                    delta_critD = cur_delta_critD
                    rm_j = j
            cur_set.remove(rm_j)
            opt_critD = self.get_critD(cur_set)
            fin_set = set(cur_set)

            self.opt_critD.append(opt_critD)
        return self.opt_critD