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
Configuration Manager for FIGAROH Framework

This module implements an enhanced configuration architecture supporting:
- Task inheritance (optimal_configuration inherits from calibration)
- Four main tasks: calibration, optimal_configuration, identification,
  optimal_trajectory
- Common parameters shared across all tasks
- Full type safety and validation
- Simplified configuration structure focused on core parameters
"""

import yaml
import logging
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from enum import Enum
from yaml.loader import SafeLoader
import copy


class TaskType(Enum):
    """Supported task types."""
    CALIBRATION = "calibration"
    OPTIMAL_CONFIGURATION = "optimal_configuration"
    IDENTIFICATION = "identification"
    OPTIMAL_TRAJECTORY = "optimal_trajectory"


@dataclass
class OutputConfig:
    """Configuration for output settings."""
    save_configs: bool = True
    config_file: Optional[str] = None
    save_trajectory: bool = True
    trajectory_file: Optional[str] = None
    save_metrics: bool = True
    metrics_file: Optional[str] = None
    visualization: bool = True
    animation: bool = False
    plot_metrics: bool = True
    plot_joints: bool = True
    plot_cartesian: bool = True


@dataclass
class LoggingConfig:
    """Configuration for logging settings."""
    level: str = "INFO"
    log_file: Optional[str] = None
    console_output: bool = True
    detailed_timing: bool = False
    
    def __post_init__(self):
        """Validate logging configuration."""
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR"]
        if self.level not in valid_levels:
            raise ValueError(
                f"Invalid log level. Must be one of {valid_levels}"
            )


@dataclass
class DataConfig:
    """Configuration for data management."""
    base_directory: str = "data/"
    backup_results: bool = True


@dataclass
class CommonConfig:
    """Common configuration shared across all tasks."""
    robot_name: str = "tiago"
    robot_description: str = "tiago_description"
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    data: DataConfig = field(default_factory=DataConfig)


class ConfigurationManager:
    """
    Enhanced Configuration Manager supporting task inheritance and modularity.
    
    Features:
    - Task inheritance (optimal_configuration inherits from calibration)
    - Four main tasks with specific configurations
    - Common parameters shared across tasks
    - Type safety and validation
    - Flexible parameter overrides
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """Initialize configuration manager.
        
        Args:
            config_file: Path to YAML configuration file
        """
        self.logger = logging.getLogger(__name__)
        self.config_file = config_file
        self.config = self._load_config(config_file) if config_file else {}
        self.runtime_overrides = {}
        self._resolved_configs = {}  # Cache for resolved configurations
    
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            with open(config_file, "r") as f:
                config = yaml.load(f, Loader=SafeLoader)
            
            self.logger.info(f"Loaded configuration from {config_file}")
            return config
            
        except FileNotFoundError:
            raise FileNotFoundError(
                f"Configuration file {config_file} not found"
            )
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file {config_file}: {e}")
        except Exception as e:
            raise ValueError(f"Error loading configuration: {e}")
    
    def _resolve_inheritance(self, task_name: str) -> Dict[str, Any]:
        """Resolve configuration inheritance for a task.
        
        Args:
            task_name: Name of the task to resolve
            
        Returns:
            Resolved configuration with inheritance applied
        """
        if task_name in self._resolved_configs:
            return self._resolved_configs[task_name]
        
        if task_name not in self.config:
            raise ValueError(f"Task '{task_name}' not found in configuration")
        
        task_config = copy.deepcopy(self.config[task_name])
        
        # Check if this task inherits from another
        if "inherit_from" in task_config:
            parent_task = task_config.pop("inherit_from")
            
            # Recursively resolve parent configuration
            parent_config = self._resolve_inheritance(parent_task)
            
            # Merge parent config with current task config
            resolved_config = self._deep_merge(parent_config, task_config)
        else:
            resolved_config = task_config
        
        # Cache the resolved configuration
        self._resolved_configs[task_name] = resolved_config
        return resolved_config
    
    def _deep_merge(self, base: Dict[str, Any],
                    overlay: Dict[str, Any]) -> Dict[str, Any]:
        """Deep merge two dictionaries, with overlay taking precedence.
        
        Args:
            base: Base dictionary
            overlay: Overlay dictionary (takes precedence)
            
        Returns:
            Merged dictionary
        """
        result = copy.deepcopy(base)
        
        for key, value in overlay.items():
            if (key in result and
                    isinstance(result[key], dict) and
                    isinstance(value, dict)):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = copy.deepcopy(value)
        
        return result
    
    def load_task_config(self, task_type: TaskType) -> Dict[str, Any]:
        """Load configuration for a specific task with inheritance resolved.
        
        Args:
            task_type: Type of task to load
            
        Returns:
            Complete task configuration as dictionary
        """
        task_name = task_type.value
        task_config = self._resolve_inheritance(task_name)
        
        # Add common configuration
        if "common" in self.config:
            common_config = self.config["common"]
            task_config = self._deep_merge(
                {"common": common_config}, task_config
            )
        
        # Apply runtime overrides
        if task_name in self.runtime_overrides:
            task_config = self._deep_merge(
                task_config, self.runtime_overrides[task_name]
            )
        
        return task_config
    
    def load_calibration_config(self) -> Dict[str, Any]:
        """Load calibration configuration with robust missing key handling.
        
        Implements pattern similar to get_param_from_yaml from
        calibration_tools.py with comprehensive try/catch blocks
        for optional parameters.
        
        Returns:
            Complete calibration configuration dictionary with all parameters
            properly validated and missing keys assigned default values.
        """
        # Get base configuration using inheritance resolution
        task_config = self.load_task_config(TaskType.CALIBRATION)
        task_name = TaskType.CALIBRATION.value
        
        # Ensure all required calibration parameters are present with defaults
        enhanced_config = copy.deepcopy(task_config)
        
        # Handle core calibration parameters with defaults
        enhanced_config["calib_level"] = self._get_config_value(
            enhanced_config, "calib_level",
            default="full_params", task_name=task_name
        )
        enhanced_config["nb_sample"] = self._get_config_value(
            enhanced_config, "nb_sample", default=100, task_name=task_name
        )
        enhanced_config["base_frame"] = self._get_config_value(
            enhanced_config, "base_frame",
            default="base_link", task_name=task_name
        )
        enhanced_config["tool_frame"] = self._get_config_value(
            enhanced_config, "tool_frame", default="tool0", task_name=task_name
        )
        enhanced_config["free_flyer"] = self._get_config_value(
            enhanced_config, "free_flyer", default=False, task_name=task_name
        )
        enhanced_config["non_geom"] = self._get_config_value(
            enhanced_config, "non_geom", default=False, task_name=task_name
        )
        
        # Handle markers configuration (required for calibration)
        if "markers" not in enhanced_config:
            self.logger.warning(
                f"Missing required 'markers' configuration in {task_name}. "
                f"Auto-assigning default marker configuration."
            )
            enhanced_config["markers"] = [
                {
                    "name": "marker_1",
                    "measure": [True, True, True, True, True, True]
                }
            ]
        
        # Handle eye-hand calibration frames (optional parameters)
        # Pattern: try/catch with group assignment and informative messages
        try:
            base_to_ref_frame = enhanced_config["base_to_ref_frame"]
            ref_frame = enhanced_config["ref_frame"]
            self.logger.info(
                f"Eye-hand calibration frames configured: "
                f"base_to_ref='{base_to_ref_frame}', ref='{ref_frame}'"
            )
        except KeyError:
            enhanced_config["base_to_ref_frame"] = None
            enhanced_config["ref_frame"] = None
            self.logger.info(
                "base_to_ref_frame and ref_frame are not defined."
            )
        
        # Handle initial poses (optional parameters)
        try:
            base_pose = enhanced_config["base_pose"]
            tip_pose = enhanced_config["tip_pose"]
            self.logger.info(
                "Initial poses configured: "
                f"base_pose={bool(base_pose)}, tip_pose={bool(tip_pose)}"
            )
        except KeyError:
            enhanced_config["base_pose"] = None
            enhanced_config["tip_pose"] = None
            self.logger.info("base_pose and tip_pose are not defined.")
        
        # Handle optimization and processing parameters (optional)
        try:
            enhanced_config.update({
                "coeff_regularize": enhanced_config["coeff_regularize"],
                "data_file": enhanced_config["data_file"],
                "sample_configs_file": enhanced_config["sample_configs_file"],
                "outlier_eps": enhanced_config["outlier_eps"],
            })
            self.logger.info("Optional processing parameters configured")
        except KeyError:
            enhanced_config.update({
                "coeff_regularize": None,
                "data_file": None,
                "sample_configs_file": None,
                "outlier_eps": None,
            })
            self.logger.info(
                "Optional processing parameters not defined, using defaults"
            )
        
        # Add fixed parameters (following get_param_from_yaml pattern)
        enhanced_config.update({
            "eps": enhanced_config.get("eps", 1e-3),
            "PLOT": enhanced_config.get("PLOT", 0),
        })
        
        return enhanced_config
    
    def load_optimal_configuration_config(self) -> Dict[str, Any]:
        """Load optimal configuration task configuration.
        
        Inherits from calibration config and adds optimization-specific
        parameters with robust missing key handling.
        
        Returns:
            Complete optimal configuration dictionary with calibration base
            and optimization-specific parameters.
        """
        # Get base configuration (inherits from calibration)
        task_config = self.load_task_config(TaskType.OPTIMAL_CONFIGURATION)
        task_name = TaskType.OPTIMAL_CONFIGURATION.value
        
        # Ensure all required parameters are present with defaults
        enhanced_config = copy.deepcopy(task_config)
        
        # Inherit calibration parameters (optimal_configuration extends
        # calibration)
        # Handle optimization-specific parameters
        enhanced_config["optimization_method"] = self._get_config_value(
            enhanced_config, "optimization_method",
            default="least_squares", task_name=task_name
        )
        enhanced_config["max_iterations"] = self._get_config_value(
            enhanced_config, "max_iterations",
            default=1000, task_name=task_name
        )
        enhanced_config["tolerance"] = self._get_config_value(
            enhanced_config, "tolerance", default=1e-6, task_name=task_name
        )
        
        # Handle optimization bounds (optional parameters)
        try:
            parameter_bounds = enhanced_config["parameter_bounds"]
            self.logger.info(
                f"Parameter bounds configured with "
                f"{len(parameter_bounds)} parameter sets"
            )
        except KeyError:
            enhanced_config["parameter_bounds"] = None
            self.logger.info("parameter_bounds not defined.")
        
        # Handle convergence criteria (optional parameters)
        try:
            enhanced_config.update({
                "gradient_tolerance": enhanced_config["gradient_tolerance"],
                "step_tolerance": enhanced_config["step_tolerance"],
                "function_tolerance": enhanced_config["function_tolerance"],
            })
            self.logger.info("Convergence criteria configured")
        except KeyError:
            enhanced_config.update({
                "gradient_tolerance": None,
                "step_tolerance": None,
                "function_tolerance": None,
            })
            self.logger.info(
                "Convergence criteria not defined, using defaults"
            )
        
        return enhanced_config
    
    def load_identification_config(self) -> Dict[str, Any]:
        """Load identification config with flattened sub-configs."""
        task_config = self.load_task_config(TaskType.IDENTIFICATION)
        task_name = TaskType.IDENTIFICATION.value
        
        # For backward compatibility, flatten sub-configurations
        flattened_config = {}
        
        # First add any top-level parameters
        for key, value in task_config.items():
            sub_keys = [
                'robot_params', 'trajectory_params', 'problem_params',
                'processing_params', 'tls_params'
            ]
            if key not in sub_keys:
                flattened_config[key] = value
        
        # Then merge all sub-configurations at the top level with missing key handling
        sub_config_keys = [
            'robot_params', 'trajectory_params', 'problem_params',
            'processing_params', 'tls_params'
        ]
        
        for sub_key in sub_config_keys:
            if sub_key in task_config and isinstance(task_config[sub_key], dict):
                sub_config = task_config[sub_key]
                for param_key, param_value in sub_config.items():
                    # Check for conflicts and warn if key already exists
                    if param_key in flattened_config:
                        self.logger.warning(
                            f"Key '{param_key}' from '{sub_key}' conflicts with "
                            f"existing key in {task_name}. Using '{sub_key}' value."
                        )
                    flattened_config[param_key] = param_value
            else:
                # Handle missing sub-configuration
                if sub_key not in task_config:
                    self.logger.warning(
                        f"Missing sub-configuration '{sub_key}' in {task_name}. "
                        f"Auto-assigning empty dict."
                    )
                    task_config[sub_key] = {}
        
        return flattened_config
    
    def load_optimal_trajectory_config(self) -> Dict[str, Any]:
        """Load optimal trajectory config with flattened sub-configs."""
        task_config = self.load_task_config(TaskType.OPTIMAL_TRAJECTORY)
        task_name = TaskType.OPTIMAL_TRAJECTORY.value
        
        # For backward compatibility, flatten sub-configurations
        flattened_config = {}
        
        # First add any top-level parameters
        for key, value in task_config.items():
            sub_keys = [
                'robot_params', 'trajectory_params', 'problem_params',
                'processing_params', 'tls_params'
            ]
            if key not in sub_keys:
                flattened_config[key] = value
        
        # Then merge all sub-configurations at the top level with missing key handling
        sub_config_keys = [
            'robot_params', 'trajectory_params', 'problem_params',
            'processing_params', 'tls_params'
        ]
        
        for sub_key in sub_config_keys:
            if (sub_key in task_config and
                    isinstance(task_config[sub_key], dict)):
                sub_config = task_config[sub_key]
                for param_key, param_value in sub_config.items():
                    # Check for conflicts and warn if key already exists
                    if param_key in flattened_config:
                        self.logger.warning(
                            f"Key '{param_key}' from '{sub_key}' conflicts "
                            f"with existing key in {task_name}. "
                            f"Using '{sub_key}' value."
                        )
                    flattened_config[param_key] = param_value
            else:
                # Handle missing sub-configuration
                if sub_key not in task_config:
                    self.logger.warning(
                        f"Missing sub-configuration '{sub_key}' "
                        f"in {task_name}. Auto-assigning empty dict."
                    )
                    task_config[sub_key] = {}
        
        return flattened_config
    
    def get_output_config(self, task_type: TaskType) -> OutputConfig:
        """Get output configuration for a task.
        
        Args:
            task_type: Type of task
            
        Returns:
            Output configuration
        """
        task_config = self.load_task_config(task_type)
        
        if "output" not in task_config:
            return OutputConfig()  # Return defaults
        
        output_data = task_config["output"]
        
        return OutputConfig(
            save_configs=output_data.get("save_configs", True),
            config_file=output_data.get("config_file"),
            save_trajectory=output_data.get("save_trajectory", True),
            trajectory_file=output_data.get("trajectory_file"),
            save_metrics=output_data.get("save_metrics", True),
            metrics_file=output_data.get("metrics_file"),
            visualization=output_data.get("visualization", True),
            animation=output_data.get("animation", False),
            plot_metrics=output_data.get("plot_metrics", True),
            plot_joints=output_data.get("plot_joints", True),
            plot_cartesian=output_data.get("plot_cartesian", True)
        )
    
    def update_task_runtime(self, task_type: TaskType, **kwargs) -> None:
        """Update task configuration at runtime.
        
        Args:
            task_type: Type of task to update
            **kwargs: Configuration parameters to update
        """
        task_name = task_type.value
        
        if task_name not in self.runtime_overrides:
            self.runtime_overrides[task_name] = {}
        
        old_overrides = copy.deepcopy(self.runtime_overrides[task_name])
        
        try:
            self.runtime_overrides[task_name].update(kwargs)
            # Clear cached resolved configs since we changed runtime overrides
            self._resolved_configs.clear()
            self.logger.debug(
                f"Updated runtime configuration for {task_name}: {kwargs}"
            )
        except Exception:
            # Rollback on error
            self.runtime_overrides[task_name] = old_overrides
            raise
    
    def validate_task_config(self, task_type: TaskType) -> None:
        """Validate configuration for a specific task.
        
        Args:
            task_type: Type of task to validate
        """
        try:
            # Use the appropriate loading method for each task type
            if task_type in [TaskType.IDENTIFICATION,
                             TaskType.OPTIMAL_TRAJECTORY]:
                # Use flattened config for identification tasks
                if task_type == TaskType.IDENTIFICATION:
                    task_config = self.load_identification_config()
                else:
                    task_config = self.load_optimal_trajectory_config()
            else:
                # Use regular config for calibration tasks
                task_config = self.load_task_config(task_type)
            
            # Validate output config if present
            if "output" in task_config:
                try:
                    self.get_output_config(task_type)
                except Exception as e:
                    self.logger.warning(
                        f"Output validation failed for {task_type.value}: {e}"
                    )
            
            self.logger.info(
                f"Configuration validation successful for {task_type.value}"
            )
            
        except Exception as e:
            self.logger.error(
                f"Configuration validation failed for {task_type.value}: {e}"
            )
            # Don't re-raise to allow graceful handling of missing keys
    
    def validate_all_configs(self) -> None:
        """Validate all task configurations."""
        for task_type in TaskType:
            try:
                self.validate_task_config(task_type)
            except Exception as e:
                self.logger.warning(
                    f"Validation failed for {task_type.value}: {e}"
                )
    
    def _get_config_value(self, config: Dict[str, Any], key: str,
                          default: Any = None, task_name: str = None) -> Any:
        """Get configuration value with missing key handling.
        
        Args:
            config: Configuration dictionary
            key: Key to retrieve
            default: Default value if key is missing
            task_name: Task name for warning context
            
        Returns:
            Configuration value or default/None if missing
        """
        if key in config:
            return config[key]
        else:
            # Auto-assign None and issue warning
            task_context = f" in {task_name}" if task_name else ""
            self.logger.warning(
                f"Missing configuration key '{key}'{task_context}. "
                f"Auto-assigning default value: {default}"
            )
            return default

# Factory functions for easy instantiation
def create_configuration_manager(
    config_file: str
) -> ConfigurationManager:
    """Create and configure a ConfigurationManager instance.
    
    Args:
        config_file: Path to YAML configuration file
        
    Returns:
        Configured ConfigurationManager instance
    """
    manager = ConfigurationManager(config_file)
    
    # Validate all configurations
    manager.validate_all_configs()
    
    return manager


# Example usage and testing
if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(level=logging.INFO)
    
    # Example usage
    try:
        # Create configuration manager
        config_manager = create_configuration_manager(
            "config/tiago_config_improved.yaml"
        )
        
        # Load different task configurations with error handling
        try:
            calib_config = config_manager.load_calibration_config()
            opt_config_config = config_manager.load_optimal_configuration_config()
            identif_config = config_manager.load_identification_config()
            opt_traj_config = config_manager.load_optimal_trajectory_config()
        except Exception as load_error:
            print(f"Error loading configurations: {load_error}")
            raise
        
        # Print all 4 configuration objects with safe access
        print("\n" + "="*60)
        print("📋 CALIBRATION CONFIG:")
        print("="*60)
        print(f"Keys: {list(calib_config.keys()) if calib_config else []}")
        
        print("\n🎯 OPTIMAL CONFIGURATION CONFIG:")
        print("="*60)
        print(f"Keys: {list(opt_config_config.keys()) if opt_config_config else []}")
        
        print("\n🔍 IDENTIFICATION CONFIG (FLATTENED):")
        print("="*60)
        print(f"Keys: {list(identif_config.keys())}")
        
        print("\n🚀 OPTIMAL TRAJECTORY CONFIG (FLATTENED):")
        print("="*60)
        print(f"Keys: {list(opt_traj_config.keys())}")
        print("="*60)
        
        # Test runtime updates
        config_manager.update_task_runtime(TaskType.CALIBRATION, nb_sample=200)
        updated_calib = config_manager.load_calibration_config()
        updated_samples = updated_calib.get('nb_sample', 'N/A')
        print(f"Updated calibration samples: {updated_samples}")
        
    except Exception as e:
        print(f"Configuration error: {e}")
