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
from typing import List, Optional, Dict, Any
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
class ConstraintsConfig:
    """Configuration for optimization constraints."""
    joint_limits: bool = True
    collision_avoidance: bool = True
    workspace_limits: bool = True
    velocity_limits: bool = True
    acceleration_limits: bool = True
    jerk_limits: bool = True
    minimum_distance: float = 0.1
    maximum_jerk: Optional[float] = None
    trajectory_duration: Optional[List[float]] = None
    
    def __post_init__(self):
        """Validate constraints configuration."""
        if self.minimum_distance < 0:
            raise ValueError("minimum_distance must be non-negative")
        if self.trajectory_duration is not None:
            if len(self.trajectory_duration) != 2:
                raise ValueError(
                    "trajectory_duration must have exactly 2 values [min, max]"
                )


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
        """Load calibration configuration."""
        return self.load_task_config(TaskType.CALIBRATION)
    
    def load_optimal_configuration_config(self) -> Dict[str, Any]:
        """Load optimal configuration task configuration."""
        return self.load_task_config(TaskType.OPTIMAL_CONFIGURATION)
    
    def load_identification_config(self) -> Dict[str, Any]:
        """Load identification config with flattened sub-configs."""
        task_config = self.load_task_config(TaskType.IDENTIFICATION)
        
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
        
        # Then merge all sub-configurations at the top level
        sub_config_keys = [
            'robot_params', 'trajectory_params', 'problem_params',
            'processing_params', 'tls_params'
        ]
        
        for sub_key in sub_config_keys:
            if (sub_key in task_config and
                    isinstance(task_config[sub_key], dict)):
                flattened_config.update(task_config[sub_key])
        
        return flattened_config
    
    def load_optimal_trajectory_config(self) -> Dict[str, Any]:
        """Load optimal trajectory config with flattened sub-configs."""
        task_config = self.load_task_config(TaskType.OPTIMAL_TRAJECTORY)
        
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
        
        # Then merge all sub-configurations at the top level
        sub_config_keys = [
            'robot_params', 'trajectory_params', 'problem_params',
            'processing_params', 'tls_params'
        ]
        
        for sub_key in sub_config_keys:
            if (sub_key in task_config and
                    isinstance(task_config[sub_key], dict)):
                flattened_config.update(task_config[sub_key])
        
        return flattened_config
    
    def load_common_config(self) -> Dict[str, Any]:
        """Load common configuration shared across all tasks."""
        if "common" not in self.config:
            return {}
        return copy.deepcopy(self.config["common"])
    
    def get_constraints_config(self, task_type: TaskType) -> ConstraintsConfig:
        """Get validated constraints configuration for a task.
        
        Args:
            task_type: Type of task
            
        Returns:
            Validated constraints configuration
        """
        task_config = self.load_task_config(task_type)
        
        if "constraints" not in task_config:
            return ConstraintsConfig()  # Return defaults
        
        const_data = task_config["constraints"]
        
        return ConstraintsConfig(
            joint_limits=const_data.get("joint_limits", True),
            collision_avoidance=const_data.get("collision_avoidance", True),
            workspace_limits=const_data.get("workspace_limits", True),
            velocity_limits=const_data.get("velocity_limits", True),
            acceleration_limits=const_data.get("acceleration_limits", True),
            jerk_limits=const_data.get("jerk_limits", True),
            minimum_distance=const_data.get("minimum_distance", 0.1),
            maximum_jerk=const_data.get("maximum_jerk"),
            trajectory_duration=const_data.get("trajectory_duration")
        )
    
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
            task_config = self.load_task_config(task_type)
            
            # Validate constraints config if present
            if "constraints" in task_config:
                self.get_constraints_config(task_type)
            
            # Validate output config if present
            if "output" in task_config:
                self.get_output_config(task_type)
            
            self.logger.info(
                f"Configuration validation successful for {task_type.value}"
            )
            
        except Exception as e:
            self.logger.error(
                f"Configuration validation failed for {task_type.value}: {e}"
            )
            raise
    
    def validate_all_configs(self) -> None:
        """Validate all task configurations."""
        for task_type in TaskType:
            try:
                self.validate_task_config(task_type)
            except Exception as e:
                self.logger.warning(
                    f"Validation failed for {task_type.value}: {e}"
                )
    
    def get_task_legacy_dict(
        self, task_type: TaskType, robot_name: str = None
    ) -> Dict[str, Any]:
        """Convert task config to legacy dictionary format.
        
        Provides backward compatibility with legacy methods.
        
        Args:
            task_type: Type of task
            robot_name: Robot name (uses common config if not provided)
            
        Returns:
            Legacy format dictionary
        """
        task_config = self.load_task_config(task_type)
        
        if robot_name is None:
            common_config = self.load_common_config()
            robot_name = common_config.get("robot_name", "unknown")
        
        # Add robot_name to the config
        legacy_config = copy.deepcopy(task_config)
        legacy_config["robot_name"] = robot_name
        
        return legacy_config
    
    def log_configuration_summary(
        self, task_type: Optional[TaskType] = None
    ) -> None:
        """Log summary of configuration.
        
        Args:
            task_type: Specific task to summarize (all tasks if None)
        """
        self.logger.info("Configuration Summary:")
        self.logger.info(f"  - Config file: {self.config_file}")
        
        if task_type is None:
            # Log summary for all tasks
            for task in TaskType:
                self._log_task_summary(task)
        else:
            self._log_task_summary(task_type)
    
    def _log_task_summary(self, task_type: TaskType) -> None:
        """Log summary for a specific task."""
        try:
            task_config = self.load_task_config(task_type)
            self.logger.info(f"  - Task: {task_type.value}")
            
            # Log inheritance info
            original_config = self.config.get(task_type.value, {})
            if "inherit_from" in original_config:
                inherit_from = original_config['inherit_from']
                self.logger.info(f"    - Inherits from: {inherit_from}")
            
            # Log key parameters based on task type
            calibration_tasks = [
                TaskType.CALIBRATION, 
                TaskType.OPTIMAL_CONFIGURATION
            ]
            if task_type in calibration_tasks:
                if "calib_level" in task_config:
                    level = task_config['calib_level']
                    self.logger.info(f"    - Calibration model: {level}")
                if "nb_sample" in task_config:
                    samples = task_config['nb_sample']
                    self.logger.info(f"    - Samples: {samples}")
            
            identification_tasks = [
                TaskType.IDENTIFICATION, 
                TaskType.OPTIMAL_TRAJECTORY
            ]
            if task_type in identification_tasks:
                if "processing_params" in task_config:
                    ts = task_config["processing_params"].get("ts", 0.0002)
                    self.logger.info(f"    - Sample rate: {1 / ts:.0f} Hz")
                if "problem_params" in task_config:
                    problem_params = task_config["problem_params"]
                    active_joints = problem_params.get("active_joints", [])
                    self.logger.info(f"    - Active joints: {len(active_joints)}")
                
        except Exception as e:
            self.logger.warning(f"  - {task_type.value} config error: {e}")


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
    
    # Log summary
    manager.log_configuration_summary()
    
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
        
        # Load different task configurations
        calib_config = config_manager.load_calibration_config()
        opt_config_config = config_manager.load_optimal_configuration_config()
        identif_config = config_manager.load_identification_config()
        opt_traj_config = config_manager.load_optimal_trajectory_config()
        
        # Print all 4 configuration objects
        print("\n" + "="*60)
        print("📋 CALIBRATION CONFIG:")
        print("="*60)
        print(f"Samples: {calib_config.get('nb_sample', 'N/A')}")
        print(f"Calibration level: {calib_config.get('calib_level', 'N/A')}")
        print(f"Keys: {list(calib_config.keys())}")
        
        print("\n🎯 OPTIMAL CONFIGURATION CONFIG:")
        print("="*60)
        print(f"Samples: {opt_config_config.get('nb_sample', 'N/A')}")
        print(f"Calibration level: {opt_config_config.get('calib_level', 'N/A')}")
        print(f"Objectives: {opt_config_config.get('objectives', 'N/A')}")
        print(f"Keys: {list(opt_config_config.keys())}")
        
        print("\n🔍 IDENTIFICATION CONFIG (FLATTENED):")
        print("="*60)
        print(f"Active joints: {identif_config.get('active_joints', 'N/A')}")
        print(f"Sample frequency: {identif_config.get('freq', 'N/A')} Hz")
        print(f"Sampling time: {identif_config.get('ts', 'N/A')} s")
        print(f"Number of waypoints: {identif_config.get('n_wps', 'N/A')}")
        print(f"Has friction: {identif_config.get('has_friction', 'N/A')}")
        print(f"Total flattened keys: {len(identif_config)}")
        print(f"Keys: {list(identif_config.keys())}")
        
        print("\n🚀 OPTIMAL TRAJECTORY CONFIG (FLATTENED):")
        print("="*60)
        print(f"Active joints: {opt_traj_config.get('active_joints', 'N/A')}")
        print(f"Sample frequency: {opt_traj_config.get('freq', 'N/A')} Hz")
        print(f"Sampling time: {opt_traj_config.get('ts', 'N/A')} s")
        print(f"Number of waypoints: {opt_traj_config.get('n_wps', 'N/A')}")
        print(f"Has friction: {opt_traj_config.get('has_friction', 'N/A')}")
        print(f"Objectives: {opt_traj_config.get('objectives', 'N/A')}")
        print(f"Total flattened keys: {len(opt_traj_config)}")
        print(f"Keys: {list(opt_traj_config.keys())}")
        print("="*60)
        
        # Test runtime updates
        config_manager.update_task_runtime(TaskType.CALIBRATION, nb_sample=200)
        updated_calib = config_manager.load_calibration_config()
        updated_samples = updated_calib.get('nb_sample', 'N/A')
        print(f"Updated calibration samples: {updated_samples}")
        
    except Exception as e:
        print(f"Configuration error: {e}")
