"""
Centralized configuration management for FIGAROH examples.

This module provides unified configuration loading, validation, and management
across different robot implementations.
"""

import os
import yaml
import json
from typing import Dict, Any, Optional, List
from pathlib import Path
import logging
from dataclasses import dataclass, field as dataclass_field

logger = logging.getLogger(__name__)


@dataclass
class ConfigSchema:
    """Configuration schema definition for validation."""
    required_fields: List[str] = dataclass_field(default_factory=list)
    optional_fields: List[str] = dataclass_field(default_factory=list)
    field_types: Dict[str, type] = dataclass_field(default_factory=dict)
    nested_schemas: Dict[str, 'ConfigSchema'] = dataclass_field(
        default_factory=dict
    )


class ConfigurationError(Exception):
    """Custom exception for configuration-related errors."""
    pass


class ConfigManager:
    """Centralized configuration manager for robot examples."""
    
    # Default configuration schemas for different robot types
    ROBOT_SCHEMAS = {
        'tx40': ConfigSchema(
            required_fields=['robot_name', 'active_joints', 'NbSample'],
            optional_fields=['reduction_ratio', 'kmotor', 'motor_offsets'],
            field_types={'NbSample': int, 'active_joints': list}
        ),
        'staubli_tx40': ConfigSchema(
            required_fields=['robot_name', 'active_joints', 'NbSample'],
            optional_fields=['reduction_ratio', 'kmotor', 'motor_offsets'],
            field_types={'NbSample': int, 'active_joints': list}
        ),
        'ur10': ConfigSchema(
            required_fields=['robot_name', 'NbSample', 'config_idx'],
            optional_fields=['outlier_threshold', 'regularization'],
            field_types={'NbSample': int, 'config_idx': list}
        ),
        'talos': ConfigSchema(
            required_fields=['robot_name', 'NbSample', 'NbMarkers'],
            optional_fields=['coeff_regularize', 'known_baseframe'],
            field_types={'NbSample': int, 'NbMarkers': int}
        ),
        'tiago': ConfigSchema(
            required_fields=['robot_name', 'NbSample'],
            optional_fields=['reduction_ratio', 'kmotor'],
            field_types={'NbSample': int}
        ),
        'mate': ConfigSchema(
            required_fields=['robot_name', 'NbSample'],
            optional_fields=['coeff_regularize'],
            field_types={'NbSample': int}
        )
    }
    
    @staticmethod
    def load_robot_config(
        robot_type: str,
        config_name: Optional[str] = None,
        config_dir: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Load robot configuration with validation.
        
        Args:
            robot_type: Type of robot (tx40, ur10, talos, etc.)
            config_name: Optional specific config file name
            config_dir: Optional config directory override
            
        Returns:
            Validated configuration dictionary
            
        Raises:
            ConfigurationError: If config is invalid or not found
        """
        config_path = ConfigManager._get_config_path(
            robot_type, config_name, config_dir
        )
        
        try:
            config = ConfigManager._load_config_file(config_path)
            ConfigManager._validate_config(config, robot_type)
            return config
        except Exception as e:
            raise ConfigurationError(
                f"Failed to load config for {robot_type}: {e}"
            ) from e
    
    @staticmethod
    def _get_config_path(
        robot_type: str,
        config_name: Optional[str] = None,
        config_dir: Optional[str] = None
    ) -> Path:
        """Get the configuration file path."""
        if config_dir:
            base_dir = Path(config_dir)
        else:
            # Default: look in robot's config directory
            examples_dir = Path(__file__).parent.parent
            base_dir = examples_dir / robot_type / "config"
        
        if config_name:
            config_file = config_name
        else:
            # Default config file naming convention
            config_file = f"{robot_type}_config.yaml"
        
        config_path = base_dir / config_file
        
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        return config_path
    
    @staticmethod
    def _load_config_file(config_path: Path) -> Dict[str, Any]:
        """Load configuration from file (YAML or JSON)."""
        suffix = config_path.suffix.lower()
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                if suffix in ['.yaml', '.yml']:
                    return yaml.safe_load(f) or {}
                elif suffix == '.json':
                    return json.load(f) or {}
                else:
                    raise ValueError(f"Unsupported config format: {suffix}")
        except Exception as e:
            raise ConfigurationError(
                f"Failed to parse config file {config_path}: {e}"
            ) from e
    
    @staticmethod
    def _validate_config(config: Dict[str, Any], robot_type: str) -> None:
        """Validate configuration against schema."""
        if robot_type not in ConfigManager.ROBOT_SCHEMAS:
            logger.warning(f"No schema defined for robot type: {robot_type}")
            return
        
        schema = ConfigManager.ROBOT_SCHEMAS[robot_type]
        
        # Check required fields
        missing_fields = []
        for field in schema.required_fields:
            if field not in config:
                missing_fields.append(field)
        
        if missing_fields:
            raise ConfigurationError(
                f"Missing required fields: {missing_fields}"
            )
        
        # Check field types
        for field, expected_type in schema.field_types.items():
            if field in config:
                value = config[field]
                if not isinstance(value, expected_type):
                    raise ConfigurationError(
                        f"Field '{field}' should be {expected_type.__name__}, "
                        f"got {type(value).__name__}"
                    )
    
    @staticmethod
    def get_default_config_path(robot_type: str) -> str:
        """Get default configuration file path for robot type."""
        examples_dir = Path(__file__).parent.parent
        config_path = examples_dir / robot_type / "config"
        return str(config_path / f"{robot_type}_config.yaml")
    
    @staticmethod
    def create_default_config(robot_type: str, output_path: str) -> None:
        """Create a default configuration file for a robot type."""
        default_configs = {
            'tx40': {
                'robot_name': 'Staubli TX40',
                'active_joints': [0, 1, 2, 3, 4, 5],
                'NbSample': 100,
                'reduction_ratio': {
                    'joint_1': 100, 'joint_2': 100, 'joint_3': 100,
                    'joint_4': 100, 'joint_5': 100, 'joint_6': 100
                },
                'motor_offsets': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'ur10': {
                'robot_name': 'Universal Robots UR10',
                'NbSample': 100,
                'config_idx': [0, 1, 2, 3, 4, 5],
                'outlier_threshold': 3.0,
                'regularization': 1e-6
            },
            'talos': {
                'robot_name': 'PAL Robotics TALOS',
                'NbSample': 50,
                'NbMarkers': 1,
                'coeff_regularize': 1e-3,
                'known_baseframe': False,
                'known_tipframe': False
            }
        }
        
        if robot_type not in default_configs:
            raise ValueError(f"No default config available for {robot_type}")
        
        config = default_configs[robot_type]
        
        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        
        msg = f"Created default config for {robot_type} at {output_path}"
        logger.info(msg)
    
    @staticmethod
    def merge_configs(
        base_config: Dict[str, Any],
        override_config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Merge two configuration dictionaries."""
        merged = base_config.copy()
        
        for key, value in override_config.items():
            if (key in merged and isinstance(merged[key], dict)
                    and isinstance(value, dict)):
                merged[key] = ConfigManager.merge_configs(merged[key], value)
            else:
                merged[key] = value
        
        return merged
    
    @staticmethod
    def validate_file_paths(
        config: Dict[str, Any], base_dir: str
    ) -> Dict[str, Any]:
        """Validate and resolve file paths in configuration."""
        validated_config = config.copy()
        
        # Common file path fields to validate
        path_fields = ['urdf_file', 'data_file', 'output_dir', 'model_path']
        
        for field in path_fields:
            if field in validated_config:
                path = validated_config[field]
                if not os.path.isabs(path):
                    # Convert relative path to absolute
                    validated_config[field] = os.path.join(base_dir, path)
                
                # Check if file/directory exists for certain fields
                if field in ['urdf_file', 'data_file']:
                    if not os.path.exists(validated_config[field]):
                        raise ConfigurationError(
                            f"File not found: {validated_config[field]}"
                        )
        
        return validated_config


# Convenience functions for backward compatibility
def load_robot_config(
    robot_type: str, config_name: str = None
) -> Dict[str, Any]:
    """Load robot configuration (convenience function)."""
    return ConfigManager.load_robot_config(robot_type, config_name)


def get_default_config_path(robot_type: str) -> str:
    """Get default config path (convenience function)."""
    return ConfigManager.get_default_config_path(robot_type)
