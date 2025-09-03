"""
Test configuration manager functionality.

Tests for the centralized configuration management system.
"""

import pytest
import tempfile
import os
import yaml
from pathlib import Path

# Import using relative path for testing
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from shared.config_manager import (
    ConfigManager,
    ConfigurationError,
    ConfigSchema
)


class TestConfigManager:
    """Test suite for ConfigManager class."""
    
    def test_config_schema_creation(self):
        """Test ConfigSchema dataclass creation."""
        schema = ConfigSchema(
            required_fields=['robot_name'],
            optional_fields=['debug'],
            field_types={'robot_name': str}
        )
        
        assert schema.required_fields == ['robot_name']
        assert schema.optional_fields == ['debug']
        assert schema.field_types == {'robot_name': str}
    
    def test_load_valid_yaml_config(self):
        """Test loading a valid YAML configuration."""
        config_data = {
            'robot_name': 'Test Robot',
            'NbSample': 100,
            'active_joints': [0, 1, 2]
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_data, f)
            temp_path = f.name
        
        try:
            # Test loading with path override
            loaded_config = ConfigManager._load_config_file(Path(temp_path))
            assert loaded_config == config_data
        finally:
            os.unlink(temp_path)
    
    def test_load_invalid_config_file(self):
        """Test loading a non-existent configuration file."""
        with pytest.raises(FileNotFoundError):
            ConfigManager._get_config_path('nonexistent_robot', 'fake_config.yaml')
    
    def test_validate_config_with_schema(self):
        """Test configuration validation against schema."""
        # Test valid configuration
        valid_config = {
            'robot_name': 'TX40',
            'NbSample': 100,
            'active_joints': [0, 1, 2, 3, 4, 5]
        }
        
        # Should not raise exception
        ConfigManager._validate_config(valid_config, 'tx40')
        
        # Test invalid configuration (missing required field)
        invalid_config = {
            'NbSample': 100,
            'active_joints': [0, 1, 2, 3, 4, 5]
        }
        
        with pytest.raises(ConfigurationError):
            ConfigManager._validate_config(invalid_config, 'tx40')
    
    def test_validate_config_wrong_types(self):
        """Test configuration validation with wrong field types."""
        invalid_config = {
            'robot_name': 'TX40',
            'NbSample': 'not_a_number',  # Should be int
            'active_joints': [0, 1, 2, 3, 4, 5]
        }
        
        with pytest.raises(ConfigurationError):
            ConfigManager._validate_config(invalid_config, 'tx40')
    
    def test_merge_configs(self):
        """Test merging configuration dictionaries."""
        base_config = {
            'robot_name': 'Base Robot',
            'settings': {
                'debug': False,
                'verbose': True
            },
            'joints': [0, 1, 2]
        }
        
        override_config = {
            'robot_name': 'Override Robot',
            'settings': {
                'debug': True
            },
            'new_param': 'test'
        }
        
        merged = ConfigManager.merge_configs(base_config, override_config)
        
        assert merged['robot_name'] == 'Override Robot'
        assert merged['settings']['debug'] is True
        assert merged['settings']['verbose'] is True  # Preserved from base
        assert merged['new_param'] == 'test'
        assert merged['joints'] == [0, 1, 2]
    
    def test_validate_file_paths(self):
        """Test file path validation in configuration."""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create a test file
            test_file = os.path.join(temp_dir, 'test.urdf')
            with open(test_file, 'w') as f:
                f.write('test content')
            
            config = {
                'urdf_file': 'test.urdf',  # Relative path
                'robot_name': 'Test Robot'
            }
            
            validated = ConfigManager.validate_file_paths(config, temp_dir)
            
            # Path should be converted to absolute
            assert os.path.isabs(validated['urdf_file'])
            assert os.path.exists(validated['urdf_file'])
    
    def test_create_default_config(self):
        """Test creating default configuration files."""
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = os.path.join(temp_dir, 'tx40_config.yaml')
            
            ConfigManager.create_default_config('tx40', output_path)
            
            assert os.path.exists(output_path)
            
            # Load and verify the created config
            with open(output_path, 'r') as f:
                config = yaml.safe_load(f)
            
            assert config['robot_name'] == 'Staubli TX40'
            assert 'NbSample' in config
            assert 'active_joints' in config
    
    def test_unknown_robot_type(self):
        """Test handling of unknown robot types."""
        config = {'robot_name': 'Unknown Robot'}
        
        # Should not raise exception but log warning
        ConfigManager._validate_config(config, 'unknown_robot')
    
    def test_convenience_functions(self):
        """Test convenience functions for backward compatibility."""
        # Test get_default_config_path
        path = ConfigManager.get_default_config_path('tx40')
        assert 'tx40' in path
        assert path.endswith('tx40_config.yaml')


class TestConfigSchemas:
    """Test predefined configuration schemas."""
    
    def test_tx40_schema(self):
        """Test TX40 configuration schema."""
        schema = ConfigManager.ROBOT_SCHEMAS['tx40']
        
        assert 'robot_name' in schema.required_fields
        assert 'NbSample' in schema.required_fields
        assert 'active_joints' in schema.required_fields
        assert schema.field_types['NbSample'] == int
        assert schema.field_types['active_joints'] == list
    
    def test_ur10_schema(self):
        """Test UR10 configuration schema."""
        schema = ConfigManager.ROBOT_SCHEMAS['ur10']
        
        assert 'robot_name' in schema.required_fields
        assert 'NbSample' in schema.required_fields
        assert 'config_idx' in schema.required_fields
    
    def test_all_robot_schemas_valid(self):
        """Test that all predefined schemas are valid."""
        for robot_type, schema in ConfigManager.ROBOT_SCHEMAS.items():
            assert isinstance(schema.required_fields, list)
            assert isinstance(schema.optional_fields, list)
            assert isinstance(schema.field_types, dict)


if __name__ == '__main__':
    pytest.main([__file__])
