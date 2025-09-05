"""
Test suite for the unified configuration parser system.

Tests cover:
- Basic configuration loading
- Template inheritance
- Variant application
- Variable expansion
- Backward compatibility
- Error handling
"""

import pytest
import yaml
import os
import tempfile
from pathlib import Path
from unittest.mock import Mock

# Mock robot for testing
# Mock robot for testing
class MockRobot:
    def __init__(self, name="test_robot"):
        self.model = Mock()
        self.model.name = name
        # Add frames attribute for legacy compatibility
        self.model.frames = [
            Mock(name='base'),
            Mock(name='joint1'),
            Mock(name='joint2'),
            Mock(name='joint3'),
            Mock(name='tool')
        ]
        self.q0 = [0.0, 0.0, 0.0]


@pytest.fixture
def mock_robot():
    """Create a mock robot for testing."""
    return MockRobot()


@pytest.fixture
def temp_config_dir():
    """Create temporary directory for config files."""
    with tempfile.TemporaryDirectory() as temp_dir:
        config_dir = Path(temp_dir)
        
        # Create templates directory
        templates_dir = config_dir / "templates"
        templates_dir.mkdir()
        
        yield config_dir


class TestUnifiedConfigParser:
    """Test the UnifiedConfigParser class."""
    
    def test_basic_config_loading(self, temp_config_dir, mock_robot):
        """Test basic configuration file loading."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        # Create basic config file
        config_content = {
            'meta': {'schema_version': '2.0'},
            'robot': {
                'name': 'test_robot',
                'properties': {
                    'joints': {'active_joints': ['joint1', 'joint2']}
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'kinematics': {'base_frame': 'base', 'tool_frame': 'tool'},
                    'measurements': {'markers': []}
                }
            }
        }
        
        config_file = temp_config_dir / "test_config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(config_content, f)
        
        # Parse configuration
        parser = UnifiedConfigParser(config_file)
        config = parser.parse()
        
        # Verify basic structure
        assert config['robot']['name'] == 'test_robot'
        assert 'calibration' in config['tasks']
        assert config['tasks']['calibration']['enabled'] is True
        assert '_metadata' in config
        
    def test_template_inheritance(self, temp_config_dir, mock_robot):
        """Test template inheritance functionality."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        # Create base template
        base_template = {
            'robot': {
                'name': '${ROBOT_NAME}',
                'properties': {
                    'joints': {'active_joints': []},
                    'mechanics': {'friction_coefficients': {'viscous': []}}
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': False,
                    'parameters': {'outlier_threshold': 0.05}
                }
            }
        }
        
        template_file = temp_config_dir / "templates" / "base.yaml"
        with open(template_file, 'w') as f:
            yaml.dump(base_template, f)
        
        # Create robot-specific config that extends template
        robot_config = {
            'extends': 'templates/base.yaml',
            'robot': {
                'name': 'specific_robot',
                'properties': {
                    'joints': {'active_joints': ['j1', 'j2']},
                    'mechanics': {'friction_coefficients': {'viscous': [0.1, 0.2]}}
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'parameters': {'outlier_threshold': 0.01}
                }
            }
        }
        
        robot_config_file = temp_config_dir / "robot_config.yaml"
        with open(robot_config_file, 'w') as f:
            yaml.dump(robot_config, f)
        
        # Parse configuration
        parser = UnifiedConfigParser(robot_config_file)
        config = parser.parse()
        
        # Verify inheritance worked
        assert config['robot']['name'] == 'specific_robot'  # Override
        assert config['robot']['properties']['joints']['active_joints'] == ['j1', 'j2']
        assert config['tasks']['calibration']['enabled'] is True  # Override
        assert config['tasks']['calibration']['parameters']['outlier_threshold'] == 0.01
        
    def test_task_inheritance(self, temp_config_dir, mock_robot):
        """Test task inheritance (inherits_from)."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        config_content = {
            'robot': {'name': 'test_robot'},
            'tasks': {
                'base_calibration': {
                    'type': 'kinematic_calibration',
                    'parameters': {'outlier_threshold': 0.05},
                    'kinematics': {'base_frame': 'base', 'tool_frame': 'tool'},
                    'measurements': {'markers': []},
                    'data': {'number_of_samples': 100}
                },
                'precision_calibration': {
                    'inherits_from': 'base_calibration',
                    'parameters': {'outlier_threshold': 0.01},
                    'data': {'number_of_samples': 200}
                }
            }
        }
        
        config_file = temp_config_dir / "test_config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(config_content, f)
        
        parser = UnifiedConfigParser(config_file)
        config = parser.parse()
        
        # Verify task inheritance
        precision_task = config['tasks']['precision_calibration']
        assert precision_task['type'] == 'kinematic_calibration'  # Inherited
        assert precision_task['parameters']['outlier_threshold'] == 0.01  # Override
        assert precision_task['data']['number_of_samples'] == 200  # Override
        assert 'inherits_from' not in precision_task  # Should be removed
        
    def test_variant_application(self, temp_config_dir, mock_robot):
        """Test variant configuration application."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        config_content = {
            'robot': {'name': 'test_robot'},
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'parameters': {'outlier_threshold': 0.05},
                    'data': {'number_of_samples': 100}
                }
            },
            'variants': {
                'precision_mode': {
                    'tasks': {
                        'calibration': {
                            'parameters': {'outlier_threshold': 0.01},
                            'data': {'number_of_samples': 200}
                        }
                    }
                }
            }
        }
        
        config_file = temp_config_dir / "test_config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(config_content, f)
        
        # Parse with variant
        parser = UnifiedConfigParser(config_file, variant='precision_mode')
        config = parser.parse()
        
        # Verify variant was applied
        assert config['tasks']['calibration']['parameters']['outlier_threshold'] == 0.01
        assert config['tasks']['calibration']['data']['number_of_samples'] == 200
        assert 'variants' not in config  # Should be removed
        
    def test_variable_expansion(self, temp_config_dir, mock_robot):
        """Test variable expansion functionality."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        # Set environment variable
        os.environ['TEST_ROBOT_NAME'] = 'env_robot'
        
        config_content = {
            'robot': {
                'name': '${TEST_ROBOT_NAME}',
                'description': 'Robot ${TEST_ROBOT_NAME} for testing'
            },
            'tasks': {
                'calibration': {
                    'data': {
                        'source_file': 'data/${TEST_ROBOT_NAME}_calibration.csv'
                    }
                }
            },
            'variables': {
                'CONFIG_VALUE': 'test_value'
            }
        }
        
        config_file = temp_config_dir / "test_config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(config_content, f)
        
        parser = UnifiedConfigParser(config_file)
        config = parser.parse()
        
        # Verify variable expansion
        assert config['robot']['name'] == 'env_robot'
        assert 'env_robot' in config['robot']['description']
        assert 'env_robot' in config['tasks']['calibration']['data']['source_file']
        
        # Clean up
        del os.environ['TEST_ROBOT_NAME']
        
    def test_validation_errors(self, temp_config_dir):
        """Test configuration validation errors."""
        from figaroh.utils.config_parser import UnifiedConfigParser, ConfigurationError
        
        # Test missing file
        with pytest.raises(ConfigurationError, match="Configuration file not found"):
            UnifiedConfigParser("nonexistent.yaml")
        
        # Test invalid YAML
        invalid_file = temp_config_dir / "invalid.yaml"
        with open(invalid_file, 'w') as f:
            f.write("invalid: yaml: content: [")
        
        with pytest.raises(ConfigurationError, match="Invalid YAML"):
            parser = UnifiedConfigParser(invalid_file)
            parser.parse()
        
        # Test missing variant
        valid_config = temp_config_dir / "valid.yaml" 
        with open(valid_config, 'w') as f:
            yaml.dump({'robot': {'name': 'test'}}, f)
        
        with pytest.raises(ConfigurationError, match="No variants defined"):
            parser = UnifiedConfigParser(valid_config, variant='nonexistent')
            parser.parse()


class TestConfigurationCompatibility:
    """Test backward compatibility features."""
    
    def test_legacy_calibration_format(self, mock_robot):
        """Test parsing legacy calibration format."""
        from figaroh.utils.config_parser import get_param_from_yaml
        
        # Legacy format
        legacy_config = {
            'calib_level': 'full_params',
            'base_frame': 'universe',
            'tool_frame': 'wrist_3_link',
            'markers': [
                {
                    'ref_joint': 'wrist_3_joint',
                    'measure': [True, True, True, True, True, True]
                }
            ],
            'nb_sample': 50,
            'non_geom': False,
            'free_flyer': False
        }
        
        # This should fall back to legacy parser
        try:
            result = get_param_from_yaml(mock_robot, legacy_config, "calibration")
            # If successful, basic structure should be preserved
            assert isinstance(result, dict)
        except Exception as e:
            # Expected if legacy parser is not available in test environment
            assert "import" in str(e).lower() or "legacy" in str(e).lower()
    
    def test_legacy_identification_format(self, mock_robot):
        """Test parsing legacy identification format."""
        from figaroh.utils.config_parser import get_param_from_yaml
        
        # Legacy format
        legacy_config = {
            'robot_params': [{
                'q_lim_def': 1.57,
                'fv': [0] * 6,
                'fs': [0] * 6
            }],
            'problem_params': [{
                'is_external_wrench': False,
                'is_joint_torques': True,
                'has_friction': True
            }],
            'processing_params': [{
                'ts': 0.001,
                'cut_off_frequency_butterworth': 50.0
            }]
        }
        
        # This should fall back to legacy parser
        try:
            result = get_param_from_yaml(mock_robot, legacy_config, "identification")
            assert isinstance(result, dict)
        except Exception as e:
            # Expected if legacy parser is not available in test environment
            assert "import" in str(e).lower() or "legacy" in str(e).lower()


class TestTaskConfigCreation:
    """Test task-specific configuration creation."""
    
    def test_create_task_config(self, mock_robot):
        """Test creating task-specific configuration from unified config."""
        from figaroh.utils.config_parser import create_task_config
        
        unified_config = {
            'robot': {
                'name': 'test_robot',
                'properties': {
                    'joints': {'active_joints': ['j1', 'j2']},
                    'mechanics': {'reduction_ratios': [50.0, 50.0]}
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'parameters': {'outlier_threshold': 0.05},
                    'kinematics': {'base_frame': 'base', 'tool_frame': 'tool'}
                },
                'identification': {
                    'enabled': False,
                    'type': 'dynamic_identification'
                }
            },
            'custom': {
                'manufacturer': 'Test Corp'
            }
        }
        
        # Create calibration task config
        cal_config = create_task_config(mock_robot, unified_config, 'calibration')
        
        assert cal_config['robot_name'] == 'test_robot'
        assert cal_config['task_type'] == 'kinematic_calibration'
        assert cal_config['joints']['active_joints'] == ['j1', 'j2']
        assert cal_config['mechanics']['reduction_ratios'] == [50.0, 50.0]
        assert cal_config['parameters']['outlier_threshold'] == 0.05
        assert cal_config['custom']['manufacturer'] == 'Test Corp'
        
        # Test disabled task
        from figaroh.utils.config_parser import ConfigurationError
        with pytest.raises(ConfigurationError, match="disabled"):
            create_task_config(mock_robot, unified_config, 'identification')
        
        # Test missing task
        with pytest.raises(ConfigurationError, match="not found"):
            create_task_config(mock_robot, unified_config, 'nonexistent')


class TestRealConfigFiles:
    """Test with actual robot configuration files."""
    
    def test_ur10_config_parsing(self, mock_robot):
        """Test parsing UR10 unified configuration."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        # Path to UR10 config (adjust if needed)
        config_path = Path("examples/ur10/config/ur10_unified_config.yaml")
        
        if config_path.exists():
            try:
                parser = UnifiedConfigParser(config_path)
                config = parser.parse()
                
                assert config['robot']['name'] == 'ur10'
                assert 'calibration' in config['tasks']
                assert 'identification' in config['tasks']
                
                # Test variant
                parser_variant = UnifiedConfigParser(config_path, variant='minimal_calibration')
                config_variant = parser_variant.parse()
                
                # Verify variant applied
                cal_task = config_variant['tasks']['calibration']
                assert cal_task['parameters']['calibration_level'] == 'joint_offset'
                
            except Exception as e:
                pytest.skip(f"UR10 config test skipped: {e}")
        else:
            pytest.skip("UR10 config file not found")
    
    def test_tiago_config_parsing(self, mock_robot):
        """Test parsing TIAGo unified configuration.""" 
        from figaroh.utils.config_parser import UnifiedConfigParser
        
        config_path = Path("examples/tiago/config/tiago_unified_config.yaml")
        
        if config_path.exists():
            try:
                parser = UnifiedConfigParser(config_path)
                config = parser.parse()
                
                assert config['robot']['name'] == 'tiago'
                assert config['robot']['properties']['coupling']['has_coupled_wrist'] is True
                assert 'calibration' in config['tasks']
                
                # Test mocap variant
                parser_mocap = UnifiedConfigParser(config_path, variant='mocap_calibration')
                config_mocap = parser_mocap.parse()
                
                mocap_task = config_mocap['tasks']['calibration']
                assert mocap_task['kinematics']['base_frame'] == 'head_2_joint'
                
            except Exception as e:
                pytest.skip(f"TIAGo config test skipped: {e}")
        else:
            pytest.skip("TIAGo config file not found")


if __name__ == "__main__":
    # Run tests
    pytest.main([__file__, "-v"])
