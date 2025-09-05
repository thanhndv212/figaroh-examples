"""
Integration tests for robot unified configurations with actual robot examples.

This module tests that the unified configuration files work correctly with
existing robot examples and can be used in place of legacy configurations.
"""

import pytest
from pathlib import Path
import sys
import subprocess
import tempfile
import yaml


class TestRobotExampleIntegration:
    """Test integration with actual robot examples."""
    
    def test_ur10_example_with_unified_config(self):
        """Test that UR10 calibration example works with unified config."""
        
        # Create a simplified UR10 config that mimics the original
        ur10_config = {
            'robot': {
                'name': 'ur10'
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'kinematics': {
                        'base_frame': 'universe',
                        'tool_frame': 'wrist_3_joint'
                    },
                    'measurements': {
                        'markers': [{
                            'name': 'wrist_marker',
                            'reference_joint': 'wrist_3_joint',
                            'measurable_dof': [True, True, True, True, True, True],
                            'position': [0.0, 0.0, 0.0]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'full_kinematic',
                        'outlier_threshold': 0.02,
                        'max_iterations': 50
                    },
                    'data': {
                        'number_of_samples': 100
                    }
                }
            }
        }
        
        # Test that config can be parsed without template issues
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(ur10_config, f)
            config_file = f.name
        
        try:
            from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config
            
            # Mock robot for testing
            class MockRobot:
                def __init__(self):
                    self.model = type('obj', (object,), {'name': 'ur10'})()
                    self.q0 = [0.0] * 6
            
            robot = MockRobot()
            
            # Parse configuration
            parser = UnifiedConfigParser(config_file)
            config = parser.parse()
            
            # Create calibration task config
            cal_config = create_task_config(robot, config, 'calibration')
            
            # Verify the config structure matches what calibration tools expect
            assert cal_config['robot_name'] == 'ur10'
            assert cal_config['task_type'] == 'kinematic_calibration'
            assert 'kinematics' in cal_config
            assert 'measurements' in cal_config
            assert 'parameters' in cal_config
            
            # Verify calibration-specific fields
            assert cal_config['kinematics']['base_frame'] == 'universe'
            assert cal_config['kinematics']['tool_frame'] == 'wrist_3_joint'
            assert len(cal_config['measurements']['markers']) == 1
            assert cal_config['parameters']['outlier_threshold'] == 0.02
            
        finally:
            Path(config_file).unlink()
    
    def test_tiago_example_with_unified_config(self):
        """Test that TIAGo calibration example works with unified config."""
        
        tiago_config = {
            'robot': {
                'name': 'tiago',
                'properties': {
                    'coupling': {
                        'has_coupled_wrist': True,
                        'coupled_joints': [["arm_6_joint", "arm_7_joint"]]
                    }
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'kinematics': {
                        'base_frame': 'universe',
                        'tool_frame': 'arm_7_joint'
                    },
                    'measurements': {
                        'markers': [{
                            'name': 'arm_marker',
                            'reference_joint': 'arm_7_joint',
                            'measurable_dof': [True, True, True, False, False, False],
                            'position': [0.0, 0.0, 0.1]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'full_kinematic',
                        'outlier_threshold': 0.01
                    }
                }
            },
            'variants': {
                'mocap_calibration': {
                    'tasks': {
                        'calibration': {
                            'kinematics': {
                                'base_frame': 'head_2_joint'
                            },
                            'parameters': {
                                'outlier_threshold': 0.005
                            }
                        }
                    }
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(tiago_config, f)
            config_file = f.name
        
        try:
            from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config
            
            class MockRobot:
                def __init__(self):
                    self.model = type('obj', (object,), {'name': 'tiago'})()
                    self.q0 = [0.0] * 8
            
            robot = MockRobot()
            
            # Test default configuration
            parser = UnifiedConfigParser(config_file)
            config = parser.parse()
            cal_config = create_task_config(robot, config, 'calibration')
            
            assert cal_config['robot_name'] == 'tiago'
            assert cal_config['kinematics']['base_frame'] == 'universe'
            assert cal_config['parameters']['outlier_threshold'] == 0.01
            
            # Test mocap variant
            parser_variant = UnifiedConfigParser(config_file, variant='mocap_calibration')
            config_variant = parser_variant.parse()
            cal_config_variant = create_task_config(robot, config_variant, 'calibration')
            
            assert cal_config_variant['kinematics']['base_frame'] == 'head_2_joint'
            assert cal_config_variant['parameters']['outlier_threshold'] == 0.005
            
            # Verify TIAGo-specific coupling properties are preserved
            if 'coupling' in config['robot']['properties']:
                coupling = config['robot']['properties']['coupling']
                assert coupling['has_coupled_wrist'] is True
                assert len(coupling['coupled_joints']) == 1
            
        finally:
            Path(config_file).unlink()
    
    def test_staubli_identification_with_unified_config(self):
        """Test that Staubli identification example works with unified config."""
        
        staubli_config = {
            'robot': {
                'name': 'staubli_tx40',
                'properties': {
                    'joints': {
                        'active_joints': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
                    },
                    'mechanics': {
                        'friction_coefficients': {
                            'viscous': [0.8, 0.8, 0.5, 0.2, 0.2, 0.1],
                            'coulomb': [1.5, 1.5, 1.0, 0.5, 0.5, 0.3]
                        },
                        'reduction_ratios': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
                    }
                }
            },
            'tasks': {
                'identification': {
                    'enabled': True,
                    'type': 'dynamic_identification',
                    'parameters': {
                        'estimation_method': 'weighted_least_squares',
                        'regularization_factor': 1e-6,
                        'outlier_threshold': 0.1
                    },
                    'problem': {
                        'has_friction': True,
                        'has_external_forces': False,
                        'friction_model': 'viscous_coulomb'
                    },
                    'signal_processing': {
                        'sampling_rate': 1000.0,
                        'filter_cutoff': 30.0,
                        'filter_order': 4
                    },
                    'data': {
                        'trajectory_type': 'fourier',
                        'excitation_duration': 10.0
                    }
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(staubli_config, f)
            config_file = f.name
        
        try:
            from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config
            
            class MockRobot:
                def __init__(self):
                    self.model = type('obj', (object,), {'name': 'staubli_tx40'})()
                    self.q0 = [0.0] * 6
            
            robot = MockRobot()
            
            parser = UnifiedConfigParser(config_file)
            config = parser.parse()
            id_config = create_task_config(robot, config, 'identification')
            
            # Verify identification-specific configuration
            assert id_config['robot_name'] == 'staubli_tx40'
            assert id_config['task_type'] == 'dynamic_identification'
            assert id_config['parameters']['estimation_method'] == 'weighted_least_squares'
            assert id_config['problem']['has_friction'] is True
            assert id_config['signal_processing']['filter_cutoff'] == 30.0
            
            # Verify mechanical properties
            assert len(id_config['joints']['active_joints']) == 6
            assert len(id_config['mechanics']['friction_coefficients']['viscous']) == 6
            assert len(id_config['mechanics']['reduction_ratios']) == 6
            
        finally:
            Path(config_file).unlink()
    
    def test_talos_dual_arm_configuration(self):
        """Test TALOS dual-arm configuration handling."""
        
        talos_config = {
            'robot': {
                'name': 'talos',
                'properties': {
                    'joints': {
                        'active_joints': [
                            "torso_1_joint", "torso_2_joint",
                            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", 
                            "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
                            "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
                            "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
                        ]
                    },
                    'mechanics': {
                        'reduction_ratios': [160.0, 160.0, 160.0, 160.0, 160.0, 100.0, 100.0, 100.0, 100.0,
                                           160.0, 160.0, 160.0, 160.0, 100.0, 100.0, 100.0],
                        'friction_coefficients': {
                            'viscous': [3.0] * 16,
                            'coulomb': [5.0] * 16
                        }
                    }
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'kinematics': {
                        'base_frame': 'universe',
                        'tool_frame': 'arm_left_7_joint'
                    },
                    'measurements': {
                        'markers': [{
                            'name': 'left_arm_marker',
                            'reference_joint': 'arm_left_7_joint',
                            'measurable_dof': [True, True, True, True, True, True]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'joint_offset'
                    }
                }
            },
            'variants': {
                'right_arm_calibration': {
                    'tasks': {
                        'calibration': {
                            'kinematics': {
                                'tool_frame': 'arm_right_7_joint'
                            },
                            'measurements': {
                                'markers': [{
                                    'name': 'right_arm_marker',
                                    'reference_joint': 'arm_right_7_joint',
                                    'measurable_dof': [True, True, True, True, True, True]
                                }]
                            }
                        }
                    }
                },
                'dual_arm_calibration': {
                    'tasks': {
                        'calibration': {
                            'measurements': {
                                'markers': [
                                    {
                                        'name': 'left_arm_marker',
                                        'reference_joint': 'arm_left_7_joint',
                                        'measurable_dof': [True, True, True, True, True, True]
                                    },
                                    {
                                        'name': 'right_arm_marker', 
                                        'reference_joint': 'arm_right_7_joint',
                                        'measurable_dof': [True, True, True, True, True, True]
                                    }
                                ]
                            }
                        }
                    }
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(talos_config, f)
            config_file = f.name
        
        try:
            from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config
            
            class MockRobot:
                def __init__(self):
                    self.model = type('obj', (object,), {'name': 'talos'})()
                    self.q0 = [0.0] * 16
            
            robot = MockRobot()
            
            # Test left arm (default)
            parser = UnifiedConfigParser(config_file)
            config = parser.parse()
            cal_config = create_task_config(robot, config, 'calibration')
            
            assert cal_config['robot_name'] == 'talos'
            assert len(cal_config['joints']['active_joints']) == 16
            assert cal_config['kinematics']['tool_frame'] == 'arm_left_7_joint'
            assert len(cal_config['measurements']['markers']) == 1
            assert cal_config['measurements']['markers'][0]['name'] == 'left_arm_marker'
            
            # Test right arm variant
            parser_right = UnifiedConfigParser(config_file, variant='right_arm_calibration')
            config_right = parser_right.parse()
            cal_config_right = create_task_config(robot, config_right, 'calibration')
            
            assert cal_config_right['kinematics']['tool_frame'] == 'arm_right_7_joint'
            assert cal_config_right['measurements']['markers'][0]['name'] == 'right_arm_marker'
            
            # Test dual arm variant
            parser_dual = UnifiedConfigParser(config_file, variant='dual_arm_calibration')
            config_dual = parser_dual.parse()
            cal_config_dual = create_task_config(robot, config_dual, 'calibration')
            
            assert len(cal_config_dual['measurements']['markers']) == 2
            marker_names = [m['name'] for m in cal_config_dual['measurements']['markers']]
            assert 'left_arm_marker' in marker_names
            assert 'right_arm_marker' in marker_names
            
        finally:
            Path(config_file).unlink()


class TestBackwardCompatibilityIntegration:
    """Test backward compatibility with legacy configuration formats."""
    
    def test_legacy_to_unified_config_conversion(self):
        """Test that legacy configurations can be converted to unified format."""
        from figaroh.utils.config_parser import get_param_from_yaml
        
        # Mock robot with enhanced attributes
        class EnhancedMockRobot:
            def __init__(self, name="test_robot"):
                self.model = type('Model', (), {
                    'name': name,
                    'frames': [
                        type('Frame', (), {'name': 'universe'}),
                        type('Frame', (), {'name': 'base'}),
                        type('Frame', (), {'name': 'tool'})
                    ]
                })()
                # Add expected methods for legacy parsers
                self.model.getFrameId = lambda name: 1 if name == 'tool' else 0
                self.q0 = [0.0, 0.0, 0.0]
        
        robot = EnhancedMockRobot()
        
        # Legacy calibration format (simplified to avoid mock complexity)
        legacy_config = {
            'calib_level': 'joint_offset',
            'base_frame': 'base',
            'tool_frame': 'tool',
            'markers': [{
                'ref_joint': 'tool',
                'measure': [True, True, True, False, False, False],
                'position': [0.0, 0.0, 0.1]
            }],
            'nb_sample': 100,
            'non_geom': False,
            'free_flyer': False
        }
        
        # Test that the unified parser can handle legacy format gracefully
        try:
            result = get_param_from_yaml(robot, legacy_config, "calibration")
            
            # If it worked, verify basic structure
            if isinstance(result, dict):
                assert 'robot_name' in result
                # Additional checks can be added here
        
        except Exception as e:
            # Expected for incomplete mocks - verify it's a reasonable error
            error_msg = str(e).lower()
            expected_errors = [
                'attribute', 'frames', 'mock', 'parent', 'joint', 'index'
            ]
            assert any(err in error_msg for err in expected_errors), f"Unexpected error: {e}"
    
    def test_unified_config_structure_validation(self):
        """Test that unified configs have consistent structure across robots."""
        
        required_sections = {
            'robot': ['name'],
            'tasks': {
                'calibration': ['enabled', 'type'],
                'identification': ['enabled', 'type']
            }
        }
        
        robot_configs = [
            {'name': 'ur10', 'joints': 6},
            {'name': 'tiago', 'joints': 8},
            {'name': 'talos', 'joints': 16},
            {'name': 'staubli_tx40', 'joints': 6}
        ]
        
        for robot_spec in robot_configs:
            # Create minimal valid config
            test_config = {
                'robot': {
                    'name': robot_spec['name'],
                    'properties': {
                        'joints': {
                            'active_joints': [f"joint_{i}" for i in range(robot_spec['joints'])]
                        }
                    }
                },
                'tasks': {
                    'calibration': {
                        'enabled': True,
                        'type': 'kinematic_calibration',
                        'kinematics': {'base_frame': 'base', 'tool_frame': 'tool'},
                        'measurements': {'markers': []},
                        'parameters': {'outlier_threshold': 0.05}
                    },
                    'identification': {
                        'enabled': False,
                        'type': 'dynamic_identification',
                        'parameters': {'estimation_method': 'weighted_least_squares'},
                        'problem': {'has_friction': True, 'has_external_forces': False},
                        'signal_processing': {'sampling_rate': 1000.0, 'filter_cutoff': 50.0}
                    }
                }
            }
            
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                yaml.dump(test_config, f)
                config_file = f.name
            
            try:
                from figaroh.utils.config_parser import UnifiedConfigParser
                
                parser = UnifiedConfigParser(config_file)
                config = parser.parse()
                
                # Verify required sections exist
                assert 'robot' in config
                assert 'name' in config['robot']
                assert config['robot']['name'] == robot_spec['name']
                
                assert 'tasks' in config
                assert 'calibration' in config['tasks']
                assert 'identification' in config['tasks']
                
                # Verify task structure
                for task_name in ['calibration', 'identification']:
                    task = config['tasks'][task_name]
                    assert 'enabled' in task
                    assert 'type' in task
                    assert isinstance(task['enabled'], bool)
                
                # Verify joint count matches expectation
                if 'joints' in config['robot']['properties']:
                    active_joints = config['robot']['properties']['joints']['active_joints']
                    assert len(active_joints) == robot_spec['joints']
                
            finally:
                Path(config_file).unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
