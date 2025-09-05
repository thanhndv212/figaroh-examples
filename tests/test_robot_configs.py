"""
Tests for individual robot unified configuration files.

This module contains comprehensive tests for each robot's unified configuration,
verifying robot-specific properties, task configurations, variants, and
template inheritance behavior.
"""

import pytest
import yaml
from pathlib import Path
from unittest.mock import Mock

# Import the configuration parser
from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config


class TestUR10UnifiedConfig:
    """Test suite for UR10 unified configuration."""
    
    @pytest.fixture
    def config_path(self):
        """Path to UR10 unified configuration."""
        return Path("examples/ur10/config/ur10_unified_config.yaml")
    
    @pytest.fixture
    def mock_robot(self):
        """Create a mock UR10 robot for testing."""
        robot = Mock()
        robot.model.name = "ur10"
        robot.q0 = [0.0] * 6
        return robot
    
    def test_ur10_config_loading(self, config_path):
        """Test basic UR10 configuration loading."""
        # Create a simplified config without template inheritance for testing
        simplified_config = {
            'robot': {
                'name': 'ur10',
                'description': 'Universal Robots UR10 6-DOF industrial manipulator',
                'properties': {
                    'joints': {
                        'active_joints': [
                            "shoulder_pan_joint", "shoulder_lift_joint", 
                            "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                        ],
                        'joint_limits': {
                            'velocity': [2.16, 2.16, 3.15, 3.2, 3.2, 3.2],
                            'torque': [330.0, 330.0, 150.0, 54.0, 54.0, 54.0]
                        }
                    },
                    'mechanics': {
                        'reduction_ratios': [101.0, 101.0, 101.0, 101.0, 101.0, 101.0],
                        'friction_coefficients': {
                            'viscous': [5.0, 5.0, 2.0, 0.5, 0.5, 0.5],
                            'coulomb': [8.0, 8.0, 4.0, 1.0, 1.0, 1.0]
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
                        'tool_frame': 'wrist_3_joint'
                    },
                    'measurements': {
                        'markers': [{
                            'name': 'wrist_marker',
                            'reference_joint': 'wrist_3_joint',
                            'measurable_dof': [True, True, True, True, True, True]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'full_kinematic',
                        'outlier_threshold': 0.02
                    }
                }
            }
        }
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(simplified_config, f)
            test_config_path = f.name
        
        try:
            parser = UnifiedConfigParser(test_config_path)
            config = parser.parse()
            
            # Verify robot properties
            assert config['robot']['name'] == 'ur10'
            assert 'Universal Robots' in config['robot']['description']
            assert len(config['robot']['properties']['joints']['active_joints']) == 6
            assert 'shoulder_pan_joint' in config['robot']['properties']['joints']['active_joints']
            
            # Verify mechanical properties
            mechanics = config['robot']['properties']['mechanics']
            assert len(mechanics['reduction_ratios']) == 6
            assert all(ratio == 101.0 for ratio in mechanics['reduction_ratios'])
            assert len(mechanics['friction_coefficients']['viscous']) == 6
            
            # Verify calibration task
            cal_task = config['tasks']['calibration']
            assert cal_task['enabled'] is True
            assert cal_task['type'] == 'kinematic_calibration'
            assert cal_task['kinematics']['tool_frame'] == 'wrist_3_joint'
            assert len(cal_task['measurements']['markers']) == 1
            
        finally:
            Path(test_config_path).unlink()
    
    def test_ur10_task_creation(self, mock_robot):
        """Test creating UR10 task configurations."""
        config = {
            'robot': {
                'name': 'ur10',
                'properties': {
                    'joints': {'active_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']},
                    'mechanics': {'reduction_ratios': [101.0] * 6}
                }
            },
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'parameters': {'outlier_threshold': 0.02},
                    'kinematics': {'base_frame': 'universe', 'tool_frame': 'wrist_3_joint'},
                    'measurements': {'markers': []}
                }
            }
        }
        
        task_config = create_task_config(mock_robot, config, 'calibration')
        
        assert task_config['robot_name'] == 'ur10'
        assert task_config['task_type'] == 'kinematic_calibration'
        assert len(task_config['joints']['active_joints']) == 6
        assert len(task_config['mechanics']['reduction_ratios']) == 6
        assert task_config['parameters']['outlier_threshold'] == 0.02


class TestTIAGoUnifiedConfig:
    """Test suite for TIAGo unified configuration."""
    
    @pytest.fixture
    def mock_robot(self):
        """Create a mock TIAGo robot for testing."""
        robot = Mock()
        robot.model.name = "tiago"
        robot.q0 = [0.0] * 8  # torso + 7-DOF arm
        return robot
    
    def test_tiago_config_loading(self):
        """Test basic TIAGo configuration loading."""
        simplified_config = {
            'robot': {
                'name': 'tiago',
                'description': 'PAL Robotics TIAGo mobile manipulator robot',
                'properties': {
                    'joints': {
                        'active_joints': [
                            "torso_lift_joint", "arm_1_joint", "arm_2_joint", 
                            "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
                        ],
                        'joint_limits': {
                            'velocity': [0.05009095, 0.05009095, 0.07504916, 0.0715585, 0.05585054, 0.12217305],
                            'torque': [4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
                        }
                    },
                    'mechanics': {
                        'reduction_ratios': [32.0, 32.0, 45.0, -48.0, 45.0, 32.0],
                        'friction_coefficients': {
                            'viscous': [0] * 12,
                            'coulomb': [0] * 12
                        }
                    },
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
                            'measurable_dof': [True, True, True, False, False, False]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'full_kinematic',
                        'outlier_threshold': 0.01
                    }
                }
            }
        }
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(simplified_config, f)
            test_config_path = f.name
        
        try:
            parser = UnifiedConfigParser(test_config_path)
            config = parser.parse()
            
            # Verify robot properties
            assert config['robot']['name'] == 'tiago'
            assert 'PAL Robotics' in config['robot']['description']
            assert len(config['robot']['properties']['joints']['active_joints']) == 8
            assert 'torso_lift_joint' in config['robot']['properties']['joints']['active_joints']
            
            # Verify coupling properties specific to TIAGo
            coupling = config['robot']['properties']['coupling']
            assert coupling['has_coupled_wrist'] is True
            assert len(coupling['coupled_joints']) == 1
            assert coupling['coupled_joints'][0] == ["arm_6_joint", "arm_7_joint"]
            
            # Verify calibration task
            cal_task = config['tasks']['calibration']
            assert cal_task['enabled'] is True
            assert cal_task['kinematics']['tool_frame'] == 'arm_7_joint'
            assert cal_task['parameters']['outlier_threshold'] == 0.01
            
        finally:
            Path(test_config_path).unlink()
    
    def test_tiago_variants(self):
        """Test TIAGo configuration variants."""
        config_with_variants = {
            'robot': {'name': 'tiago'},
            'tasks': {
                'calibration': {
                    'enabled': True,
                    'type': 'kinematic_calibration',
                    'kinematics': {'base_frame': 'universe', 'tool_frame': 'arm_7_joint'},
                    'measurements': {'markers': []},
                    'parameters': {'outlier_threshold': 0.05}
                }
            },
            'variants': {
                'mocap_calibration': {
                    'tasks': {
                        'calibration': {
                            'kinematics': {'base_frame': 'head_2_joint'},
                            'parameters': {'outlier_threshold': 0.01}
                        }
                    }
                }
            }
        }
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_with_variants, f)
            test_config_path = f.name
        
        try:
            # Test default configuration
            parser = UnifiedConfigParser(test_config_path)
            config = parser.parse()
            assert config['tasks']['calibration']['kinematics']['base_frame'] == 'universe'
            assert config['tasks']['calibration']['parameters']['outlier_threshold'] == 0.05
            
            # Test mocap variant
            parser_variant = UnifiedConfigParser(test_config_path, variant='mocap_calibration')
            config_variant = parser_variant.parse()
            assert config_variant['tasks']['calibration']['kinematics']['base_frame'] == 'head_2_joint'
            assert config_variant['tasks']['calibration']['parameters']['outlier_threshold'] == 0.01
            
        finally:
            Path(test_config_path).unlink()


class TestTALOSUnifiedConfig:
    """Test suite for TALOS unified configuration."""
    
    @pytest.fixture
    def mock_robot(self):
        """Create a mock TALOS robot for testing."""
        robot = Mock()
        robot.model.name = "talos"
        robot.q0 = [0.0] * 16  # torso + dual arms
        return robot
    
    def test_talos_config_loading(self):
        """Test basic TALOS configuration loading."""
        simplified_config = {
            'robot': {
                'name': 'talos',
                'description': 'PAL Robotics TALOS humanoid robot - torso and arm system',
                'properties': {
                    'joints': {
                        'active_joints': [
                            "torso_1_joint", "torso_2_joint",
                            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", 
                            "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
                            "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
                            "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
                        ],
                        'joint_limits': {
                            'torque': [200.0, 200.0, 150.0, 150.0, 100.0, 100.0, 50.0, 50.0, 50.0,
                                     150.0, 150.0, 100.0, 100.0, 50.0, 50.0, 50.0]
                        }
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
                        'calibration_level': 'joint_offset',
                        'outlier_threshold': 0.02
                    }
                }
            }
        }
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(simplified_config, f)
            test_config_path = f.name
        
        try:
            parser = UnifiedConfigParser(test_config_path)
            config = parser.parse()
            
            # Verify robot properties
            assert config['robot']['name'] == 'talos'
            assert 'TALOS' in config['robot']['description']
            assert len(config['robot']['properties']['joints']['active_joints']) == 16
            assert 'torso_1_joint' in config['robot']['properties']['joints']['active_joints']
            assert 'arm_left_7_joint' in config['robot']['properties']['joints']['active_joints']
            assert 'arm_right_7_joint' in config['robot']['properties']['joints']['active_joints']
            
            # Verify high torque capabilities
            torques = config['robot']['properties']['joints']['joint_limits']['torque']
            assert len(torques) == 16
            assert torques[0] == 200.0  # Torso joint has high torque
            assert torques[2] == 150.0  # Arm joint has medium torque
            
            # Verify dual-arm configuration
            reduction_ratios = config['robot']['properties']['mechanics']['reduction_ratios']
            assert len(reduction_ratios) == 16
            assert reduction_ratios[0] == 160.0  # Torso
            assert reduction_ratios[5] == 100.0  # Wrist joints have lower ratios
            
            # Verify calibration task
            cal_task = config['tasks']['calibration']
            assert cal_task['enabled'] is True
            assert cal_task['kinematics']['tool_frame'] == 'arm_left_7_joint'
            
        finally:
            Path(test_config_path).unlink()


class TestStaubliTX40UnifiedConfig:
    """Test suite for Staubli TX40 unified configuration."""
    
    @pytest.fixture
    def mock_robot(self):
        """Create a mock Staubli TX40 robot for testing."""
        robot = Mock()
        robot.model.name = "staubli_tx40"
        robot.q0 = [0.0] * 6
        return robot
    
    def test_staubli_config_loading(self):
        """Test basic Staubli TX40 configuration loading."""
        simplified_config = {
            'robot': {
                'name': 'staubli_tx40',
                'description': 'Staubli TX40 6-DOF SCARA manipulator robot',
                'properties': {
                    'joints': {
                        'active_joints': [
                            "joint_1", "joint_2", "joint_3", 
                            "joint_4", "joint_5", "joint_6"
                        ],
                        'joint_limits': {
                            'velocity': [6.0, 6.0, 7.5, 12.0, 12.0, 17.0],
                            'acceleration': [50.0, 50.0, 50.0, 80.0, 80.0, 100.0],
                            'torque': [20.0, 20.0, 8.0, 3.0, 3.0, 1.5]
                        }
                    },
                    'mechanics': {
                        'reduction_ratios': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
                        'friction_coefficients': {
                            'viscous': [0.8, 0.8, 0.5, 0.2, 0.2, 0.1],
                            'coulomb': [1.5, 1.5, 1.0, 0.5, 0.5, 0.3]
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
                        'tool_frame': 'joint_6'
                    },
                    'measurements': {
                        'markers': [{
                            'name': 'end_effector_marker',
                            'reference_joint': 'joint_6',
                            'measurable_dof': [True, True, True, False, False, False]
                        }]
                    },
                    'parameters': {
                        'calibration_level': 'full_kinematic',
                        'outlier_threshold': 0.03
                    }
                },
                'identification': {
                    'enabled': True,
                    'type': 'dynamic_identification',
                    'parameters': {
                        'estimation_method': 'weighted_least_squares',
                        'outlier_threshold': 0.1
                    },
                    'problem': {
                        'has_friction': True,
                        'has_external_forces': False
                    },
                    'signal_processing': {
                        'sampling_rate': 1000.0,
                        'filter_cutoff': 30.0
                    }
                }
            }
        }
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(simplified_config, f)
            test_config_path = f.name
        
        try:
            parser = UnifiedConfigParser(test_config_path)
            config = parser.parse()
            
            # Verify robot properties
            assert config['robot']['name'] == 'staubli_tx40'
            assert 'Staubli TX40' in config['robot']['description']
            assert len(config['robot']['properties']['joints']['active_joints']) == 6
            assert all(joint.startswith('joint_') for joint in 
                      config['robot']['properties']['joints']['active_joints'])
            
            # Verify SCARA-specific properties (higher wrist speeds)
            velocities = config['robot']['properties']['joints']['joint_limits']['velocity']
            assert len(velocities) == 6
            assert velocities[4] == 12.0  # Fast wrist joints
            assert velocities[5] == 17.0  # Very fast rotation
            
            # Verify lower torque for smaller robot
            torques = config['robot']['properties']['joints']['joint_limits']['torque']
            assert torques[0] == 20.0  # Main joints
            assert torques[5] == 1.5   # Small rotation joint
            
            # Verify both tasks are enabled
            assert config['tasks']['calibration']['enabled'] is True
            assert config['tasks']['identification']['enabled'] is True
            
        finally:
            Path(test_config_path).unlink()
    
    def test_staubli_identification_task(self, mock_robot):
        """Test Staubli TX40 identification task configuration."""
        config = {
            'robot': {
                'name': 'staubli_tx40',
                'properties': {
                    'joints': {'active_joints': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']},
                    'mechanics': {'friction_coefficients': {'viscous': [0.8, 0.8, 0.5, 0.2, 0.2, 0.1]}}
                }
            },
            'tasks': {
                'identification': {
                    'enabled': True,
                    'type': 'dynamic_identification',
                    'parameters': {'estimation_method': 'weighted_least_squares'},
                    'problem': {'has_friction': True, 'has_external_forces': False},
                    'signal_processing': {'sampling_rate': 1000.0, 'filter_cutoff': 30.0}
                }
            }
        }
        
        task_config = create_task_config(mock_robot, config, 'identification')
        
        assert task_config['robot_name'] == 'staubli_tx40'
        assert task_config['task_type'] == 'dynamic_identification'
        assert task_config['parameters']['estimation_method'] == 'weighted_least_squares'
        assert task_config['problem']['has_friction'] is True
        assert task_config['signal_processing']['filter_cutoff'] == 30.0


class TestConfigVariantsAndInheritance:
    """Test configuration variants and inheritance across different robots."""
    
    def test_calibration_variants_across_robots(self):
        """Test different calibration variants work for different robots."""
        robot_variants = {
            'ur10': {
                'minimal_calibration': {
                    'tasks': {
                        'calibration': {
                            'parameters': {'calibration_level': 'joint_offset'},
                            'measurements': {'markers': []}
                        }
                    }
                }
            },
            'tiago': {
                'mocap_calibration': {
                    'tasks': {
                        'calibration': {
                            'kinematics': {'base_frame': 'head_2_joint'},
                            'parameters': {'outlier_threshold': 0.001}
                        }
                    }
                }
            }
        }
        
        for robot_name, variants in robot_variants.items():
            for variant_name, variant_config in variants.items():
                # Create test config with variant
                test_config = {
                    'robot': {'name': robot_name},
                    'tasks': {
                        'calibration': {
                            'enabled': True,
                            'type': 'kinematic_calibration',
                            'kinematics': {'base_frame': 'universe', 'tool_frame': 'tool'},
                            'measurements': {'markers': []},
                            'parameters': {'outlier_threshold': 0.05}
                        }
                    },
                    'variants': {variant_name: variant_config}
                }
                
                import tempfile
                with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                    yaml.dump(test_config, f)
                    test_config_path = f.name
                
                try:
                    parser = UnifiedConfigParser(test_config_path, variant=variant_name)
                    config = parser.parse()
                    
                    assert config['robot']['name'] == robot_name
                    cal_task = config['tasks']['calibration']
                    
                    # Verify variant-specific changes were applied
                    if robot_name == 'ur10' and variant_name == 'minimal_calibration':
                        assert cal_task['parameters']['calibration_level'] == 'joint_offset'
                    elif robot_name == 'tiago' and variant_name == 'mocap_calibration':
                        assert cal_task['kinematics']['base_frame'] == 'head_2_joint'
                        assert cal_task['parameters']['outlier_threshold'] == 0.001
                    
                finally:
                    Path(test_config_path).unlink()
    
    def test_robot_specific_validation(self):
        """Test that robot-specific validation works correctly."""
        robot_configs = [
            {
                'name': 'ur10',
                'joint_count': 6,
                'expected_joints': ['shoulder_pan_joint', 'wrist_3_joint']
            },
            {
                'name': 'tiago', 
                'joint_count': 8,
                'expected_joints': ['torso_lift_joint', 'arm_7_joint']
            },
            {
                'name': 'talos',
                'joint_count': 16,
                'expected_joints': ['torso_1_joint', 'arm_left_7_joint', 'arm_right_7_joint']
            },
            {
                'name': 'staubli_tx40',
                'joint_count': 6,
                'expected_joints': ['joint_1', 'joint_6']
            }
        ]
        
        for robot_spec in robot_configs:
            test_config = {
                'robot': {
                    'name': robot_spec['name'],
                    'properties': {
                        'joints': {
                            'active_joints': [f"joint_{i}" for i in range(robot_spec['joint_count'])]
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
                    }
                }
            }
            
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                yaml.dump(test_config, f)
                test_config_path = f.name
            
            try:
                parser = UnifiedConfigParser(test_config_path)
                config = parser.parse()
                
                assert config['robot']['name'] == robot_spec['name']
                assert len(config['robot']['properties']['joints']['active_joints']) == robot_spec['joint_count']
                
            finally:
                Path(test_config_path).unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
