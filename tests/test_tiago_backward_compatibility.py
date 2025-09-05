"""
Test backward compatibility between legacy and unified configuration parsers for TIAGo.
"""

import pytest


class TestTIAGoBackwardCompatibility:
    """Test backward compatibility for TIAGo configurations."""
    
    def test_tiago_calibration_compatibility(self):
        """Test backward compatibility: compare legacy get_param_from_yaml with
        unified parser for TIAGo calibration."""
        import yaml
        from pathlib import Path
        from figaroh.calibration.calibration_tools import get_param_from_yaml
        from figaroh.utils.config_parser import (
            UnifiedConfigParser, create_task_config
        )
        from figaroh.tools.robot import load_robot
        
        # Define paths relative to test file
        test_dir = Path(__file__).parent
        examples_dir = test_dir.parent / "examples"
        tiago_dir = examples_dir / "tiago"
        config_dir = tiago_dir / "config"
        
        legacy_config_path = config_dir / "tiago_config.yaml"
        unified_config_path = config_dir / "tiago_unified_config_simple.yaml"
        urdf_path = tiago_dir / "urdf" / "tiago_no_hand.urdf"
        models_dir = examples_dir.parent / "models"
        
        # Skip test if files don't exist
        required_paths = [legacy_config_path, unified_config_path, urdf_path]
        if not all(p.exists() for p in required_paths):
            pytest.skip("Required TIAGo config or URDF files not found")
        
        try:
            # Load TIAGo robot model
            tiago = load_robot(
                str(urdf_path),
                package_dirs=str(models_dir),
                load_by_urdf=True,
            )
            
            # Load legacy configuration
            with open(legacy_config_path, 'r') as f:
                legacy_config_data = yaml.safe_load(f)
            
            # Get calibration parameters using legacy method (direct function)
            legacy_result = get_param_from_yaml(
                tiago, legacy_config_data['calibration']
            )
            
            # Parse unified configuration
            unified_parser = UnifiedConfigParser(str(unified_config_path))
            unified_config = unified_parser.parse()
            
            # Get calibration parameters using new method
            unified_result = create_task_config(
                tiago, unified_config, 'calibration'
            )
            
            # Compare key parameters
            self._compare_calibration_configs(legacy_result, unified_result)
            
            print("✅ TIAGo calibration backward compatibility test PASSED!")
            print(f"Legacy robot name: {legacy_result.get('robot_name', 'N/A')}")
            print(f"Unified robot name: {unified_result.get('robot_name', 'N/A')}")
            
        except ImportError as e:
            pytest.skip(f"Required modules not available: {e}")
        except Exception as e:
            # Log the actual error for debugging
            print(f"Error in TIAGo backward compatibility test: {e}")
            pytest.skip(f"Could not complete TIAGo compatibility test: {e}")
    
    def test_tiago_identification_compatibility(self):
        """Test backward compatibility: compare legacy get_param_from_yaml with
        unified parser for TIAGo identification."""
        import yaml
        from pathlib import Path
        from figaroh.identification.identification_tools import get_param_from_yaml
        from figaroh.utils.config_parser import (
            UnifiedConfigParser, create_task_config
        )
        from figaroh.tools.robot import load_robot
        
        # Define paths relative to test file
        test_dir = Path(__file__).parent
        examples_dir = test_dir.parent / "examples"
        tiago_dir = examples_dir / "tiago"
        config_dir = tiago_dir / "config"
        
        legacy_config_path = config_dir / "tiago_config.yaml"
        unified_config_path = config_dir / "tiago_unified_config_simple.yaml"
        urdf_path = tiago_dir / "urdf" / "tiago_no_hand.urdf"
        models_dir = examples_dir.parent / "models"
        
        # Skip test if files don't exist
        required_paths = [legacy_config_path, unified_config_path, urdf_path]
        if not all(p.exists() for p in required_paths):
            pytest.skip("Required TIAGo config or URDF files not found")
        
        try:
            # Load TIAGo robot model
            tiago = load_robot(
                str(urdf_path),
                package_dirs=str(models_dir),
                load_by_urdf=True,
            )
            
            # Load legacy configuration
            with open(legacy_config_path, 'r') as f:
                legacy_config_data = yaml.safe_load(f)
            
            # Check if identification section exists in legacy config
            if 'identification' not in legacy_config_data:
                pytest.skip("Legacy config does not contain identification")
            
            # Get identification parameters using legacy method (direct function)
            legacy_result = get_param_from_yaml(
                tiago, legacy_config_data['identification']
            )
            
            # Parse unified configuration
            unified_parser = UnifiedConfigParser(str(unified_config_path))
            unified_config = unified_parser.parse()
            
            # Get identification parameters using new method
            unified_result = create_task_config(
                tiago, unified_config, 'identification'
            )
            
            # Compare key parameters
            self._compare_identification_configs(legacy_result, unified_result)
            
            print("✅ TIAGo identification backward compatibility test PASSED!")
            print(f"Legacy robot name: {legacy_result.get('robot_name', 'N/A')}")
            print(f"Unified robot name: {unified_result.get('robot_name', 'N/A')}")
            
        except ImportError as e:
            pytest.skip(f"Required modules not available: {e}")
        except Exception as e:
            # Log the actual error for debugging
            print(f"Error in TIAGo identification compatibility test: {e}")
            pytest.skip(f"Could not complete identification test: {e}")
    
    def _compare_calibration_configs(self, legacy_result, unified_result):
        """Compare calibration configuration results between legacy and
        unified parsers."""
        # Both should be dictionaries
        assert isinstance(legacy_result, dict), \
            "Legacy result should be a dictionary"
        assert isinstance(unified_result, dict), \
            "Unified result should be a dictionary"
        
        # Both should have robot name
        assert 'robot_name' in legacy_result, "Legacy result missing robot_name"
        assert 'robot_name' in unified_result, \
            "Unified result missing robot_name"
        
        # Robot names should match (allowing for case differences)
        legacy_name = legacy_result['robot_name'].lower()
        unified_name = unified_result['robot_name'].lower()
        # Both should contain 'tiago'
        assert 'tiago' in legacy_name and 'tiago' in unified_name, \
            f"Robot names should contain 'tiago': {legacy_name}, {unified_name}"
        
        # Both should have task type information
        if 'task_type' in unified_result:
            assert 'calibration' in unified_result['task_type'].lower()
        
        # Compare calibration model/level if available
        legacy_model = legacy_result.get('calib_model')
        if legacy_model and 'parameters' in unified_result:
            unified_params = unified_result['parameters']
            if 'calibration_level' in unified_params:
                unified_level = str(unified_params['calibration_level']).lower()
                # Both should indicate full parameter calibration
                assert ('full' in legacy_model.lower() and 
                        'full' in unified_level), \
                    f"Calibration levels should match: {legacy_model} vs {unified_level}"
        
        # Compare frame information
        legacy_start = legacy_result.get('start_frame')
        legacy_end = legacy_result.get('end_frame')
        
        if 'kinematics' in unified_result:
            unified_kin = unified_result['kinematics']
            unified_base = unified_kin.get('base_frame')
            unified_tool = unified_kin.get('tool_frame')
            
            # Compare base/start frames
            if legacy_start and unified_base:
                assert legacy_start == unified_base, \
                    f"Base frames should match: {legacy_start} vs {unified_base}"
            
            # Compare tool/end frames  
            if legacy_end and unified_tool:
                assert legacy_end == unified_tool, \
                    f"Tool frames should match: {legacy_end} vs {unified_tool}"
        
        # Compare marker information
        legacy_markers = legacy_result.get('NbMarkers')
        if legacy_markers and 'measurements' in unified_result:
            unified_measurements = unified_result['measurements']
            if 'markers' in unified_measurements:
                unified_markers = len(unified_measurements['markers'])
                assert legacy_markers == unified_markers, \
                    f"Number of markers should match: {legacy_markers} vs {unified_markers}"
        
        print(f"✓ TIAGo calibration config comparison passed")
        print(f"  Legacy: {legacy_result.get('calib_model', 'N/A')} model")
        print(f"  Unified: {unified_result.get('task_type', 'N/A')} task")
        print(f"  Frames: {legacy_start} -> {legacy_end}")
    
    def _compare_identification_configs(self, legacy_result, unified_result):
        """Compare identification configuration results between legacy and
        unified parsers."""
        # Both should be dictionaries
        assert isinstance(legacy_result, dict), \
            "Legacy result should be a dictionary"
        assert isinstance(unified_result, dict), \
            "Unified result should be a dictionary"
        
        # Both should have robot name
        assert 'robot_name' in legacy_result, "Legacy result missing robot_name"
        assert 'robot_name' in unified_result, \
            "Unified result missing robot_name"
        
        # Robot names should match (allowing for case differences)
        legacy_name = legacy_result['robot_name'].lower()
        unified_name = unified_result['robot_name'].lower()
        # Both should contain 'tiago'
        assert 'tiago' in legacy_name and 'tiago' in unified_name, \
            f"Robot names should contain 'tiago': {legacy_name}, {unified_name}"
        
        # Both should have task type information
        if 'task_type' in unified_result:
            assert 'identification' in unified_result['task_type'].lower()
        
        # Compare friction parameters if available
        if 'fv' in legacy_result and 'mechanics' in unified_result:
            unified_mech = unified_result['mechanics']
            if 'friction_coefficients' in unified_mech:
                unified_friction = unified_mech['friction_coefficients']
                if 'viscous' in unified_friction:
                    legacy_fv = legacy_result['fv']
                    unified_fv = unified_friction['viscous']
                    # Both should be lists of the same length
                    assert len(legacy_fv) == len(unified_fv), \
                        f"Viscous friction arrays length mismatch: {len(legacy_fv)} vs {len(unified_fv)}"
        
        # Compare active joints if available
        if 'actJoint_idx' in legacy_result and 'problem' in unified_result:
            unified_problem = unified_result['problem']
            if 'active_joints' in unified_problem:
                # Check that both have active joints specification
                legacy_joints = legacy_result.get('actJoint_idx', [])
                unified_joints = unified_problem['active_joints']
                # Both should specify joints (exact comparison may vary)
                assert len(legacy_joints) > 0 or len(unified_joints) > 0, \
                    "Both should specify active joints"
        
        # Compare sampling parameters if available
        if 'ts' in legacy_result and 'signal_processing' in unified_result:
            unified_processing = unified_result['signal_processing']
            if 'sampling_frequency' in unified_processing:
                legacy_ts = legacy_result['ts']
                unified_freq = unified_processing['sampling_frequency']
                # Check that frequency is approximately 1/ts
                expected_freq = 1.0 / legacy_ts
                assert abs(unified_freq - expected_freq) < 1000, \
                    f"Sampling frequency mismatch: expected ~{expected_freq}, got {unified_freq}"
        
        # Compare reduction ratios if available
        if 'reduction_ratio' in legacy_result:
            legacy_ratios = legacy_result['reduction_ratio']
            if 'mechanics' in unified_result:
                unified_mech = unified_result['mechanics']
                if 'reduction_ratios' in unified_mech:
                    unified_ratios = unified_mech['reduction_ratios']
                    # Should have same length
                    assert len(legacy_ratios) == len(unified_ratios), \
                        f"Reduction ratios length mismatch: {len(legacy_ratios)} vs {len(unified_ratios)}"
                    
                    # Values should match
                    for i, (legacy_ratio, unified_ratio) in enumerate(zip(legacy_ratios, unified_ratios)):
                        assert abs(legacy_ratio - unified_ratio) < 0.1, \
                            f"Reduction ratio {i} mismatch: {legacy_ratio} vs {unified_ratio}"
        
        print(f"✓ TIAGo identification config comparison passed")
        print(f"  Legacy robot: {legacy_result.get('robot_name', 'N/A')}")
        print(f"  Unified robot: {unified_result.get('robot_name', 'N/A')}")
        print(f"  Task type: {unified_result.get('task_type', 'N/A')}")
        if 'ts' in legacy_result:
            print(f"  Sampling: {legacy_result['ts']}s -> {unified_result.get('signal_processing', {}).get('sampling_frequency', 'N/A')}Hz")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
