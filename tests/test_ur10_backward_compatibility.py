"""
Test backward compatibility between legacy and unified configuration parsers for UR10.
"""

import pytest


class TestUR10BackwardCompatibility:
    """Test backward compatibility for UR10 configurations."""

    def test_ur10_calibration_compatibility(self):
        """Test backward compatibility: compare legacy get_param_from_yaml with
        unified parser for UR10 calibration."""
        import yaml
        from pathlib import Path
        from figaroh.calibration.calibration_tools import get_param_from_yaml
        from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config
        from figaroh.tools.robot import load_robot

        # Define paths relative to test file
        test_dir = Path(__file__).parent
        examples_dir = test_dir.parent / "examples"
        ur10_dir = examples_dir / "ur10"
        config_dir = ur10_dir / "config"

        legacy_config_path = config_dir / "ur10_config_new.yaml"
        unified_config_path = config_dir / "ur10_unified_config_simple.yaml"
        urdf_path = ur10_dir / "urdf" / "ur10_robot.urdf"
        models_dir = examples_dir.parent / "models"

        # Skip test if files don't exist
        required_paths = [legacy_config_path, unified_config_path, urdf_path]
        if not all(p.exists() for p in required_paths):
            pytest.skip("Required UR10 config or URDF files not found")

        try:
            # Load UR10 robot model
            ur10 = load_robot(
                str(urdf_path),
                package_dirs=str(models_dir),
                load_by_urdf=True,
            )

            # Load legacy configuration
            with open(legacy_config_path, "r") as f:
                legacy_config_data = yaml.safe_load(f)

            # Get calibration parameters using legacy method (direct function)
            legacy_result = get_param_from_yaml(ur10, legacy_config_data["calibration"])

            # Parse unified configuration
            unified_parser = UnifiedConfigParser(str(unified_config_path))
            unified_config = unified_parser.parse()

            # Get calibration parameters using new method
            unified_result = create_task_config(ur10, unified_config, "calibration")

            # Compare key parameters
            self._compare_calibration_configs(legacy_result, unified_result)

            print("✅ UR10 calibration backward compatibility test PASSED!")
            print(f"Legacy robot name: {legacy_result.get('robot_name', 'N/A')}")
            print(f"Unified robot name: {unified_result.get('robot_name', 'N/A')}")

        except ImportError as e:
            pytest.skip(f"Required modules not available: {e}")

    # No UR10 legacy-vs-unified *identification* comparison: no legacy UR10
    # config in examples/ur10/config has the problem/processing/TLS sections
    # the legacy identification parser requires (ur10_config_new.yaml keeps
    # only robot_params), so there is nothing valid to compare against
    # (figaroh-examples#45).

    def _compare_calibration_configs(self, legacy_result, unified_result):
        """Compare calibration configuration results between legacy and
        unified parsers."""
        # Both should be dictionaries
        assert isinstance(legacy_result, dict), "Legacy result should be a dictionary"
        assert isinstance(unified_result, dict), "Unified result should be a dictionary"

        # Both should have robot name
        assert "robot_name" in legacy_result, "Legacy result missing robot_name"
        assert "robot_name" in unified_result, "Unified result missing robot_name"

        # Robot names should match (allowing for case differences)
        legacy_name = legacy_result["robot_name"].lower()
        unified_name = unified_result["robot_name"].lower()
        # Both should contain 'ur' (ur10, ur10e, etc.)
        assert (
            "ur" in legacy_name and "ur" in unified_name
        ), f"Robot names should contain 'ur': {legacy_name}, {unified_name}"

        # Both should have task type information
        if "task_type" in unified_result:
            assert "calibration" in unified_result["task_type"].lower()

        # Compare calibration model/level if available
        legacy_model = legacy_result.get("calib_model")
        if legacy_model and "parameters" in unified_result:
            unified_params = unified_result["parameters"]
            if "calibration_level" in unified_params:
                unified_level = str(unified_params["calibration_level"]).lower()
                # Both should indicate full parameter calibration
                assert (
                    "full" in legacy_model.lower() and "full" in unified_level
                ), f"Calibration levels should match: {legacy_model} vs {unified_level}"

        # Compare frame information
        legacy_start = legacy_result.get("start_frame")
        legacy_end = legacy_result.get("end_frame")

        if "kinematics" in unified_result:
            unified_kin = unified_result["kinematics"]
            unified_base = unified_kin.get("base_frame")
            unified_tool = unified_kin.get("tool_frame")

            # Compare base/start frames
            if legacy_start and unified_base:
                assert (
                    legacy_start == unified_base
                ), f"Base frames should match: {legacy_start} vs {unified_base}"

            # Compare tool/end frames
            if legacy_end and unified_tool:
                assert (
                    legacy_end == unified_tool
                ), f"Tool frames should match: {legacy_end} vs {unified_tool}"

        # Compare marker information
        legacy_markers = legacy_result.get("NbMarkers")
        if legacy_markers and "measurements" in unified_result:
            unified_measurements = unified_result["measurements"]
            if "markers" in unified_measurements:
                unified_markers = len(unified_measurements["markers"])
                assert (
                    legacy_markers == unified_markers
                ), f"Number of markers should match: {legacy_markers} vs {unified_markers}"

        print("✓ Calibration config comparison passed")
        print(f"  Legacy: {legacy_result.get('calib_model', 'N/A')} model")
        print(f"  Unified: {unified_result.get('task_type', 'N/A')} task")
        print(f"  Frames: {legacy_start} -> {legacy_end}")
