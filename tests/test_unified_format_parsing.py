"""
Unified-format (extends: + tasks.*) parsing tests.

The legacy flat format is gone from the examples; the tests that exercised
its conversion lived in test_backward_compatibility.py (figaroh-examples#125).
"""

import pytest
import yaml
import tempfile
from pathlib import Path


class TestUnifiedFormatParsing:
    """Parse unified configurations into task configs."""

    def test_unified_format_parsing(self):
        """Test parsing unified format configurations."""
        from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config

        # Mock robot
        class MockRobot:
            def __init__(self):
                self.model = type(
                    "Model",
                    (),
                    {
                        "name": "test_robot",
                        "frames": [
                            type("Frame", (), {"name": "base"}),
                            type("Frame", (), {"name": "joint1"}),
                            type("Frame", (), {"name": "joint2"}),
                            type("Frame", (), {"name": "joint3"}),
                            type("Frame", (), {"name": "tool"}),
                        ],
                    },
                )()
                self.q0 = [0.0, 0.0, 0.0]

        robot = MockRobot()

        # Unified format configuration
        unified_config = {
            "meta": {"schema_version": "2.0"},
            "robot": {
                "name": "test_robot",
                "properties": {
                    "joints": {
                        "active_joints": ["joint1", "joint2", "joint3"],
                        "joint_limits": {
                            "lower": [-3.14, -1.57, -2.0],
                            "upper": [3.14, 1.57, 2.0],
                        },
                    },
                    "mechanics": {
                        "friction_coefficients": {
                            "viscous": [0.1, 0.12, 0.08],
                            "coulomb": [0.05, 0.04, 0.06],
                        },
                        "reduction_ratios": [50.0, 50.0, 30.0],
                    },
                },
            },
            "tasks": {
                "calibration": {
                    "enabled": True,
                    "type": "kinematic_calibration",
                    "parameters": {
                        "calibration_level": "full_kinematic",
                        "outlier_threshold": 0.02,
                        "max_iterations": 100,
                    },
                    "kinematics": {
                        "base_frame": "robot_base",
                        "tool_frame": "end_effector",
                    },
                    "measurements": {
                        "markers": [
                            {
                                "ref_joint": "joint3",
                                "position": [0.0, 0.0, 0.1],
                                "measure": [True, True, True, False, False, False],
                            }
                        ]
                    },
                    "data": {"number_of_samples": 200, "sampling_rate": 125.0},
                },
                "identification": {
                    "enabled": True,
                    "type": "dynamic_identification",
                    "parameters": {
                        "estimation_method": "recursive_least_squares",
                        "regularization_factor": 1e-6,
                        "outlier_threshold": 0.1,
                    },
                    "problem": {"has_friction": True, "has_external_forces": False},
                    "signal_processing": {
                        "sampling_frequency": 1000.0,
                        "cutoff_frequency": 50.0,
                        "filter_type": "butterworth",
                        "differentiation_method": "gradient",
                    },
                    "data": {"sampling_rate": 1000.0, "filter_cutoff": 50.0},
                },
            },
            "environment": {"gravity": [0, 0, -9.81], "temperature": 22.0},
            "custom": {"manufacturer": "Test Robotics", "model": "TR-3000"},
        }

        # Create temporary file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(unified_config, f)
            config_file = f.name

        try:
            # Parse configuration
            parser = UnifiedConfigParser(config_file)
            config = parser.parse()

            # Verify parsing
            assert config["robot"]["name"] == "test_robot"
            assert len(config["robot"]["properties"]["joints"]["active_joints"]) == 3
            assert config["tasks"]["calibration"]["enabled"] is True
            assert config["tasks"]["identification"]["enabled"] is True

            # Create task configurations
            cal_config = create_task_config(robot, config, "calibration")
            id_config = create_task_config(robot, config, "identification")

            # Verify task configurations
            assert cal_config["robot_name"] == "test_robot"
            assert cal_config["task_type"] == "kinematic_calibration"
            assert cal_config["parameters"]["outlier_threshold"] == 0.02

            assert id_config["robot_name"] == "test_robot"
            assert id_config["task_type"] == "dynamic_identification"
            assert (
                id_config["parameters"]["estimation_method"]
                == "recursive_least_squares"
            )

        finally:
            # Clean up
            Path(config_file).unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
