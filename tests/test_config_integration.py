"""
Test configuration files for testing the unified config parser.
"""

import pytest
import tempfile
import yaml
from pathlib import Path


@pytest.fixture
def sample_robot_configs():
    """Create sample configuration files for testing."""
    configs = {}

    # Base template
    configs["base_template"] = {
        "robot": {
            "name": "${ROBOT_NAME}",
            "properties": {
                "joints": {"active_joints": []},
                "mechanics": {
                    "friction_coefficients": {"viscous": []},
                    "reduction_ratios": [],
                },
            },
        },
        "tasks": {
            "calibration": {
                "enabled": False,
                "type": "kinematic_calibration",
                "parameters": {"outlier_threshold": 0.05},
                "kinematics": {"base_frame": "base", "tool_frame": "tool"},
                "measurements": {"markers": []},
                "data": {"number_of_samples": 100},
            },
            "identification": {
                "enabled": False,
                "type": "dynamic_identification",
                "parameters": {"estimation_method": "weighted_least_squares"},
                "problem": {"has_friction": True, "has_external_forces": False},
                "signal_processing": {
                    "sampling_frequency": 1000.0,
                    "cutoff_frequency": 50.0,
                    "filter_type": "butterworth",
                    "differentiation_method": "gradient",
                },
                "data": {"sampling_rate": 1000.0},
            },
        },
    }

    # Test robot config
    configs["test_robot"] = {
        "extends": "templates/base.yaml",
        "meta": {"schema_version": "2.0"},
        "robot": {
            "name": "test_robot",
            "properties": {
                "joints": {"active_joints": ["joint1", "joint2", "joint3"]},
                "mechanics": {
                    "friction_coefficients": {"viscous": [0.1, 0.15, 0.2]},
                    "reduction_ratios": [50.0, 50.0, 30.0],
                },
                "coupling": {"has_coupled_joints": False},
            },
        },
        "tasks": {
            "calibration": {
                "enabled": True,
                "parameters": {
                    "outlier_threshold": 0.02,
                    "calibration_level": "full_kinematic",
                },
                "kinematics": {
                    "base_frame": "robot_base",
                    "tool_frame": "end_effector",
                },
                "measurements": {
                    "markers": [
                        {
                            "ref_joint": "joint3",
                            "position": [0, 0, 0.1],
                            "measure": [True, True, True, False, False, False],
                        }
                    ]
                },
            },
            "identification": {
                "enabled": True,
                "parameters": {
                    "estimation_method": "recursive_least_squares",
                    "regularization_factor": 1e-6,
                },
                "problem": {"has_friction": True, "has_external_forces": False},
                "signal_processing": {
                    "sampling_frequency": 1000.0,
                    "cutoff_frequency": 50.0,
                    "filter_type": "butterworth",
                    "differentiation_method": "gradient",
                },
                "data": {"sampling_rate": 1000.0},
            },
        },
        "environment": {"gravity": [0, 0, -9.81], "temperature": 20.0},
        "custom": {"manufacturer": "Test Robotics Inc.", "model_year": 2024},
        "variants": {
            "minimal_calibration": {
                "tasks": {
                    "calibration": {
                        "parameters": {"calibration_level": "joint_offset"},
                        "measurements": {"markers": []},
                    }
                }
            },
            "high_precision": {
                "tasks": {
                    "calibration": {
                        "parameters": {"outlier_threshold": 0.005},
                        "data": {"number_of_samples": 500},
                    },
                    "identification": {"parameters": {"regularization_factor": 1e-8}},
                }
            },
        },
    }

    return configs


@pytest.fixture
def temp_config_structure(sample_robot_configs):
    """Create temporary directory structure with config files."""
    with tempfile.TemporaryDirectory() as temp_dir:
        base_path = Path(temp_dir)

        # Create directory structure
        templates_dir = base_path / "templates"
        templates_dir.mkdir()

        configs_dir = base_path / "configs"
        configs_dir.mkdir()

        # Write template files
        template_file = templates_dir / "base.yaml"
        with open(template_file, "w") as f:
            yaml.dump(sample_robot_configs["base_template"], f)

        # Write test robot config
        robot_file = configs_dir / "test_robot.yaml"
        with open(robot_file, "w") as f:
            yaml.dump(sample_robot_configs["test_robot"], f)

        yield {
            "base_path": base_path,
            "template_file": template_file,
            "robot_file": robot_file,
        }


class TestConfigurationIntegration:
    """Integration tests with realistic configuration scenarios."""

    def test_complete_workflow(self, temp_config_structure):
        """Test complete workflow from config loading to task creation."""
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
        robot_file = temp_config_structure["robot_file"]

        # Parse configuration
        parser = UnifiedConfigParser(robot_file)
        config = parser.parse()

        # Verify structure
        assert config["robot"]["name"] == "test_robot"
        assert len(config["robot"]["properties"]["joints"]["active_joints"]) == 3
        assert config["tasks"]["calibration"]["enabled"] is True

        # Create task-specific config
        cal_config = create_task_config(robot, config, "calibration")

        # Verify task config creation
        assert cal_config["robot_name"] == "test_robot"
        assert cal_config["task_type"] == "kinematic_calibration"
        assert "joints" in cal_config
        assert "mechanics" in cal_config
        assert "parameters" in cal_config
        assert cal_config["parameters"]["outlier_threshold"] == 0.02

    def test_variant_workflow(self, temp_config_structure):
        """Test workflow with variant application."""
        from figaroh.utils.config_parser import UnifiedConfigParser, create_task_config

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
        robot_file = temp_config_structure["robot_file"]

        # Parse with variant
        parser = UnifiedConfigParser(robot_file, variant="minimal_calibration")
        config = parser.parse()

        # Verify variant was applied
        cal_task = config["tasks"]["calibration"]
        assert cal_task["parameters"]["calibration_level"] == "joint_offset"
        assert len(cal_task["measurements"]["markers"]) == 0

        # Create task config
        cal_config = create_task_config(robot, config, "calibration")
        assert cal_config["parameters"]["calibration_level"] == "joint_offset"

    def test_error_handling_integration(self, temp_config_structure):
        """Test error handling in realistic scenarios."""
        from figaroh.utils.config_parser import UnifiedConfigParser, ConfigurationError

        # Test circular inheritance
        circular_config = {
            "extends": "circular_b.yaml",
            "robot": {"name": "circular_a"},
        }

        circular_a_file = (
            temp_config_structure["base_path"] / "configs" / "circular_a.yaml"
        )
        with open(circular_a_file, "w") as f:
            yaml.dump(circular_config, f)

        circular_b_config = {
            "extends": "circular_a.yaml",
            "robot": {"name": "circular_b"},
        }

        circular_b_file = (
            temp_config_structure["base_path"] / "configs" / "circular_b.yaml"
        )
        with open(circular_b_file, "w") as f:
            yaml.dump(circular_b_config, f)

        # Should detect circular inheritance (via recursion limit)
        with pytest.raises(
            ConfigurationError, match="Failed to parse configuration|maximum recursion"
        ):
            parser = UnifiedConfigParser(circular_a_file)
            parser.parse()

    def test_performance_large_config(self, temp_config_structure):
        """Test performance with large configuration files."""
        from figaroh.utils.config_parser import UnifiedConfigParser
        import time

        # Create large config
        large_config = {"robot": {"name": "large_robot"}, "tasks": {}}

        # Add many tasks
        for i in range(100):
            large_config["tasks"][f"task_{i}"] = {
                "enabled": i % 2 == 0,
                "type": "test_task",
                "parameters": {f"param_{j}": f"value_{j}" for j in range(20)},
                "data": {f"data_{j}": list(range(10)) for j in range(10)},
            }

        large_file = temp_config_structure["base_path"] / "large_config.yaml"
        with open(large_file, "w") as f:
            yaml.dump(large_config, f)

        # Time the parsing
        start_time = time.time()
        parser = UnifiedConfigParser(large_file)
        config = parser.parse()
        end_time = time.time()

        # Should complete in reasonable time (< 1 second)
        assert end_time - start_time < 1.0
        assert len(config["tasks"]) == 100


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
