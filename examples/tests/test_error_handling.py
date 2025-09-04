"""
Test error handling functionality.

Tests for the custom exception hierarchy and validation utilities.
"""

import pytest
import numpy as np

# Import from figaroh utils
from figaroh.utils.error_handling import (
    FigarohExampleError,
    RobotInitializationError,
    ConfigurationError,
    DataProcessingError,
    CalibrationError,
    IdentificationError,
    ValidationError,
    validate_robot_config,
    validate_trajectory_data,
    validate_numeric_range,
    validate_input_data,
    handle_identification_errors,
    safe_execute,
    ErrorContext
)


class TestCustomExceptions:
    """Test custom exception hierarchy."""
    
    def test_exception_inheritance(self):
        """Test that custom exceptions inherit properly."""
        assert issubclass(RobotInitializationError, FigarohExampleError)
        assert issubclass(ConfigurationError, FigarohExampleError)
        assert issubclass(DataProcessingError, FigarohExampleError)
        assert issubclass(CalibrationError, FigarohExampleError)
        assert issubclass(IdentificationError, FigarohExampleError)
        assert issubclass(ValidationError, FigarohExampleError)
    
    def test_exception_messages(self):
        """Test exception message handling."""
        message = "Test error message"
        error = RobotInitializationError(message)
        assert str(error) == message


class TestValidationFunctions:
    """Test validation utility functions."""
    
    def test_validate_robot_config_valid(self):
        """Test robot config validation with valid input."""
        valid_config = {
            'robot_name': 'Test Robot',
            'some_param': 'value'
        }
        
        # Should not raise exception
        validate_robot_config(valid_config)
    
    def test_validate_robot_config_invalid(self):
        """Test robot config validation with invalid input."""
        # Test non-dict input
        with pytest.raises(ValidationError):
            validate_robot_config("not a dict")
        
        # Test missing required field
        invalid_config = {'some_param': 'value'}
        with pytest.raises(ValidationError):
            validate_robot_config(invalid_config)
    
    def test_validate_trajectory_data_valid(self):
        """Test trajectory data validation with valid arrays."""
        n_samples, n_joints = 100, 6
        q = np.random.randn(n_samples, n_joints)
        qd = np.random.randn(n_samples, n_joints)
        qdd = np.random.randn(n_samples, n_joints)
        tau = np.random.randn(n_samples, n_joints)
        
        # Should not raise exception
        validate_trajectory_data(q, qd, qdd, tau)
    
    def test_validate_trajectory_data_invalid_shape(self):
        """Test trajectory data validation with invalid shapes."""
        q = np.random.randn(100, 6)
        qd = np.random.randn(50, 6)  # Wrong number of samples
        
        with pytest.raises(ValidationError):
            validate_trajectory_data(q, qd)
    
    def test_validate_trajectory_data_nan_values(self):
        """Test trajectory data validation with NaN values."""
        q = np.random.randn(100, 6)
        q[10, 2] = np.nan  # Insert NaN
        
        with pytest.raises(ValidationError):
            validate_trajectory_data(q)
    
    def test_validate_trajectory_data_inf_values(self):
        """Test trajectory data validation with infinite values."""
        q = np.random.randn(100, 6)
        qd = np.random.randn(100, 6)
        qd[5, 1] = np.inf  # Insert infinity
        
        with pytest.raises(ValidationError):
            validate_trajectory_data(q, qd)
    
    def test_validate_numeric_range_scalar(self):
        """Test numeric range validation with scalar values."""
        # Valid range
        validate_numeric_range(5.0, min_val=0.0, max_val=10.0)
        
        # Below minimum
        with pytest.raises(ValidationError):
            validate_numeric_range(-1.0, min_val=0.0, max_val=10.0)
        
        # Above maximum
        with pytest.raises(ValidationError):
            validate_numeric_range(15.0, min_val=0.0, max_val=10.0)
    
    def test_validate_numeric_range_array(self):
        """Test numeric range validation with arrays."""
        values = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        
        # Valid range
        validate_numeric_range(values, min_val=0.0, max_val=10.0)
        
        # Array with value below minimum
        values_low = np.array([1.0, 2.0, -1.0, 4.0, 5.0])
        with pytest.raises(ValidationError):
            validate_numeric_range(values_low, min_val=0.0, max_val=10.0)


class TestDecorators:
    """Test decorator functions."""
    
    def test_validate_input_data_decorator(self):
        """Test input data validation decorator."""
        
        @validate_input_data
        def process_data(data):
            return data * 2
        
        # Valid data
        valid_data = np.array([1.0, 2.0, 3.0])
        result = process_data(valid_data)
        np.testing.assert_array_equal(result, [2.0, 4.0, 6.0])
        
        # Invalid data with NaN
        invalid_data = np.array([1.0, np.nan, 3.0])
        with pytest.raises(DataProcessingError):
            process_data(invalid_data)
    
    def test_handle_identification_errors_decorator(self):
        """Test identification error handling decorator."""
        
        @handle_identification_errors
        def failing_identification():
            raise ValueError("Identification failed")
        
        with pytest.raises(IdentificationError):
            failing_identification()
    
    def test_safe_execute_success(self):
        """Test safe_execute with successful function."""
        def successful_function(x, y):
            return x + y
        
        success, result = safe_execute(successful_function, 2, 3)
        assert success is True
        assert result == 5
    
    def test_safe_execute_failure(self):
        """Test safe_execute with failing function."""
        def failing_function():
            raise ValueError("This function always fails")
        
        success, error = safe_execute(failing_function)
        assert success is False
        assert isinstance(error, ValueError)


class TestErrorContext:
    """Test ErrorContext context manager."""
    
    def test_error_context_success(self):
        """Test ErrorContext with successful operation."""
        with ErrorContext("test operation") as ctx:
            result = 2 + 2
        
        assert ctx.error is None
        assert result == 4
    
    def test_error_context_with_error_raise(self):
        """Test ErrorContext with error and raise_on_error=True."""
        with pytest.raises(ValueError):
            with ErrorContext("test operation", raise_on_error=True):
                raise ValueError("Test error")
    
    def test_error_context_with_error_suppress(self):
        """Test ErrorContext with error and raise_on_error=False."""
        with ErrorContext("test operation", raise_on_error=False) as ctx:
            raise ValueError("Test error")
        
        assert isinstance(ctx.error, ValueError)


if __name__ == '__main__':
    pytest.main([__file__])
