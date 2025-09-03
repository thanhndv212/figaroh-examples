"""
Data processing utilities for FIGAROH examples.

This module provides common data processing functions with optimized
implementations and consistent error handling across robot examples.
"""

import numpy as np
import pandas as pd
from scipy import signal
from typing import Dict, List, Optional, Tuple, Union
import logging
from functools import lru_cache
from concurrent.futures import ThreadPoolExecutor, as_completed

from .error_handling import (
    DataProcessingError,
    validate_input_data
)

logger = logging.getLogger(__name__)


class DataProcessor:
    """
    Centralized data processing utilities for robot trajectories.
    
    This class provides optimized implementations with caching,
    vectorization, and parallel processing for improved performance.
    """
    
    # Class-level cache for frequently accessed data
    _cache = {}
    _cache_size_limit = 100  # Maximum number of cached items
    
    @classmethod
    def clear_cache(cls):
        """Clear the internal data cache."""
        cls._cache.clear()
        logger.info("DataProcessor cache cleared")
    
    @classmethod
    def get_cache_stats(cls) -> Dict[str, int]:
        """Get cache statistics."""
        return {
            'cache_size': len(cls._cache),
            'cache_limit': cls._cache_size_limit
        }
    
    @staticmethod
    @lru_cache(maxsize=128)
    def _cached_filter_design(
        filter_type: str,
        cutoff: float,
        fs: float,
        order: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Cached filter design for reuse across multiple filtering operations.
        
        This significantly improves performance when applying the same filter
        to multiple datasets.
        """
        if filter_type == 'butterworth':
            return signal.butter(order, cutoff, fs=fs, btype='low')
        elif filter_type == 'chebyshev':
            return signal.cheby1(order, 0.5, cutoff, fs=fs, btype='low')
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")
    
    @staticmethod
    @validate_input_data
    def vectorized_differentiation(
        data: np.ndarray,
        dt: float = 0.01,
        method: str = 'gradient'
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Vectorized computation of velocities and accelerations.
        
        Uses numpy's optimized gradient function for improved performance
        compared to manual finite differences.
        
        Args:
            data: Position data (n_samples, n_joints)
            dt: Time step
            method: Differentiation method ('gradient' or 'finite_diff')
            
        Returns:
            Tuple of (velocities, accelerations)
        """
        if method == 'gradient':
            # Vectorized gradient computation
            velocities = np.gradient(data, dt, axis=0)
            accelerations = np.gradient(velocities, dt, axis=0)
        elif method == 'finite_diff':
            # Traditional finite differences (for comparison)
            velocities = np.diff(data, axis=0) / dt
            accelerations = np.diff(velocities, axis=0) / dt
            # Pad to maintain shape
            velocities = np.vstack([velocities, velocities[-1:]])
            accelerations = np.vstack([accelerations, accelerations[-1:]])
        else:
            raise ValueError(f"Unknown method: {method}")
        
        return velocities, accelerations
    
    @staticmethod
    @validate_input_data
    def parallel_filtering(
        data_list: List[np.ndarray],
        cutoff: float = 10.0,
        fs: float = 100.0,
        filter_type: str = 'butterworth',
        order: int = 4,
        max_workers: Optional[int] = None
    ) -> List[np.ndarray]:
        """
        Apply filtering to multiple datasets in parallel.
        
        This is especially useful when processing multiple robot trajectories
        or multi-joint data where each joint can be filtered independently.
        
        Args:
            data_list: List of data arrays to filter
            cutoff: Cutoff frequency
            fs: Sampling frequency 
            filter_type: Filter type ('butterworth' or 'chebyshev')
            order: Filter order
            max_workers: Maximum number of parallel workers
            
        Returns:
            List of filtered data arrays
        """
        # Get filter coefficients (cached)
        b, a = DataProcessor._cached_filter_design(
            filter_type, cutoff, fs, order
        )
        
        def filter_single_array(data):
            """Filter a single array."""
            if data.ndim == 1:
                return signal.filtfilt(b, a, data)
            else:
                # Filter each column independently
                filtered = np.zeros_like(data)
                for i in range(data.shape[1]):
                    filtered[:, i] = signal.filtfilt(b, a, data[:, i])
                return filtered
        
        # Use parallel processing for large datasets
        if len(data_list) > 1 and data_list[0].size > 1000:
            with ThreadPoolExecutor(max_workers=max_workers) as executor:
                futures = {
                    executor.submit(filter_single_array, data): i
                    for i, data in enumerate(data_list)
                }
                
                filtered_data = [None] * len(data_list)
                for future in as_completed(futures):
                    idx = futures[future]
                    filtered_data[idx] = future.result()
                
                return filtered_data
        else:
            # Sequential processing for small datasets
            return [filter_single_array(data) for data in data_list]
    
    @staticmethod
    @validate_input_data
    def load_csv_data(
        file_paths: Union[str, List[str]],
        validate_shapes: bool = True
    ) -> Union[np.ndarray, List[np.ndarray]]:
        """
        Load CSV data with validation and error handling.
        
        Args:
            file_paths: Single file path or list of file paths
            validate_shapes: Whether to validate that all arrays have same shape
            
        Returns:
            Single array or list of arrays
            
        Raises:
            DataProcessingError: If loading or validation fails
        """
        try:
            if isinstance(file_paths, str):
                # Single file
                data = pd.read_csv(file_paths).to_numpy()
                return data
            else:
                # Multiple files
                data_arrays = []
                expected_shape = None
                
                for i, file_path in enumerate(file_paths):
                    data = pd.read_csv(file_path).to_numpy()
                    data_arrays.append(data)
                    
                    if validate_shapes:
                        if expected_shape is None:
                            expected_shape = data.shape
                        elif data.shape != expected_shape:
                            raise DataProcessingError(
                                f"Shape mismatch in file {i}: expected "
                                f"{expected_shape}, got {data.shape}"
                            )
                
                return data_arrays
                
        except Exception as e:
            raise DataProcessingError(f"Failed to load CSV data: {e}") from e
    
    @staticmethod
    @lru_cache(maxsize=128)
    def create_filter_coefficients(
        filter_type: str,
        cutoff_freq: float,
        sampling_freq: float,
        order: int = 4
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create filter coefficients with caching for efficiency.
        
        Args:
            filter_type: Type of filter ('lowpass', 'highpass', 'bandpass')
            cutoff_freq: Cutoff frequency or tuple for bandpass
            sampling_freq: Sampling frequency
            order: Filter order
            
        Returns:
            Tuple of (numerator, denominator) coefficients
        """
        nyquist = sampling_freq / 2
        
        if filter_type == 'lowpass':
            normalized_cutoff = cutoff_freq / nyquist
            b, a = signal.butter(order, normalized_cutoff, btype='low')
        elif filter_type == 'highpass':
            normalized_cutoff = cutoff_freq / nyquist
            b, a = signal.butter(order, normalized_cutoff, btype='high')
        elif filter_type == 'bandpass':
            if not isinstance(cutoff_freq, (list, tuple)) or len(cutoff_freq) != 2:
                raise ValueError("Bandpass filter requires two cutoff frequencies")
            normalized_cutoff = [f / nyquist for f in cutoff_freq]
            b, a = signal.butter(order, normalized_cutoff, btype='band')
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")
        
        return b, a
    
    @staticmethod
    @validate_input_data
    def apply_filter(
        data: np.ndarray,
        filter_type: str,
        cutoff_freq: float,
        sampling_freq: float,
        order: int = 4,
        axis: int = 0
    ) -> np.ndarray:
        """
        Apply digital filter to data with validation.
        
        Args:
            data: Input data array
            filter_type: Type of filter
            cutoff_freq: Cutoff frequency
            sampling_freq: Sampling frequency
            order: Filter order
            axis: Axis along which to apply filter
            
        Returns:
            Filtered data array
        """
        try:
            # Get or create filter coefficients
            b, a = DataProcessor.create_filter_coefficients(
                filter_type, cutoff_freq, sampling_freq, order
            )
            
            # Apply filter
            filtered_data = signal.filtfilt(b, a, data, axis=axis)
            
            return filtered_data
            
        except Exception as e:
            raise DataProcessingError(f"Filter application failed: {e}") from e
    
    @staticmethod
    @validate_input_data
    def compute_derivatives(
        positions: np.ndarray,
        timestamps: np.ndarray,
        method: str = 'central',
        filter_params: Optional[Dict] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute velocities and accelerations from position data.
        
        Args:
            positions: Position data (n_samples, n_joints)
            timestamps: Time stamps (n_samples,)
            method: Differentiation method ('central', 'forward', 'backward')
            filter_params: Optional filtering parameters
            
        Returns:
            Tuple of (velocities, accelerations)
        """
        if positions.shape[0] != timestamps.shape[0]:
            raise DataProcessingError(
                f"Position and timestamp shapes don't match: "
                f"{positions.shape[0]} vs {timestamps.shape[0]}"
            )
        
        n_samples, n_joints = positions.shape
        
        # Compute time differences
        dt = np.diff(timestamps)
        if np.any(dt <= 0):
            raise DataProcessingError("Timestamps must be strictly increasing")
        
        # Initialize output arrays
        velocities = np.zeros_like(positions)
        accelerations = np.zeros_like(positions)
        
        # Compute derivatives based on method
        if method == 'central':
            # Central difference (more accurate)
            for i in range(1, n_samples - 1):
                dt_prev = timestamps[i] - timestamps[i-1]
                dt_next = timestamps[i+1] - timestamps[i]
                dt_avg = (dt_prev + dt_next) / 2
                
                velocities[i, :] = (positions[i+1, :] - positions[i-1, :]) / (2 * dt_avg)
            
            # Handle boundaries with forward/backward difference
            velocities[0, :] = (positions[1, :] - positions[0, :]) / dt[0]
            velocities[-1, :] = (positions[-1, :] - positions[-2, :]) / dt[-1]
            
        elif method == 'forward':
            # Forward difference
            for i in range(n_samples - 1):
                velocities[i, :] = (positions[i+1, :] - positions[i, :]) / dt[i]
            velocities[-1, :] = velocities[-2, :]  # Extrapolate last point
            
        elif method == 'backward':
            # Backward difference
            velocities[0, :] = velocities[1, :]  # Extrapolate first point
            for i in range(1, n_samples):
                velocities[i, :] = (positions[i, :] - positions[i-1, :]) / dt[i-1]
        else:
            raise ValueError(f"Unknown differentiation method: {method}")
        
        # Compute accelerations from velocities
        dt_vel = np.diff(timestamps)
        for i in range(1, n_samples - 1):
            dt_avg = (dt_vel[i-1] + dt_vel[i]) / 2
            accelerations[i, :] = (velocities[i+1, :] - velocities[i-1, :]) / (2 * dt_avg)
        
        # Handle boundaries
        accelerations[0, :] = (velocities[1, :] - velocities[0, :]) / dt_vel[0]
        accelerations[-1, :] = (velocities[-1, :] - velocities[-2, :]) / dt_vel[-1]
        
        # Apply filtering if requested
        if filter_params:
            sampling_freq = 1.0 / np.mean(dt)
            velocities = DataProcessor.apply_filter(
                velocities, 
                sampling_freq=sampling_freq,
                **filter_params
            )
            accelerations = DataProcessor.apply_filter(
                accelerations,
                sampling_freq=sampling_freq, 
                **filter_params
            )
        
        return velocities, accelerations
    
    @staticmethod
    @validate_input_data
    def decimate_data(
        data_dict: Dict[str, np.ndarray],
        factor: int,
        method: str = 'uniform'
    ) -> Dict[str, np.ndarray]:
        """
        Decimate trajectory data with consistent indexing.
        
        Args:
            data_dict: Dictionary of data arrays
            factor: Decimation factor
            method: Decimation method ('uniform', 'adaptive')
            
        Returns:
            Dictionary of decimated data arrays
        """
        if factor <= 1:
            return data_dict
        
        decimated_data = {}
        
        for key, data in data_dict.items():
            if data is None:
                decimated_data[key] = None
                continue
                
            if method == 'uniform':
                # Simple uniform decimation
                decimated_data[key] = data[::factor]
            elif method == 'adaptive':
                # Adaptive decimation (could be implemented later)
                # For now, fallback to uniform
                decimated_data[key] = data[::factor]
            else:
                raise ValueError(f"Unknown decimation method: {method}")
        
        return decimated_data
    
    @staticmethod
    def validate_data_consistency(
        data_dict: Dict[str, np.ndarray],
        expected_keys: Optional[List[str]] = None
    ) -> None:
        """
        Validate that all data arrays have consistent shapes.
        
        Args:
            data_dict: Dictionary of data arrays
            expected_keys: Optional list of expected keys
            
        Raises:
            DataProcessingError: If data is inconsistent
        """
        if expected_keys:
            missing_keys = set(expected_keys) - set(data_dict.keys())
            if missing_keys:
                raise DataProcessingError(f"Missing required keys: {missing_keys}")
        
        # Find first non-None array to get reference shape
        reference_shape = None
        reference_key = None
        
        for key, data in data_dict.items():
            if data is not None:
                reference_shape = data.shape
                reference_key = key
                break
        
        if reference_shape is None:
            raise DataProcessingError("No valid data arrays found")
        
        # Check all other arrays
        for key, data in data_dict.items():
            if data is not None and data.shape != reference_shape:
                raise DataProcessingError(
                    f"Shape mismatch: {reference_key} has shape {reference_shape}, "
                    f"but {key} has shape {data.shape}"
                )
    
    @staticmethod
    @validate_input_data
    def remove_outliers(
        data: np.ndarray,
        method: str = 'iqr',
        threshold: float = 1.5,
        axis: int = 0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Remove outliers from data using statistical methods.
        
        Args:
            data: Input data array
            method: Outlier detection method ('iqr', 'zscore', 'modified_zscore')
            threshold: Threshold for outlier detection
            axis: Axis along which to detect outliers
            
        Returns:
            Tuple of (cleaned_data, outlier_mask)
        """
        if method == 'iqr':
            # Interquartile range method
            q1 = np.percentile(data, 25, axis=axis, keepdims=True)
            q3 = np.percentile(data, 75, axis=axis, keepdims=True)
            iqr = q3 - q1
            
            lower_bound = q1 - threshold * iqr
            upper_bound = q3 + threshold * iqr
            
            outlier_mask = (data < lower_bound) | (data > upper_bound)
            
        elif method == 'zscore':
            # Z-score method
            mean = np.mean(data, axis=axis, keepdims=True)
            std = np.std(data, axis=axis, keepdims=True)
            z_scores = np.abs((data - mean) / (std + 1e-10))
            
            outlier_mask = z_scores > threshold
            
        elif method == 'modified_zscore':
            # Modified Z-score using median
            median = np.median(data, axis=axis, keepdims=True)
            mad = np.median(np.abs(data - median), axis=axis, keepdims=True)
            modified_z_scores = 0.6745 * (data - median) / (mad + 1e-10)
            
            outlier_mask = np.abs(modified_z_scores) > threshold
            
        else:
            raise ValueError(f"Unknown outlier detection method: {method}")
        
        # Create cleaned data by replacing outliers with NaN
        cleaned_data = data.copy()
        cleaned_data[outlier_mask] = np.nan
        
        return cleaned_data, outlier_mask

    @staticmethod
    @lru_cache(maxsize=32)
    def compute_performance_metrics(
        data_signature: str,
        n_samples: int,
        n_joints: int
    ) -> Dict[str, float]:
        """
        Compute performance metrics for trajectory data with caching.
        
        This method caches performance metrics based on data characteristics
        to avoid redundant computations for similar trajectory types.
        
        Args:
            data_signature: Hash signature of the data
            n_samples: Number of samples in trajectory
            n_joints: Number of joints
            
        Returns:
            Dictionary with performance metrics
        """
        return {
            'complexity_score': n_samples * n_joints / 1000.0,
            'recommended_batch_size': min(1000, max(100, n_samples // 10)),
            'parallel_threshold': 500 * n_joints,
            'cache_effective': n_samples > 100
        }
    
    @classmethod
    def create_processing_pipeline(
        cls,
        steps: List[str],
        **kwargs
    ) -> List[callable]:
        """
        Create an optimized processing pipeline.
        
        Dynamically creates a list of processing functions based on
        specified steps, with optimizations for common patterns.
        
        Args:
            steps: List of processing step names
            **kwargs: Parameters for each step
            
        Returns:
            List of configured processing functions
        """
        pipeline = []
        
        for step in steps:
            if step == 'filter':
                cutoff = kwargs.get('cutoff', 10.0)
                fs = kwargs.get('fs', 100.0)
                pipeline.append(
                    lambda data: cls.apply_filter(
                        data, 'lowpass', cutoff_freq=cutoff, sampling_freq=fs
                    )
                )
            elif step == 'differentiate':
                dt = kwargs.get('dt', 0.01)
                pipeline.append(
                    lambda data: cls.vectorized_differentiation(data, dt)[0]
                )
            elif step == 'outliers':
                method = kwargs.get('outlier_method', 'iqr')
                threshold = kwargs.get('outlier_threshold', 1.5)
                pipeline.append(
                    lambda data: cls.remove_outliers(
                        data, method=method, threshold=threshold
                    )[0]
                )
            elif step == 'decimate':
                factor = kwargs.get('decimation_factor', 2)
                pipeline.append(
                    lambda data: cls.decimate_data(data, factor)
                )
        
        return pipeline


# Utility functions for backward compatibility
def load_trajectory_data_csv(
    position_file: str,
    torque_file: str,
    validate: bool = True
) -> Dict[str, np.ndarray]:
    """
    Load trajectory data from CSV files (convenience function).
    
    Args:
        position_file: Path to position data CSV
        torque_file: Path to torque data CSV
        validate: Whether to validate data consistency
        
    Returns:
        Dictionary with loaded data arrays
    """
    processor = DataProcessor()
    
    positions, torques = processor.load_csv_data(
        [position_file, torque_file], validate_shapes=validate
    )
    
    return {
        'positions': positions,
        'torques': torques,
        'velocities': None,
        'accelerations': None,
        'timestamps': None
    }


def apply_butterworth_filter(
    data: np.ndarray,
    cutoff_freq: float,
    sampling_freq: float,
    order: int = 4
) -> np.ndarray:
    """
    Apply Butterworth lowpass filter (convenience function).
    
    Args:
        data: Input data
        cutoff_freq: Cutoff frequency
        sampling_freq: Sampling frequency
        order: Filter order
        
    Returns:
        Filtered data
    """
    return DataProcessor.apply_filter(
        data, 'lowpass', cutoff_freq, sampling_freq, order
    )
