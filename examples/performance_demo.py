"""
Performance optimization example for FIGAROH robot tools.

This example demonstrates how to use the enhanced DataProcessor
for improved performance in robot parameter identification and calibration.
"""

import numpy as np
import time
from typing import Dict, List
import sys
import os

# Add examples to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from shared.data_processing import DataProcessor
from shared.config_manager import ConfigManager
from shared.error_handling import DataProcessingError

class PerformanceOptimizedRobotTools:
    """
    Example class demonstrating performance optimizations for robot tools.
    
    This class shows how to leverage the new DataProcessor capabilities
    for improved performance in common robotics tasks.
    """
    
    def __init__(self, robot_type: str = 'ur10'):
        """Initialize with performance-optimized settings."""
        self.robot_type = robot_type
        self.processor = DataProcessor()
        
        # Performance tracking
        self.performance_stats = {
            'processing_times': [],
            'cache_hits': 0,
            'parallel_operations': 0
        }
    
    def benchmark_data_processing(
        self,
        data_files: List[str],
        processing_steps: List[str]
    ) -> Dict[str, float]:
        """
        Benchmark different data processing approaches.
        
        Args:
            data_files: List of trajectory data files
            processing_steps: Processing pipeline steps
            
        Returns:
            Dictionary with performance metrics
        """
        print("🚀 Running performance benchmarks...")
        
        # Create processing pipeline
        pipeline = DataProcessor.create_processing_pipeline(
            processing_steps,
            cutoff=10.0,
            fs=100.0,
            dt=0.01,
            outlier_threshold=2.0
        )
        
        # Sequential processing benchmark
        start_time = time.time()
        sequential_results = []
        for file_path in data_files:
            try:
                data = DataProcessor.load_csv_data(file_path)
                for process_func in pipeline:
                    data = process_func(data)
                sequential_results.append(data)
            except Exception as e:
                print(f"Warning: Could not process {file_path}: {e}")
        
        sequential_time = time.time() - start_time
        
        # Parallel processing benchmark
        start_time = time.time()
        try:
            parallel_results = DataProcessor.batch_process_trajectories(
                data_files,
                pipeline,
                parallel=True,
                max_workers=4
            )
        except Exception as e:
            print(f"Parallel processing failed: {e}")
            parallel_results = []
            
        parallel_time = time.time() - start_time
        
        # Cache performance test
        start_time = time.time()
        for _ in range(3):  # Repeat to test caching
            try:
                if data_files:
                    data = DataProcessor.load_csv_data(data_files[0])
                    DataProcessor.vectorized_differentiation(data)
            except Exception:
                pass
        cache_time = time.time() - start_time
        
        # Calculate speedup
        speedup = sequential_time / max(parallel_time, 0.001)
        
        results = {
            'sequential_time': sequential_time,
            'parallel_time': parallel_time,
            'cache_test_time': cache_time,
            'speedup_factor': speedup,
            'files_processed': len([r for r in parallel_results if 'error' not in r]),
            'cache_stats': DataProcessor.get_cache_stats()
        }
        
        self.performance_stats['processing_times'].append(results)
        
        return results
    
    def demonstrate_trajectory_optimization(self) -> Dict[str, any]:
        """
        Demonstrate trajectory processing optimizations.
        
        Returns:
            Dictionary with optimization results
        """
        print("📊 Demonstrating trajectory optimization...")
        
        # Generate sample trajectory data
        n_samples = 1000
        n_joints = 6
        dt = 0.01
        
        # Create realistic trajectory (sine waves with noise)
        t = np.linspace(0, 10, n_samples)
        positions = np.zeros((n_samples, n_joints))
        
        for i in range(n_joints):
            freq = 0.5 + i * 0.1  # Different frequency for each joint
            amplitude = 1.0 + i * 0.2
            positions[:, i] = amplitude * np.sin(2 * np.pi * freq * t)
            # Add noise
            positions[:, i] += 0.1 * np.random.randn(n_samples)
        
        print(f"Generated trajectory: {n_samples} samples, {n_joints} joints")
        
        # Test vectorized differentiation
        start_time = time.time()
        velocities, accelerations = DataProcessor.vectorized_differentiation(
            positions, dt, method='gradient'
        )
        vectorized_time = time.time() - start_time
        
        # Test traditional finite differences for comparison
        start_time = time.time()
        vel_fd, acc_fd = DataProcessor.vectorized_differentiation(
            positions, dt, method='finite_diff'
        )
        finite_diff_time = time.time() - start_time
        
        # Test filtering with different approaches
        data_list = [positions[:, i:i+1] for i in range(n_joints)]
        
        # Sequential filtering
        start_time = time.time()
        filtered_sequential = []
        for data_col in data_list:
            filtered = DataProcessor.apply_filter(
                data_col, 'lowpass', cutoff_freq=5.0, sampling_freq=100.0
            )
            filtered_sequential.append(filtered)
        sequential_filter_time = time.time() - start_time
        
        # Parallel filtering
        start_time = time.time()
        filtered_parallel = DataProcessor.parallel_filtering(
            data_list, cutoff=5.0, fs=100.0, max_workers=4
        )
        parallel_filter_time = time.time() - start_time
        
        # Compute trajectory metrics
        try:
            metrics = DataProcessor.compute_performance_metrics(
                'test_trajectory', n_samples, n_joints
            )
        except Exception:
            metrics = {'cache_effective': False}
        
        return {
            'data_shape': (n_samples, n_joints),
            'differentiation_methods': {
                'vectorized_gradient_time': vectorized_time,
                'finite_difference_time': finite_diff_time,
                'speedup': finite_diff_time / max(vectorized_time, 0.001)
            },
            'filtering_comparison': {
                'sequential_time': sequential_filter_time,
                'parallel_time': parallel_filter_time,
                'speedup': sequential_filter_time / max(parallel_filter_time, 0.001)
            },
            'trajectory_metrics': metrics,
            'memory_efficiency': {
                'vectorized_ops': True,
                'in_place_possible': True,
                'cache_effective': metrics.get('cache_effective', False)
            }
        }
    
    def get_performance_summary(self) -> Dict[str, any]:
        """Get comprehensive performance summary."""
        return {
            'robot_type': self.robot_type,
            'performance_stats': self.performance_stats,
            'cache_stats': DataProcessor.get_cache_stats(),
            'optimizations_enabled': [
                'vectorized_operations',
                'lru_caching',
                'parallel_processing',
                'batch_operations'
            ]
        }


def main():
    """Run performance optimization demonstration."""
    print("🎯 FIGAROH Performance Optimization Demo")
    print("=" * 50)
    
    # Initialize performance-optimized tools
    tools = PerformanceOptimizedRobotTools('ur10')
    
    # Demonstrate trajectory optimization
    traj_results = tools.demonstrate_trajectory_optimization()
    
    print("\n📈 Trajectory Processing Results:")
    print(f"Data shape: {traj_results['data_shape']}")
    print(f"Differentiation speedup: {traj_results['differentiation_methods']['speedup']:.2f}x")
    print(f"Filtering speedup: {traj_results['filtering_comparison']['speedup']:.2f}x")
    
    # Show cache statistics
    cache_stats = DataProcessor.get_cache_stats()
    print(f"\n🗄️ Cache Stats: {cache_stats['cache_size']}/{cache_stats['cache_limit']} items")
    
    # Performance summary
    summary = tools.get_performance_summary()
    print(f"\n✅ Optimizations enabled: {len(summary['optimizations_enabled'])}")
    
    print("\n🎉 Performance optimization demo completed!")
    
    return summary


if __name__ == "__main__":
    main()
