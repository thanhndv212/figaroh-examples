"""
Usage examples for FIGAROH modernized infrastructure.

This file demonstrates how to use the new shared infrastructure
for robot parameter identification and calibration.
"""

import numpy as np
import sys
import os

# Add examples to path
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))

from shared import (
    ConfigManager,
    DataProcessor,
    validate_robot_config,
    handle_calibration_errors,
    CalibrationError
)


def example_1_config_management():
    """Example 1: Configuration Management with Validation"""
    print("🔧 Example 1: Configuration Management")
    print("-" * 40)
    
    try:
        # Create a sample configuration
        sample_config = {
            'robot_name': 'UR10',
            'NbSample': 1000,
            'config_idx': [0, 1, 2, 3, 4, 5],
            'outlier_threshold': 2.5,
            'regularization': 0.01
        }
        
        # Validate configuration against schema
        validate_robot_config(sample_config)
        print("✅ Configuration validation passed")
        
        # Use ConfigManager for centralized management
        config_manager = ConfigManager()
        
        # Show available robot schemas
        print(f"📋 Available robot types: {list(ConfigManager.ROBOT_SCHEMAS.keys())}")
        
        # Get schema for specific robot
        ur10_schema = ConfigManager.ROBOT_SCHEMAS['ur10']
        print(f"📝 UR10 required fields: {ur10_schema.required_fields}")
        print(f"📝 UR10 optional fields: {ur10_schema.optional_fields}")
        
    except Exception as e:
        print(f"❌ Configuration error: {e}")
    
    print()


def example_2_data_processing_optimization():
    """Example 2: Optimized Data Processing"""
    print("⚡ Example 2: Data Processing Optimization")
    print("-" * 40)
    
    # Generate sample trajectory data
    n_samples = 500
    n_joints = 6
    dt = 0.01
    
    # Create realistic joint trajectories
    t = np.linspace(0, 5, n_samples)
    positions = np.zeros((n_samples, n_joints))
    
    for i in range(n_joints):
        # Different frequency and amplitude for each joint
        freq = 0.5 + i * 0.1
        amplitude = 1.0 + i * 0.3
        positions[:, i] = amplitude * np.sin(2 * np.pi * freq * t)
        # Add realistic noise
        positions[:, i] += 0.05 * np.random.randn(n_samples)
    
    print(f"📊 Generated trajectory: {n_samples} samples, {n_joints} joints")
    
    # Initialize data processor
    processor = DataProcessor()
    
    # Example 2a: Vectorized differentiation
    print("\n🔄 Computing derivatives...")
    velocities, accelerations = processor.vectorized_differentiation(positions, dt)
    print(f"✅ Computed velocities: {velocities.shape}")
    print(f"✅ Computed accelerations: {accelerations.shape}")
    
    # Example 2b: Apply filtering
    print("\n🔍 Applying filters...")
    try:
        filtered_positions = processor.apply_filter(
            positions, 'lowpass', cutoff_freq=10.0, sampling_freq=100.0
        )
        print(f"✅ Filtered data: {filtered_positions.shape}")
    except Exception as e:
        print(f"❌ Filtering error: {e}")
    
    # Example 2c: Outlier detection and removal
    print("\n🎯 Detecting outliers...")
    try:
        cleaned_data, outlier_mask = processor.remove_outliers(
            positions, method='iqr', threshold=1.5
        )
        outlier_count = np.sum(outlier_mask)
        print(f"✅ Detected {outlier_count} outliers")
        print(f"✅ Cleaned data: {cleaned_data.shape}")
    except Exception as e:
        print(f"❌ Outlier detection error: {e}")
    
    # Example 2d: Data decimation
    print("\n📉 Decimating data...")
    try:
        decimated_data = processor.decimate_data(positions, factor=2)
        print(f"✅ Decimated from {positions.shape[0]} to {decimated_data.shape[0]} samples")
    except Exception as e:
        print(f"❌ Decimation error: {e}")
    
    print()


def example_3_parallel_processing():
    """Example 3: Parallel Processing for Multiple Datasets"""
    print("🚀 Example 3: Parallel Processing")
    print("-" * 40)
    
    # Create multiple datasets to simulate multiple robot joints or files
    n_datasets = 6
    n_samples = 200
    
    print(f"📂 Creating {n_datasets} datasets with {n_samples} samples each")
    
    datasets = []
    for i in range(n_datasets):
        # Generate different signal for each dataset
        t = np.linspace(0, 2, n_samples)
        freq = 1.0 + i * 0.5
        signal_data = np.sin(2 * np.pi * freq * t).reshape(-1, 1)
        signal_data += 0.1 * np.random.randn(n_samples, 1)
        datasets.append(signal_data)
    
    # Process datasets in parallel
    print("\n⚡ Processing datasets in parallel...")
    try:
        filtered_datasets = DataProcessor.parallel_filtering(
            datasets,
            cutoff=5.0,
            fs=100.0,
            filter_type='butterworth',
            max_workers=4
        )
        print(f"✅ Processed {len(filtered_datasets)} datasets in parallel")
        
        # Show processing results
        for i, (original, filtered) in enumerate(zip(datasets, filtered_datasets)):
            noise_reduction = np.std(original) / np.std(filtered)
            print(f"   Dataset {i+1}: Noise reduction factor = {noise_reduction:.2f}")
            
    except Exception as e:
        print(f"❌ Parallel processing error: {e}")
    
    print()


def example_4_processing_pipeline():
    """Example 4: Creating and Using Processing Pipelines"""
    print("🔄 Example 4: Processing Pipeline")
    print("-" * 40)
    
    # Generate sample data
    n_samples = 300
    t = np.linspace(0, 3, n_samples)
    
    # Create signal with noise and outliers
    clean_signal = np.sin(2 * np.pi * 1.0 * t) + 0.5 * np.sin(2 * np.pi * 3.0 * t)
    noisy_signal = clean_signal + 0.2 * np.random.randn(n_samples)
    
    # Add some outliers
    outlier_indices = np.random.choice(n_samples, size=10, replace=False)
    noisy_signal[outlier_indices] += np.random.randn(10) * 2.0
    
    data = noisy_signal.reshape(-1, 1)
    
    print(f"📊 Created signal with {n_samples} samples and artificial outliers")
    
    # Create processing pipeline
    print("\n🔧 Creating processing pipeline...")
    try:
        pipeline = DataProcessor.create_processing_pipeline(
            steps=['filter', 'outliers', 'differentiate'],
            cutoff=10.0,
            fs=100.0,
            outlier_method='iqr',
            outlier_threshold=2.0,
            dt=0.01
        )
        print(f"✅ Created pipeline with {len(pipeline)} steps")
        
        # Apply pipeline
        print("\n⚙️ Applying processing pipeline...")
        processed_data = data.copy()
        
        for i, process_func in enumerate(pipeline):
            processed_data = process_func(processed_data)
            print(f"   Step {i+1}: Output shape = {processed_data.shape}")
        
        # Compare original vs processed
        original_std = np.std(data)
        processed_std = np.std(processed_data)
        improvement = original_std / processed_std
        
        print(f"✅ Processing complete!")
        print(f"   Original noise level: {original_std:.4f}")
        print(f"   Processed noise level: {processed_std:.4f}")
        print(f"   Improvement factor: {improvement:.2f}x")
        
    except Exception as e:
        print(f"❌ Pipeline error: {e}")
    
    print()


@handle_calibration_errors
def example_5_error_handling():
    """Example 5: Robust Error Handling"""
    print("🛡️ Example 5: Error Handling")
    print("-" * 40)
    
    # Demonstrate automatic error handling with decorators
    print("🔍 Testing error handling capabilities...")
    
    # Test 1: Configuration validation
    print("\n📋 Test 1: Configuration validation")
    try:
        invalid_config = {
            'robot_name': 'TestRobot',
            'invalid_field': 'should_fail'
            # Missing required fields
        }
        validate_robot_config(invalid_config)
        print("❌ Validation should have failed!")
    except Exception as e:
        print(f"✅ Caught validation error: {type(e).__name__}")
    
    # Test 2: Data processing with invalid data
    print("\n📊 Test 2: Data processing error handling")
    try:
        processor = DataProcessor()
        invalid_data = "not_an_array"  # This should fail
        processor.apply_filter(invalid_data, 'lowpass', 5.0, 100.0)
        print("❌ Should have failed with invalid data!")
    except Exception as e:
        print(f"✅ Caught data processing error: {type(e).__name__}")
    
    # Test 3: Graceful error recovery
    print("\n🔄 Test 3: Graceful error recovery")
    try:
        # This function has the @handle_calibration_errors decorator
        # so it will automatically handle CalibrationError exceptions
        raise CalibrationError("Simulated calibration failure")
    except CalibrationError as e:
        print(f"✅ Gracefully handled: {e}")
    
    print("✅ Error handling tests completed successfully!")
    print()


def example_6_cache_performance():
    """Example 6: Cache Performance and Statistics"""
    print("💾 Example 6: Cache Performance")
    print("-" * 40)
    
    processor = DataProcessor()
    
    # Show initial cache stats
    initial_stats = processor.get_cache_stats()
    print(f"📊 Initial cache: {initial_stats['cache_size']}/{initial_stats['cache_limit']} items")
    
    # Perform some cached operations
    print("\n🔄 Performing cached operations...")
    
    try:
        # Generate test data
        data = np.random.randn(100, 3)
        
        # These operations will be cached
        for i in range(3):
            print(f"   Iteration {i+1}...")
            # Filter design is cached
            filtered = processor.apply_filter(data, 'lowpass', 5.0, 100.0)
            
            # Differentiation computation  
            vel, acc = processor.vectorized_differentiation(data, 0.01)
        
        # Show final cache stats
        final_stats = processor.get_cache_stats()
        print(f"\n📊 Final cache: {final_stats['cache_size']}/{final_stats['cache_limit']} items")
        print(f"📈 Cache utilization: {final_stats['cache_size'] / final_stats['cache_limit'] * 100:.1f}%")
        
        # Clear cache demonstration
        print("\n🧹 Clearing cache...")
        processor.clear_cache()
        cleared_stats = processor.get_cache_stats()
        print(f"📊 After clearing: {cleared_stats['cache_size']}/{cleared_stats['cache_limit']} items")
        
    except Exception as e:
        print(f"❌ Cache test error: {e}")
    
    print()


def main():
    """Run all usage examples."""
    print("🎯 FIGAROH Infrastructure Usage Examples")
    print("=" * 50)
    
    # Run all examples
    example_1_config_management()
    example_2_data_processing_optimization()
    example_3_parallel_processing()
    example_4_processing_pipeline()
    example_5_error_handling()
    example_6_cache_performance()
    
    print("🎉 All examples completed successfully!")
    print("\nNext steps:")
    print("- Check out specific robot implementations in ur10/, tiago/, etc.")
    print("- Run performance_demo.py for detailed benchmarks")
    print("- Explore the test suite in tests/ for more usage patterns")
    print("- Read the full documentation in README.md")


if __name__ == "__main__":
    main()
