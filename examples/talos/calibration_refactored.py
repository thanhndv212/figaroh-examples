#!/usr/bin/env python3

# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
TALOS torso-arm calibration using the FIGAROH framework.

This script demonstrates calibration of the TALOS humanoid robot's
torso-arm kinematic chain using experimental data.
"""

import os
import sys
import argparse

from figaroh.tools.robot import load_robot

# Add utils directory to path
from utils.talos_tools import TALOSCalibration


def main(data_type="experimental", visualization=False, verbose=True):
    """
    Main function for TALOS torso-arm calibration.
    
    Args:
        data_type (str): Type of data to use ('experimental' or 'sample')
        visualization (bool): Whether to show visualization plots
        verbose (bool): Whether to enable verbose output
    """
    print("=" * 60)
    print("TALOS Torso-Arm Calibration with FIGAROH")
    print("=" * 60)
        
    # Load robot
    robot = load_robot(
        "urdf/talos_full_v2.urdf",
        package_dirs="models",
        load_by_urdf=True,
    )
    
    print("Robot model loaded successfully:")
    print(f"  - DOF: {robot.model.nq}")
    print(f"  - Joints: {robot.model.njoints}")
    
    # Initialize calibration
    print("\nInitializing TALOS calibration...")
    calibration = TALOSCalibration(
        robot=robot,
        config_file="config/talos_config.yaml"
    )
        # Set required parameters that aren't in config file
    calibration.calib_config["known_baseframe"] = False
    calibration.calib_config["known_tipframe"] = False
    calibration.initialize()

    # Load or generate data
    print(f"\nLoading {data_type} data...")
    
    if data_type == "experimental":
        # Load experimental data
        PEE_measured, q_measured = calibration.load_experimental_data()
        n_samples = calibration.calib_config['NbSample']
        print(f"  - Loaded {n_samples} experimental samples")
        
    elif data_type == "sample":
        # Generate synthetic data
        PEE_measured, q_measured = calibration.generate_sample_data(seed=0.05)
        n_samples = calibration.calib_config['NbSample']
        print(f"  - Generated {n_samples} synthetic samples")
        
    else:
        raise ValueError("data_type must be 'experimental' or 'sample'")
    
    # Display calibration parameters
    param_count = len(calibration.calib_config['param_name'])
    print(f"\nCalibration parameters ({param_count} total):")
    for i, param_name in enumerate(calibration.calib_config['param_name']):
        print(f"  {i:2d}: {param_name}")
    
    # Run calibration
    print("\n" + "=" * 40)
    print("Running calibration optimization...")
    print("=" * 40)
    
    result = calibration.solve(
        method="lm",
        max_iterations=3,
        outlier_threshold=3.0,
        enable_logging=verbose
    )
    
    # Analyze results
    print("\n" + "=" * 40)
    print("Calibration Results")
    print("=" * 40)
    
    analysis = calibration.analyze_results(result)
    
    status = 'SUCCESS' if result.success else 'FAILED'
    print(f"\nOptimization status: {status}")
    print(f"Final cost: {result.cost:.8f}")
    print(f"Optimality: {result.optimality:.2e}")
    print(f"Number of function evaluations: {result.nfev}")
    
    # Calculate and display standard deviations
    print("\nParameter uncertainty analysis:")
    std_dev, std_percentage = calibration.calculate_standard_deviations(result)
    
    print("\nParameter values with uncertainties:")
    for i, (param_name, value, std, std_pct) in enumerate(zip(
        calibration.calib_config['param_name'],
        result.x,
        std_dev,
        std_percentage
    )):
        if std_pct != float('inf'):
            line = f"  {i:2d}: {param_name:25s} = {value:8.6f} ± {std:8.6f}"
            line += f" ({std_pct:5.1f}%)"
            print(line)
        else:
            print(f"  {i:2d}: {param_name:25s} = {value:8.6f} ± {std:8.6f}")
    
    # Outlier analysis
    print("\nOutlier analysis:")
    outliers = calibration.detect_outliers(threshold=0.02)
    if outliers:
        print(f"Found {len(outliers)} outlier measurements:")
        for marker_idx, sample_idx in outliers[:10]:  # Show first 10
            print(f"  - Marker {marker_idx + 1}, Sample {sample_idx + 1}")
        if len(outliers) > 10:
            print(f"  ... and {len(outliers) - 10} more")
    else:
        print("No significant outliers detected.")
    
    # Display summary
    print("\n" + "=" * 60)
    print("CALIBRATION SUMMARY")
    print("=" * 60)
    print(f"Data type:            {data_type}")
    print(f"Number of samples:    {calibration.calib_config['NbSample']}")
    print(f"Number of markers:    {calibration.calib_config['NbMarkers']}")
    print(f"Parameters estimated: {len(result.x)}")
    print(f"RMSE before calib:    {analysis['rmse_uncalibrated']:.6f} m")
    print(f"RMSE after calib:     {analysis['rmse_calibrated']:.6f} m")
    print(f"Improvement factor:   {analysis['improvement_ratio']:.2f}x")
    print(f"Success:              {result.success}")
    
    # Visualization
    if visualization:
        print("\nGenerating visualization plots...")
        calibration.plot_calibration_results(
            show_outliers=True,
            outlier_threshold=0.02
        )
    
    # Save results
    print("\nSaving results...")
    
    # Save solution to file
    output_file = "results/talos_calibration_solution.txt"
    os.makedirs("results", exist_ok=True)
    
    with open(output_file, 'w') as f:
        f.write("TALOS Torso-Arm Calibration Results\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Data type: {data_type}\n")
        rmse_before = analysis['rmse_uncalibrated']
        rmse_after = analysis['rmse_calibrated']
        improvement = analysis['improvement_ratio']
        f.write(f"RMSE before calibration: {rmse_before:.6f} m\n")
        f.write(f"RMSE after calibration:  {rmse_after:.6f} m\n")
        f.write(f"Improvement factor:      {improvement:.2f}x\n")
        f.write(f"Optimization success:    {result.success}\n")
        f.write(f"Final cost:              {result.cost:.8f}\n")
        f.write(f"Optimality:              {result.optimality:.2e}\n\n")
        
        f.write("Calibrated Parameters:\n")
        f.write("-" * 30 + "\n")
        for i, (param_name, value, std, std_pct) in enumerate(zip(
            calibration.calib_config['param_name'],
            result.x,
            std_dev,
            std_percentage
        )):
            f.write(f"{i:2d}: {param_name:25s} = {value:12.8f} ± {std:10.8f}")
            if std_pct != float('inf'):
                f.write(f" ({std_pct:5.1f}%)")
            f.write("\n")
    
    print(f"Results saved to: {output_file}")
    
    return result, analysis


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="TALOS torso-arm calibration using FIGAROH framework"
    )
    parser.add_argument(
        "--data-type",
        choices=["experimental", "sample"],
        default="experimental",
        help="Type of data to use for calibration"
    )
    parser.add_argument(
        "--visualization",
        action="store_true",
        help="Show visualization plots"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce output verbosity"
    )
    
    args = parser.parse_args()
    
    try:
        result, analysis = main(
            data_type=args.data_type,
            visualization=args.visualization,
            verbose=not args.quiet
        )
        
        print("\nCalibration completed successfully!")
        
    except Exception as e:
        print(f"\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
