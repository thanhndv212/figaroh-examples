"""
Test runner script for the unified configuration system.

This script runs all tests and provides a summary of the results.
"""

import subprocess
import sys
from pathlib import Path


def run_tests():
    """Run all test suites for the unified configuration system."""
    
    print("🔧 Testing Unified Configuration Parser System")
    print("=" * 60)
    
    # Define test files
    test_files = [
        "test_unified_config_parser.py",
        "test_config_integration.py", 
        "test_backward_compatibility.py"
    ]
    
    # Results tracking
    results = {}
    total_passed = 0
    total_failed = 0
    
    for test_file in test_files:
        print(f"\n📋 Running {test_file}...")
        print("-" * 40)
        
        try:
            # Run pytest for the specific file
            result = subprocess.run([
                sys.executable, "-m", "pytest", 
                test_file, "-v", "--tb=short"
            ], capture_output=True, text=True, cwd=Path(__file__).parent)
            
            # Parse results
            output_lines = result.stdout.split('\n')
            
            # Look for test summary line
            passed = 0
            failed = 0
            for line in output_lines:
                if "passed" in line and "failed" in line:
                    # Extract numbers - format like "5 passed, 2 failed"
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == "passed" and i > 0:
                            try:
                                passed = int(parts[i-1])
                            except ValueError:
                                pass
                        elif part == "failed" and i > 0:
                            try:
                                failed = int(parts[i-1])
                            except ValueError:
                                pass
                elif "passed" in line and "failed" not in line:
                    # Format like "5 passed in 1.23s"
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == "passed" and i > 0:
                            try:
                                passed = int(parts[i-1])
                            except ValueError:
                                pass
            
            results[test_file] = {
                'passed': passed,
                'failed': failed,
                'return_code': result.returncode,
                'output': result.stdout,
                'error': result.stderr
            }
            
            total_passed += passed
            total_failed += failed
            
            # Print summary for this file
            if result.returncode == 0:
                print(f"✅ {test_file}: {passed} passed")
            else:
                print(f"❌ {test_file}: {passed} passed, {failed} failed")
                if result.stderr:
                    print(f"   Error: {result.stderr.strip()}")
            
        except Exception as e:
            print(f"❌ Error running {test_file}: {e}")
            results[test_file] = {
                'passed': 0,
                'failed': 1,
                'return_code': 1,
                'output': '',
                'error': str(e)
            }
            total_failed += 1
    
    # Final summary
    print("\n" + "=" * 60)
    print("📊 FINAL TEST SUMMARY")
    print("=" * 60)
    
    for test_file, result in results.items():
        status = "✅ PASS" if result['return_code'] == 0 else "❌ FAIL"
        print(f"{status} {test_file}: {result['passed']} passed, {result['failed']} failed")
    
    print(f"\n🎯 OVERALL: {total_passed} passed, {total_failed} failed")
    
    if total_failed == 0:
        print("🎉 All tests passed! Configuration system is ready.")
        return True
    else:
        print("⚠️  Some tests failed. Please review the issues above.")
        
        # Show detailed output for failed tests
        print("\n" + "=" * 60)
        print("🔍 DETAILED FAILURE OUTPUT")
        print("=" * 60)
        
        for test_file, result in results.items():
            if result['return_code'] != 0:
                print(f"\n--- {test_file} ---")
                if result['error']:
                    print("STDERR:")
                    print(result['error'])
                if result['output']:
                    print("STDOUT:")
                    print(result['output'])
        
        return False


def run_specific_tests():
    """Run specific test categories."""
    
    print("\n🎯 You can also run specific test categories:")
    print("   pytest -m 'not slow'        # Skip slow tests")
    print("   pytest -m 'integration'     # Only integration tests") 
    print("   pytest -m 'real_config'     # Only tests with real config files")
    print("   pytest test_unified_config_parser.py::TestUnifiedConfigParser::test_basic_config_loading")
    print("   pytest -v --tb=long         # Verbose output with full tracebacks")


def check_dependencies():
    """Check if all required dependencies are available."""
    
    print("🔍 Checking dependencies...")
    
    required_packages = ['pytest', 'yaml', 'pathlib']
    missing_packages = []
    
    for package in required_packages:
        try:
            if package == 'yaml':
                import yaml
            elif package == 'pathlib':
                from pathlib import Path
            elif package == 'pytest':
                import pytest
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print(f"❌ Missing packages: {', '.join(missing_packages)}")
        print("   Please install them with: pip install " + " ".join(missing_packages))
        return False
    else:
        print("✅ All dependencies available")
        return True


if __name__ == "__main__":
    print("🚀 Unified Configuration Parser Test Suite")
    print("=" * 60)
    
    # Check dependencies first
    if not check_dependencies():
        sys.exit(1)
    
    # Run the tests
    success = run_tests()
    
    # Provide additional information
    run_specific_tests()
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)
