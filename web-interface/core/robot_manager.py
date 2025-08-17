"""Robot manager for handling robot loading and management."""

from typing import Dict, Any, List
import numpy as np
import os
import sys

# Add figaroh package to path for load_robot import
sys.path.append('/Users/thanhndv212/Develop/figaroh/src')

try:
    from figaroh.tools.load_robot import (
        load_robot, get_available_loaders, list_available_robots
    )
    from figaroh.tools.robot import load_robot as load_robot_alt
except ImportError as e:
    print(f"Warning: Could not import figaroh load_robot: {e}")
    load_robot = None
    get_available_loaders = None
    list_available_robots = None
    load_robot_alt = None


class RobotManager:
    """Enhanced robot manager with multiple loader support."""
    
    def __init__(self, models_path: str):
        """Initialize the robot manager."""
        self.models_path = models_path
        self.robot = None
        self.robot_info = {}
        self.available_loaders = self._check_available_loaders()
        self.loaded_robots = {}  # Cache for loaded robots
        
    def _check_available_loaders(self) -> Dict[str, Any]:
        """Check which robot loaders are available."""
        if get_available_loaders:
            return get_available_loaders()
        else:
            return {
                "figaroh": {
                    "available": False,
                    "description": "Not available"
                },
                "robot_description": {
                    "available": False,
                    "description": "Not available"
                },
                "yourdfpy": {
                    "available": False,
                    "description": "Not available"
                }
            }

    def get_available_loaders(self) -> Dict[str, Any]:
        """Get available robot loaders."""
        return self.available_loaders

    def load_robot_with_method(self, loader: str, source: str,
                               add_fext: bool = False, **kwargs) -> bool:
        """Load robot using specified method and loader."""
        try:
            if not load_robot:
                raise ImportError("Figaroh load_robot not available")
                
            # Create cache key
            cache_key = f"{loader}_{source}_{add_fext}"
            
            # Check cache first
            if cache_key in self.loaded_robots:
                self.robot = self.loaded_robots[cache_key]['robot']
                self.robot_info = self.loaded_robots[cache_key]['info']
                print(f"✅ Robot '{source}' loaded from cache "
                      f"using '{loader}' loader")
                return True
            
            # Load robot with specified method
            robot_obj = load_robot(
                robot_name=source,
                loader=loader,
                package_dirs=self.models_path,
                add_fext=add_fext,
                **kwargs
            )
            
            # Store robot and info
            self.robot = robot_obj
            self.robot_info = {
                'name': source,
                'loader': loader,
                'type': type(robot_obj).__name__,
                'add_fext': add_fext,
                'status': 'loaded'
            }
            
            # Cache the loaded robot
            self.loaded_robots[cache_key] = {
                'robot': robot_obj,
                'info': self.robot_info.copy()
            }
            
            print(f"✅ Robot '{source}' loaded using '{loader}' loader")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load robot '{source}' "
                  f"with loader '{loader}': {e}")
            self.robot = None
            self.robot_info = {'status': 'failed', 'error': str(e)}
            return False

    def load_robot_from_paths(self, urdf_path: str, model_path: str,
                              loader: str = "figaroh", **kwargs) -> bool:
        """Load robot from specific URDF and model paths."""
        try:
            if not load_robot:
                raise ImportError("Figaroh load_robot not available")
                
            if not os.path.exists(urdf_path):
                raise FileNotFoundError(f"URDF file not found: {urdf_path}")
                
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model path not found: {model_path}")
            
            # Create cache key
            robot_name = os.path.splitext(os.path.basename(urdf_path))[0]
            cache_key = f"path_{robot_name}_{loader}"
            
            # Check cache first
            if cache_key in self.loaded_robots:
                self.robot = self.loaded_robots[cache_key]['robot']
                self.robot_info = self.loaded_robots[cache_key]['info']
                print(f"✅ Robot from '{urdf_path}' loaded from cache")
                return True
            
            # Load robot from paths
            robot_obj = load_robot(
                urdf_file=urdf_path,
                loader=loader,
                package_dirs=model_path,
                **kwargs
            )
            
            # Store robot and info
            self.robot = robot_obj
            self.robot_info = {
                'name': robot_name,
                'urdf_path': urdf_path,
                'model_path': model_path,
                'loader': loader,
                'type': type(robot_obj).__name__,
                'status': 'loaded'
            }
            
            # Cache the loaded robot
            self.loaded_robots[cache_key] = {
                'robot': robot_obj,
                'info': self.robot_info.copy()
            }
            
            print(f"✅ Robot loaded from '{urdf_path}' using '{loader}' loader")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load robot from paths: {e}")
            self.robot = None
            self.robot_info = {'status': 'failed', 'error': str(e)}
            return False

    def load_robot_model(self, urdf_file: str, **kwargs) -> bool:
        """Load robot model (legacy method for backward compatibility)."""
        try:
            # Try new method first
            if load_robot:
                return self.load_robot_from_paths(
                    urdf_path=urdf_file,
                    model_path=self.models_path,
                    loader="figaroh",
                    **kwargs
                )
            
            # Fallback to old method
            if load_robot_alt:
                self.robot = load_robot_alt(
                    urdf_file,
                    package_dirs=self.models_path
                )
                self.robot_info = {
                    'name': os.path.basename(urdf_file),
                    'urdf_path': urdf_file,
                    'loader': 'figaroh_legacy',
                    'type': type(self.robot).__name__,
                    'status': 'loaded'
                }
                print(f"✅ Robot model loaded from {urdf_file}")
                return True
            else:
                raise ImportError("No robot loading method available")
                
        except Exception as e:
            print(f"❌ Error loading robot model: {e}")
            self.robot = None
            self.robot_info = {'status': 'failed', 'error': str(e)}
            return False

    def get_available_robots(self, loader: str = None) -> List[str]:
        """Get list of available robots for a specific loader."""
        try:
            if list_available_robots and loader:
                return list_available_robots(loader)
            return []
        except Exception as e:
            print(f"Error getting available robots: {e}")
            return []

    def get_robot(self):
        """Get the currently loaded robot object."""
        return self.robot

    def get_robot_info(self) -> Dict[str, Any]:
        """Get information about the currently loaded robot."""
        return self.robot_info.copy()

    def is_robot_loaded(self) -> bool:
        """Check if a robot is currently loaded."""
        return (self.robot is not None and
                self.robot_info.get('status') == 'loaded')

    def update_robot_configuration(self, configuration: np.ndarray) -> bool:
        """Update robot configuration."""
        if not self.is_robot_loaded():
            print("No robot model loaded.")
            return False
        
        try:
            # Try to validate configuration if method exists
            if hasattr(self.robot, 'validate_configuration'):
                if not self.robot.validate_configuration(configuration):
                    print("Invalid configuration.")
                    return False
            elif (hasattr(self.robot, 'model') and
                  hasattr(self.robot.model, 'nq')):
                if len(configuration) != self.robot.model.nq:
                    print(f"Configuration size mismatch: "
                          f"expected {self.robot.model.nq}, "
                          f"got {len(configuration)}")
                    return False
            
            # Update configuration
            if hasattr(self.robot, 'q0'):
                self.robot.q0 = configuration
            elif hasattr(self.robot, 'q'):
                self.robot.q = configuration
            else:
                print("Robot object doesn't support configuration updates")
                return False
                
            print("Robot configuration updated.")
            return True
            
        except Exception as e:
            print(f"Error updating robot configuration: {e}")
            return False

    def reset_robot(self) -> bool:
        """Reset robot to initial configuration."""
        if not self.is_robot_loaded():
            print("No robot model loaded.")
            return False
        
        try:
            # Get number of DOFs
            nq = 0
            if hasattr(self.robot, 'model') and hasattr(self.robot.model, 'nq'):
                nq = self.robot.model.nq
            elif hasattr(self.robot, 'model') and hasattr(self.robot.model, 'nv'):
                nq = self.robot.model.nv
            elif hasattr(self.robot, 'nq'):
                nq = self.robot.nq
            else:
                print("Cannot determine robot DOFs for reset")
                return False
            
            # Reset to zero configuration
            zero_config = np.zeros(nq)
            return self.update_robot_configuration(zero_config)
            
        except Exception as e:
            print(f"Error resetting robot: {e}")
            return False

    def get_robot_status(self) -> str:
        """Get robot status information."""
        if not self.is_robot_loaded():
            return "No robot model loaded."
        
        try:
            name = self.robot_info.get('name', 'Unknown')
            loader = self.robot_info.get('loader', 'Unknown')
            robot_type = self.robot_info.get('type', 'Unknown')
            
            # Get DOF information
            dof_info = "Unknown DOFs"
            if hasattr(self.robot, 'model'):
                if hasattr(self.robot.model, 'nq'):
                    dof_info = f"DOFs: {self.robot.model.nq}"
                elif hasattr(self.robot.model, 'nv'):
                    dof_info = f"DOFs: {self.robot.model.nv}"
            elif hasattr(self.robot, 'nq'):
                dof_info = f"DOFs: {self.robot.nq}"
            
            return f"Robot: {name}, Loader: {loader}, Type: {robot_type}, {dof_info}"
            
        except Exception as e:
            return f"Error getting robot status: {e}"

    def clear_robot(self):
        """Clear the currently loaded robot."""
        self.robot = None
        self.robot_info = {}
        print("Robot cleared")

    def clear_cache(self):
        """Clear the robot cache."""
        self.loaded_robots.clear()
        print("Robot cache cleared")

    def get_cache_info(self) -> Dict[str, Any]:
        """Get information about cached robots."""
        cache_info = {}
        for key, data in self.loaded_robots.items():
            cache_info[key] = {
                'name': data['info'].get('name', 'Unknown'),
                'loader': data['info'].get('loader', 'Unknown'),
                'type': data['info'].get('type', 'Unknown')
            }
        return cache_info

    def export_robot_info(self) -> Dict[str, Any]:
        """Export comprehensive robot information."""
        if not self.is_robot_loaded():
            return {'status': 'no_robot'}
        
        export_data = {
            'robot_info': self.robot_info.copy(),
            'robot_status': self.get_robot_status(),
            'available_loaders': self.available_loaders,
            'cache_info': self.get_cache_info()
        }
        
        # Add robot-specific data if available
        if hasattr(self.robot, 'model'):
            export_data['model_info'] = {
                'nq': getattr(self.robot.model, 'nq', None),
                'nv': getattr(self.robot.model, 'nv', None),
                'name': getattr(self.robot.model, 'name', None)
            }
        
        return export_data

    def cleanup(self):
        """Clean up robot manager resources."""
        try:
            print("🔧 Shutting down robot manager...")
            
            if self.robot is not None:
                print("🤖 Unloading robot model...")
                self.robot = None
                
            print("🗑️ Clearing robot cache...")
            self.clear_cache()
            
            self.robot_info = {}
            
            print("✅ Robot manager shutdown complete")
            
        except Exception as e:
            print(f"❌ Error during robot manager cleanup: {e}")