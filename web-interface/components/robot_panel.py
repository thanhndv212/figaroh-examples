"""Robot panel component for Viser interface."""

import viser
from typing import Dict, Any
import traceback
import os
import sys

# Add figaroh package to path for load_robot import
sys.path.append('/Users/thanhndv212/Develop/figaroh/src')

try:
    from figaroh.tools.load_robot import (
        load_robot, get_available_loaders, list_available_robots
    )
except ImportError as e:
    print(f"Warning: Could not import load_robot: {e}")
    load_robot = None
    get_available_loaders = None
    list_available_robots = None


class RobotPanel:
    """Robot control and display panel using Viser."""
    
    # ============================================================================
    # INITIALIZATION AND SETUP
    # ============================================================================
    
    def __init__(self, server: viser.ViserServer, example_loader=None, interface=None):
        self.server = server
        self.example_loader = example_loader
        self.interface = interface  # Reference to main interface for accessing selected paths
        self.robot_models = {}
        self.current_robot = None
        self.current_robot_object = None
        self.available_loaders = self._check_available_loaders()
        self.setup_ui()
    
    def _check_available_loaders(self) -> Dict[str, Any]:
        """Check which robot loaders are available."""
        if get_available_loaders:
            return get_available_loaders()
        else:
            return {
                "figaroh": {"available": False, "description": "Not available"},
                "robot_description": {"available": False, "description": "Not available"},
                "yourdfpy": {"available": False, "description": "Not available"}
            }
    
    def setup_ui(self):
        """Setup the robot panel UI components."""
        try:
            # Robot loader selection
            available_loader_names = [name for name, info in self.available_loaders.items() 
                                    if info.get("available", False)]
            if not available_loader_names:
                available_loader_names = ["figaroh"]  # Fallback
            
            self.loader_dropdown = self.server.gui.add_dropdown(
                "🔧 Robot Loader",
                options=available_loader_names,
                initial_value=available_loader_names[0],
            )
            
            # Robot source selection (depends on loader)
            self.source_dropdown = self.server.gui.add_dropdown(
                "📁 Robot Source",
                options=["None"],
                initial_value="None",
            )
            
            # Additional parameters
            self.add_fext_checkbox = self.server.gui.add_checkbox(
                "🚀 Add Free-flyer Joint",
                initial_value=False,
            )
            
            # Loading mode dropdown (Auto/Manual)
            self.loading_mode_dropdown = self.server.gui.add_dropdown(
                "📁 Loading Mode",
                options=["Auto", "Manual"],
                initial_value="Auto",
            )
            
            # Robot info display
            self.robot_info = self.server.gui.add_text(
                "Robot Info",
                initial_value="No robot loaded",
                disabled=True
            )
            
            # Loader info display
            self.loader_info = self.server.gui.add_text(
                "Loader Info",
                initial_value=self._get_loader_info(self.loader_dropdown.value),
                disabled=True,
                visible=False,
            )
            
            # Setup callbacks
            self.loading_mode_dropdown.on_update(self._on_loading_mode_changed)
            self.loader_dropdown.on_update(self._on_loader_changed)
            self.source_dropdown.on_update(self._on_source_selected)
            
            # Initialize with first loader
            self._on_loader_changed(None)
            
        except Exception as e:
            print(f"Error setting up robot panel UI: {e}")
            traceback.print_exc()
    
    # ============================================================================
    # UI EVENT HANDLERS AND CALLBACKS
    # ============================================================================
    
    def _get_loader_info(self, loader_name: str) -> str:
        """Get information about the selected loader."""
        if loader_name in self.available_loaders:
            info = self.available_loaders[loader_name]
            status = "✅ Available" if info.get("available", False) else "❌ Not available"
            description = info.get("description", "No description")
            returns = info.get("returns", "Unknown")
            features = ", ".join(info.get("features", []))
            return f"{status}\n{description}\nReturns: {returns}\nFeatures: {features}"
        return f"Unknown loader: {loader_name}"
    
    def _on_loader_changed(self, _):
        """Handle loader type change."""
        try:
            selected_loader = self.loader_dropdown.value
            print(f"Loader changed to: {selected_loader}")
            
            # Update loader info
            self.loader_info.value = self._get_loader_info(selected_loader)
            
            # Update available sources based on loader
            self._update_sources_for_loader(selected_loader)
            
        except Exception as e:
            print(f"Error on loader change: {e}")
            traceback.print_exc()
    
    def _update_sources_for_loader(self, loader_name: str):
        """Update available robot sources based on selected loader."""
        try:
            if loader_name == "robot_description":
                # Get available robot descriptions
                if list_available_robots:
                    robots = list_available_robots("robot_description")
                    if robots:
                        self.source_dropdown.options = robots
                        print(f"Found {len(robots)} robot descriptions")
                    else:
                        self.source_dropdown.options = ["tiago", "ur10", "panda"]  # Fallback
                else:
                    self.source_dropdown.options = ["tiago", "ur10", "panda"]  # Fallback
            
            elif loader_name in ["figaroh", "yourdfpy"]:
                # Use examples from example_loader
                if self.example_loader:
                    examples = list(self.example_loader.list_examples().keys())
                    if examples:
                        self.source_dropdown.options = examples
                    else:
                        self.source_dropdown.options = ["None"]
                else:
                    self.source_dropdown.options = ["None"]
            
            else:
                self.source_dropdown.options = ["None"]
                
        except Exception as e:
            print(f"Error updating sources for loader {loader_name}: {e}")
            traceback.print_exc()
    
    def _on_source_selected(self, _):
        """Handle robot source selection."""
        try:
            selected_source = self.source_dropdown.value
            if selected_source != "None":
                self.robot_info.value = f"Selected: {selected_source} \n Click 'Load Robot' to load"
                print(f"Robot source selected: {selected_source}")
            else:
                self.robot_info.value = "No robot selected"
            
            # Notify interface to update URDF paths based on new source selection
            if (self.interface and 
                hasattr(self.interface, 'update_paths_for_source')):
                self.interface.update_paths_for_source()
            elif (self.interface and 
                  hasattr(self.interface, 'update_urdf_paths_for_source')):
                # Fallback to old method for backwards compatibility
                self.interface.update_urdf_paths_for_source()
                
        except Exception as e:
            print(f"Error on source selection: {e}")
            traceback.print_exc()
    
    def _on_load_robot_combined(self, _):
        """Handle robot loading - either from selected method or from local paths."""
        self.load_robot_combined()
    
    def _on_loading_mode_changed(self, _):
        """Handle toggle of loading mode option."""
        try:
            loading_mode = self.loading_mode_dropdown.value
            if loading_mode == "Manual":
                print("Manual loading mode - Path Selection panel will be used")
                visible = True
            else:
                print("Auto loading mode - Using loader methods")
                visible = False
            # Notify interface to show/hide path selection panel
            if self.interface and hasattr(self.interface, '_update_path_selection_visibility'):
                self.interface._update_path_selection_visibility(visible)
                
        except Exception as e:
            print(f"Error changing loading mode: {e}")
    
    # ============================================================================
    # ROBOT LOADING METHODS
    # ============================================================================
    
    
    def load_robot_combined(self):
        """Public method to handle robot loading - either from selected method or from local paths."""
        
        try:
            if self.loading_mode_dropdown.value == "Manual":
                # Load from selected local paths
                print(f"***************Loading robot with mode: {self.loading_mode_dropdown.value}")
                self._load_from_selected_paths()
            elif self.loading_mode_dropdown.value == "Auto":
                # Load using selected loader method
                selected_loader = self.loader_dropdown.value
                selected_source = self.source_dropdown.value
                add_fext = self.add_fext_checkbox.value
                
                if selected_source != "None" and load_robot:
                    self.load_robot_with_method(selected_loader, selected_source, add_fext)
                else:
                    print("No robot source selected or load_robot not available")
                    
        except Exception as e:
            print(f"Error loading robot: {e}")
            traceback.print_exc()
    
    def _load_from_selected_paths(self):
        """Load robot from selected URDF and model paths."""
        try:
            if not self.interface:
                print("No interface reference available for path selection")
                return
            
            # Get selected paths from interface
            selected_paths = self.interface.get_selected_paths()
            urdf_path = selected_paths.get('urdf_path')
            model_path = selected_paths.get('model_path')
            
            if not urdf_path:
                self.robot_info.value = "❌ No URDF path selected"
                print("No URDF path selected")
                return
            
            if not model_path:
                print("Warning: No model path selected - using URDF directory")
                model_path = os.path.dirname(urdf_path)
            
            # Load robot from selected paths
            self.load_robot_from_paths(urdf_path, model_path)
            
        except Exception as e:
            print(f"Error loading robot from selected paths: {e}")
            traceback.print_exc()
    
    def load_robot_with_method(self, loader: str, source: str, add_fext: bool = False):
        """Load robot using specified method."""
        try:
            print(f"Loading robot '{source}' with loader '{loader}' (fext: {add_fext})")
            
            if loader == "robot_description":
                robot_obj = load_robot(
                    robot_urdf=source,
                    loader="robot_description",
                    isFext=add_fext
                )
                
            elif loader in ["figaroh", "yourdfpy"]:
                if not self.example_loader:
                    raise ValueError("Example loader not available")
                
                examples = self.example_loader.list_examples()
                if source not in examples:
                    raise ValueError(f"Example '{source}' not found")
                
                # Get paths using unified helper
                paths = self._get_robot_paths(examples[source])
                if not paths['urdf_path']:
                    raise FileNotFoundError(f"No URDF file found for {source}")
                
                # Adjust package directories for loader
                package_dirs = self._adjust_package_dirs_for_loader(
                    paths['package_dirs'], loader
                )
                
                robot_obj = load_robot(
                    robot_urdf=paths['urdf_path'],
                    package_dirs=package_dirs,
                    loader=loader,
                    isFext=add_fext
                )
            
            else:
                raise ValueError(f"Unsupported loader: {loader}")
            
            # Store and display robot
            self._store_and_display_robot(robot_obj, source, loader)
            
        except Exception as e:
            self._handle_robot_loading_error(e, source)
    
    def _store_and_display_robot(self, robot_obj, source: str, loader: str):
        """Store loaded robot and update UI display."""
        self.current_robot = source
        self.current_robot_object = robot_obj
        
        robot_type = type(robot_obj).__name__
        self.robot_info.value = f"✅ Loaded: {source}\nLoader: {loader}\nType: {robot_type}"
        
        print(f"Successfully loaded robot '{source}' using '{loader}' loader")
        print(f"Robot object type: {robot_type}")
        
        # Visualize robot
        self._visualize_loaded_robot(robot_obj, loader)
    
    def _handle_robot_loading_error(self, error: Exception, source: str):
        """Handle robot loading errors consistently."""
        error_msg = f"❌ Failed to load {source}: {str(error)}"
        self.robot_info.value = error_msg
        print(f"Error loading robot: {error}")
        traceback.print_exc()
    
    def load_robot_from_paths(self, urdf_path: str, model_path: str):
        """Load robot directly from URDF and model paths."""
        try:
            add_fext = self.add_fext_checkbox.value
            selected_loader = self.loader_dropdown.value
            
            print(f"Loading robot from URDF: {urdf_path}")
            print(f"Using model path: {model_path}")
            print(f"Using loader: {selected_loader}")
            print(f"Add free-flyer: {add_fext}")
            
            # Validate inputs
            self._validate_path_loading_inputs(urdf_path, model_path, selected_loader)
            
            # Adjust package directories and load robot for figaroh
            adjusted_model_path = self._adjust_package_dirs_for_loader(model_path, selected_loader)
            robot_obj = load_robot(
                robot_urdf=urdf_path,
                package_dirs=adjusted_model_path,
                loader=selected_loader,
                isFext=add_fext
            )
            
            # For figaroh, create separate visualization object
            # if selected_loader == "figaroh":
            #     vis_obj = load_robot(
            #         robot_urdf=urdf_path,
            #         package_dirs=model_path,
            #         loader="yourdfpy",
            #         isFext=add_fext
            #     )
            #     self._visualize_loaded_robot(vis_obj, "yourdfpy", add_fext)
            # else:
            self._visualize_loaded_robot(robot_obj, selected_loader, add_fext)
            
            # Store and update UI
            robot_name = os.path.basename(urdf_path).replace('.urdf', '')
            self._store_path_loaded_robot(robot_obj, robot_name, urdf_path, selected_loader)
            
        except Exception as e:
            self._handle_robot_loading_error(e, "robot from paths")
    
    def _validate_path_loading_inputs(self, urdf_path: str, model_path: str, loader: str):
        """Validate inputs for path-based robot loading."""
        if not load_robot:
            raise ImportError("load_robot function not available")
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model path not found: {model_path}")
        
        if loader not in ["figaroh", "yourdfpy"]:
            raise ValueError(
                f"Loader '{loader}' is not supported for loading from paths. "
                f"Please use 'figaroh' or 'yourdfpy' loader instead."
            )
    
    def _store_path_loaded_robot(self, robot_obj, robot_name: str, urdf_path: str, loader: str):
        """Store robot loaded from paths and update UI."""
        self.current_robot = robot_name
        self.current_robot_object = robot_obj
        
        robot_type = type(robot_obj).__name__
        self.robot_info.value = (
            f"✅ Loaded: {robot_name}\n"
            f"URDF: {os.path.basename(urdf_path)}\n"
            f"Loader: {loader}\n"
            f"Type: {robot_type}"
        )
        
        print(f"Successfully loaded robot '{robot_name}' using '{loader}' loader")
    
    # ============================================================================
    # HELPER METHODS FOR ROBOT LOADING
    # ============================================================================
    
    def _get_robot_paths(self, example_path: str = None) -> dict:
        """Get URDF and package paths with intelligent source-based discovery."""
        try:
            selected_source = getattr(self.source_dropdown, 'value', None)
            if selected_source == "None":
                selected_source = None
            
            # If example_path provided, find URDF there first
            if example_path:
                urdf_path = self._find_urdf_in_example(example_path)
                if urdf_path:
                    return {
                        'urdf_path': urdf_path,
                        'package_dirs': self._get_intelligent_package_dirs(selected_source)
                    }
            
            # Otherwise, search in models directory based on source
            if selected_source:
                project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                models_path = os.path.join(project_root, "models")
                
                if os.path.exists(models_path):
                    for model_name in os.listdir(models_path):
                        model_dir = os.path.join(models_path, model_name)
                        if os.path.isdir(model_dir) and self._is_source_match(selected_source, model_name):
                            urdf_dir = os.path.join(model_dir, "urdf")
                            if os.path.exists(urdf_dir):
                                for file in os.listdir(urdf_dir):
                                    if file.endswith(".urdf"):
                                        return {
                                            'urdf_path': os.path.join(urdf_dir, file),
                                            'package_dirs': models_path
                                        }
            
            return {'urdf_path': None, 'package_dirs': None}
            
        except Exception as e:
            print(f"Error getting robot paths: {e}")
            return {'urdf_path': None, 'package_dirs': None}
    
    def _is_source_match(self, source: str, model_name: str) -> bool:
        """Check if source matches model name with flexible matching."""
        if not source:
            return False
        
        source_clean = source.replace('_description', '')
        model_clean = model_name.replace('_description', '')
        
        return any([
            source in model_name,
            model_name in source,
            source_clean in model_name,
            model_clean in source
        ])
    
    def _get_intelligent_package_dirs(self, selected_source: str = None) -> str:
        """Get package directories with intelligent source-based selection."""
        if not selected_source:
            return None
            
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        models_path = os.path.join(project_root, "models")
        
        if os.path.exists(models_path):
            # Look for matching model directory
            for model_name in os.listdir(models_path):
                model_dir = os.path.join(models_path, model_name)
                if os.path.isdir(model_dir) and self._is_source_match(selected_source, model_name):
                    return model_dir
            # Return models directory if no specific match
            return models_path
            
        return None
    
    def _adjust_package_dirs_for_loader(self, package_dirs: str, loader: str) -> str:
        """Adjust package directories based on loader requirements."""
        if not package_dirs:
            return package_dirs
            
        if loader == "figaroh" and not package_dirs.endswith('models'):
            adjusted = os.path.dirname(package_dirs)
            print(f"Adjusted package dirs for {loader}: {adjusted}")
            return adjusted
            
        return package_dirs
    
    def _find_urdf_in_example(self, example_path: str) -> str:
        """Find URDF file in example directory."""
        try:
            # Common URDF locations
            for urdf_dir in ["urdf", "robots", "description"]:
                urdf_path = os.path.join(example_path, urdf_dir)
                if os.path.exists(urdf_path):
                    for file in os.listdir(urdf_path):
                        if file.endswith(".urdf"):
                            return os.path.join(urdf_path, file)
            
            # Look in root directory
            for file in os.listdir(example_path):
                if file.endswith(".urdf"):
                    return os.path.join(example_path, file)
            
            return None
            
        except Exception as e:
            print(f"Error finding URDF in {example_path}: {e}")
            return None
        
    
    # ============================================================================
    # ROBOT VISUALIZATION METHODS
    # ============================================================================
    
    
    def _visualize_loaded_robot(self, robot_obj, loader: str, add_fext: bool=False):
        """Visualize the loaded robot based on the loader type."""
        try:
            if loader == "yourdfpy":
                self._visualize_yourdfpy_robot(robot_obj, add_fext)
            elif loader == "figaroh":
                # For pinocchio-based robots, we could implement a different visualization
                self._visualize_figaroh_robot(robot_obj, add_fext)
            elif loader == "robot_description":
                # For robot_description robots (RobotWrapper), visualize accordingly
                self._visualize_robot_description_robot(robot_obj, add_fext)
            else:
                print(f"Visualization not implemented for loader: {loader}")
                
        except Exception as e:
            print(f"Error visualizing robot with loader {loader}: {e}")
    
    def _visualize_figaroh_robot(self, robot_obj, add_fext: bool = False):
        """Visualize robot loaded with figaroh using yourdfpy for visualization."""
        try:
            print(f"Visualizing figaroh robot: {type(robot_obj)}")
            
            if not hasattr(self, 'current_robot') or not self.current_robot:
                print("No current robot context for visualization")
                return
            
            # Get selected source
            selected_source = getattr(self.source_dropdown, 'value', None)
            if not selected_source or selected_source == "None":
                print("No selected source for figaroh visualization")
                return
                
            # Get robot paths for visualization
            if self.example_loader:
                examples = self.example_loader.list_examples()
                if selected_source in examples:
                    paths = self._get_robot_paths(examples[selected_source])
                    
                    if paths['urdf_path'] and load_robot:
                        try:
                            # Create yourdfpy visualization object
                            _urdf_vis = load_robot(
                                robot_urdf=paths['urdf_path'],
                                package_dirs=paths['package_dirs'],
                                loader="yourdfpy",
                                isFext=add_fext
                            )
                            self._visualize_yourdfpy_robot(_urdf_vis, add_fext)
                            print("✓ Figaroh robot visualized using yourdfpy renderer")
                        except Exception as vis_error:
                            print(f"Error creating yourdfpy visualization: {vis_error}")
                            print("Figaroh robot loaded but visualization unavailable")
                    else:
                        print("Could not find URDF file or load_robot unavailable")
                else:
                    print(f"Selected source '{selected_source}' not found in examples")
            else:
                print("Example loader not available for figaroh visualization")
            
        except Exception as e:
            print(f"Error visualizing figaroh robot: {e}")
            traceback.print_exc()
    
    def _visualize_yourdfpy_robot(self, robot_obj, add_fext: bool=False):
        """Visualize robot loaded with yourdfpy using ViserUrdf."""
        try:
            import numpy as np
            
            print(f"Visualizing robot with yourdfpy object: {type(robot_obj)}")
            
            # Clear any existing robot visualization
            self._clear_robot_visualization()
            
            # Try to use ViserUrdf for better visualization
            try:
                from viser.extras import ViserUrdf
                
                # Remove existing robot visualization
                if hasattr(self, 'urdf_vis') and self.urdf_vis is not None:
                    try:
                        self.server.scene.remove("/robot")
                    except Exception:
                        pass
                    self.urdf_vis = None
                
                # Create new ViserUrdf visualization
                # ViserUrdf can handle both URDF strings and yourdfpy.URDF objects
                self.urdf_vis = ViserUrdf(
                    self.server,
                    urdf_or_path=robot_obj,
                    root_node_name="/robot",
                    load_meshes=True,
                    load_collision_meshes=False,
                )
                
                # Add GUI controls for joint manipulation (if available)
                with self.server.gui.add_folder(f"🤖 Joint Control for {self.current_robot}"):
                    try:
                        joint_limits = self.urdf_vis.get_actuated_joint_limits()
                        sliders = []
                        initial_config = []
                        
                        for joint_name, (lower, upper) in joint_limits.items():
                            lower = lower if lower is not None else -np.pi
                            upper = upper if upper is not None else np.pi
                            initial_pos = (0.0 if lower < -0.1 and upper > 0.1
                                           else (lower + upper) / 2.0)
                            
                            slider = self.server.gui.add_slider(
                                label=joint_name,
                                min=lower,
                                max=upper,
                                step=1e-3,
                                initial_value=initial_pos,
                            )
                            sliders.append(slider)
                            initial_config.append(initial_pos)
                        
                        # Update function for sliders
                        def update_robot():
                            config = np.array([s.value for s in sliders])
                            self.urdf_vis.update_cfg(config)
                        
                        for slider in sliders:
                            slider.on_update(lambda _: update_robot())
                        
                        # Set initial configuration
                        if initial_config:
                            self.urdf_vis.update_cfg(np.array(initial_config))
                        
                        # Add reset button
                        reset_button = self.server.gui.add_button("🔄 Reset Joints")
                        
                        def reset_joints(_):
                            for slider, init_val in zip(sliders, initial_config):
                                slider.value = init_val
                            update_robot()
                                
                        reset_button.on_click(reset_joints)
                        
                        print(f"✓ Added {len(sliders)} joint control sliders")
                        
                    except Exception as e:
                        print(f"Note: Could not add joint controls: {e}")
                
                # Add visibility controls
                with self.server.gui.add_folder("👁️ Display Options"):
                    show_visual_cb = self.server.gui.add_checkbox(
                        "Show visual meshes", initial_value=True
                    )
                    
                    @show_visual_cb.on_update
                    def toggle_visual(_):
                        if hasattr(self, 'urdf_vis') and self.urdf_vis:
                            self.urdf_vis.show_visual = show_visual_cb.value
                
                print("✓ Robot displayed in 3D scene using ViserUrdf with interactive controls")
                return
                
            except ImportError:
                print("ViserUrdf not available - robot visualization requires ViserUrdf")
                self.robot_info.value += "\n⚠️ ViserUrdf required for visualization"
                return
            except Exception as e:
                print(f"Error with ViserUrdf: {e}")
                self.robot_info.value += f"\n❌ Visualization error: {str(e)}"
                return
            
        except Exception as e:
            print(f"Error visualizing robot: {e}")
            import traceback
            traceback.print_exc()
    
    def _visualize_robot_description_robot(self, robot_obj, add_fext: bool=False):
        """Load robot from robot_descriptions package."""
        def _check_package_available(package_name: str) -> bool:
            """Check if a package is available for import."""
            try:
                __import__(package_name)
                return True
            except ImportError:
                return False
            
        if not _check_package_available("robot_descriptions"):
            raise ImportError("robot_descriptions package is not available")
        
        try:
            from robot_descriptions.loaders.yourdfpy import load_robot_description

            # Try to load robot description, with fallback to "_description" suffix
            current_robot_info = self.get_current_robot_info()
            robot_name = current_robot_info.get('name', 'unknown_robot')
            # r_name = self.current_robot_object.name
            print(f"Loading robot description for: {robot_name}")
            try:
                urdf = load_robot_description(f"{robot_name}_description")
            except ModuleNotFoundError:
                try:
                    urdf = load_robot_description(f"{robot_name}")
                except ModuleNotFoundError:
                    raise ModuleNotFoundError(
                        f"Robot description '{robot_name}' not found. "
                        f"Try specifying the full name like '(robot_name)_description' "
                        f"or check available robot descriptions."
                    )
                
            self._visualize_yourdfpy_robot(urdf)
            
        except ImportError as e:
            raise ImportError(f"Required packages not available for robot_description loader: {e}")
        except Exception as e:
            raise RuntimeError(f"Failed to load robot description '{robot_name}': {e}")
    
    def _clear_robot_visualization(self):
        """Clear existing robot visualization from the scene."""
        try:
            # Remove ViserUrdf visualization if it exists
            if hasattr(self, 'urdf_vis') and self.urdf_vis is not None:
                try:
                    self.server.scene.remove("/robot")
                except Exception:
                    pass
                self.urdf_vis = None
            
            print("Cleared existing robot visualization")
            
        except Exception as e:
            print(f"Error clearing robot visualization: {e}")
    
    # ============================================================================
    # PUBLIC API METHODS
    # ============================================================================
    
    def update_robot_examples(self, examples: list):
        """Update the available robot examples in the dropdown."""
        try:
            # This method is kept for backward compatibility
            # but the new UI uses loader-specific sources
            print(f"Legacy update_robot_examples called with {len(examples)} examples")
        except Exception as e:
            print(f"Error updating robot examples: {e}")
            traceback.print_exc()
    
    def get_current_robot(self):
        """Get the currently loaded robot object."""
        return self.current_robot_object
    
    def get_current_robot_info(self) -> Dict[str, Any]:
        """Get information about the currently loaded robot."""
        if self.current_robot_object is None:
            return {"status": "No robot loaded"}
        
        return {
            "status": "Robot loaded",
            "name": self.current_robot,
            "type": type(self.current_robot_object).__name__,
            "loader": getattr(self, 'loader_dropdown',
                              type('', (), {'value': 'unknown'})).value
        }