"""Consolidated setup panel for robot and data management."""

import viser
import os
import sys
import traceback
import json
import yaml
import numpy as np
import pandas as pd
import pickle
from typing import Dict, Any, Optional, Callable, List
from pathlib import Path

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


class SetupPanel:
    """Consolidated setup panel for robot and data management."""
    
    def __init__(self, server: viser.ViserServer,
                 example_loader=None, robot_manager=None, debug: bool = False):
        self.server = server
        self.example_loader = example_loader
        self.robot_manager = robot_manager
        self.debug = debug
        
        # Robot loading capabilities
        self.available_loaders = self._check_available_loaders()
        self.available_urdf_paths = {}
        self.available_model_paths = {}
        
        # State
        self.current_example = None
        self.current_example_path = None
        self.current_robot = None
        self.available_data_files = {}
        self.selected_data_files = []
        self.loaded_data = None
        
        # Callbacks
        self.on_example_selected: Optional[Callable] = None
        self.on_robot_loaded: Optional[Callable] = None
        self.on_data_loaded: Optional[Callable] = None
        
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
        """Setup the consolidated setup UI."""
        try:
            # === EXAMPLE SELECTION SECTION ===
            with self.server.gui.add_folder("📂 Project Setup"):
                # Example selection (primary driver)
                self.example_dropdown = self.server.gui.add_dropdown(
                    "🎯 Select Example",
                    options=["Loading..."],
                    initial_value="Loading...",
                )
                self.example_dropdown.on_update(self._on_example_changed)
                
                # Quick status
                self.setup_status = self.server.gui.add_text(
                    "Status",
                    initial_value="Select an example to begin",
                    disabled=True
                )
            
            # === ROBOT SECTION ===
            with self.server.gui.add_folder("🤖 Robot Configuration"):
                # Robot loading method
                self.robot_method = self.server.gui.add_dropdown(
                    "Loading Method",
                    options=["Auto (from example)", "Manual selection"],
                    initial_value="Auto (from example)"
                )
                self.robot_method.on_update(self._on_robot_method_changed)
                
                # Auto loading folder (visible by default)
                self.auto_robot_folder = self.server.gui.add_folder(
                    "Auto Loading", 
                    visible=True
                )
                
                with self.auto_robot_folder:
                    # Loader selection for auto mode (includes robot_description)
                    available_loader_names = [name for name, info in self.available_loaders.items() 
                                            if info.get("available", False)]
                    if not available_loader_names:
                        available_loader_names = ["figaroh"]  # Fallback
                    
                    self.auto_loader_dropdown = self.server.gui.add_dropdown(
                        "🔧 Robot Loader",
                        options=available_loader_names,
                        initial_value=available_loader_names[0],
                    )
                    self.auto_loader_dropdown.on_update(self._on_auto_loader_changed)
                    
                    # Robot source selection (depends on loader)
                    self.auto_source_dropdown = self.server.gui.add_dropdown(
                        "📁 Robot Source",
                        options=["None"],
                        initial_value="None",
                    )
                    self.auto_source_dropdown.on_update(self._on_auto_source_changed)
                    
                    # Additional parameters
                    self.add_fext_checkbox = self.server.gui.add_checkbox(
                        "🚀 Add Free-flyer Joint",
                        initial_value=False,
                    )
                
                # Manual selection folder (hidden by default)
                self.manual_robot_folder = self.server.gui.add_folder(
                    "Manual Selection", 
                    visible=False
                )
                
                with self.manual_robot_folder:
                    # Loader selection for manual mode (excludes robot_description)
                    manual_loader_names = [name for name, info in self.available_loaders.items() 
                                         if info.get("available", False) and name != "robot_description"]
                    if not manual_loader_names:
                        manual_loader_names = ["figaroh"]  # Fallback
                    
                    self.manual_loader_dropdown = self.server.gui.add_dropdown(
                        "🔧 Robot Loader",
                        options=manual_loader_names,
                        initial_value=manual_loader_names[0],
                    )
                    self.manual_loader_dropdown.on_update(self._on_manual_loader_changed)
                    
                    # URDF path selection
                    self.urdf_dropdown = self.server.gui.add_dropdown(
                        "📄 URDF File",
                        options=["None"],
                        initial_value="None"
                    )
                    self.urdf_dropdown.on_update(self._on_urdf_changed)
                    
                    # Model path selection
                    self.model_dropdown = self.server.gui.add_dropdown(
                        "🎯 Model Path",
                        options=["None"],
                        initial_value="None"
                    )
                    self.model_dropdown.on_update(self._on_model_changed)
                    
                    # Path refresh button
                    self.refresh_paths_button = self.server.gui.add_button(
                        "🔄 Refresh Paths"
                    )
                    self.refresh_paths_button.on_click(self._on_refresh_paths)
                
                # Robot info and actions (shared)
                self.robot_info = self.server.gui.add_text(
                    "Robot Info",
                    initial_value="No robot loaded",
                    disabled=True
                )
                
                self.load_robot_button = self.server.gui.add_button(
                    "🔧 Load Robot"
                )
                self.load_robot_button.on_click(self._on_load_robot)
                
                # Initialize auto loader
                self._on_auto_loader_changed(None)
            
            # === DATA SECTION ===
            with self.server.gui.add_folder("📊 Data Management"):
                # Available data files
                self.data_dropdown = self.server.gui.add_dropdown(
                    "Available Data Files",
                    options=["No example selected"],
                    initial_value="No example selected"
                )
                
                # Data selection controls
                with self.server.gui.add_folder("File Selection"):
                    self.add_data_button = self.server.gui.add_button(
                        "➕ Add to Selection"
                    )
                    self.add_data_button.on_click(self._on_add_data_file)
                    
                    self.selected_data_text = self.server.gui.add_text(
                        "Selected Files",
                        initial_value="None selected",
                        disabled=True
                    )
                    
                    self.clear_data_button = self.server.gui.add_button(
                        "🗑️ Clear Selection"
                    )
                    self.clear_data_button.on_click(self._on_clear_data_files)
                
                # Data loading
                self.load_data_button = self.server.gui.add_button(
                    "📥 Load Selected Data"
                )
                self.load_data_button.on_click(self._on_load_data)
                
                self.data_info = self.server.gui.add_text(
                    "Data Status",
                    initial_value="No data loaded",
                    disabled=True
                )
            
            # === QUICK ACTIONS ===
            with self.server.gui.add_folder("⚡ Quick Actions"):
                self.auto_setup_button = self.server.gui.add_button(
                    "🚀 Auto Setup (Robot + Data)"
                )
                self.auto_setup_button.on_click(self._on_auto_setup)
                
                self.reset_button = self.server.gui.add_button(
                    "🔄 Reset All"
                )
                self.reset_button.on_click(self._on_reset_all)
                
        except Exception as e:
            print(f"Error setting up setup panel: {e}")
            if self.debug:
                traceback.print_exc()
    
    def update_examples(self, examples):
        """Update available examples."""
        try:
            if examples:
                # Handle both string names and objects with name attribute
                if isinstance(examples[0], str):
                    example_names = examples
                else:
                    example_names = [ex.name for ex in examples]
                    
                self.example_dropdown.options = example_names
                self.example_dropdown.value = example_names[0]
                self._on_example_changed(None)
            else:
                self.example_dropdown.options = ["No examples found"]
                self.example_dropdown.value = "No examples found"
                
        except Exception as e:
            print(f"Error updating examples: {e}")
            if self.debug:
                traceback.print_exc()
            if self.debug:
                traceback.print_exc()
    
    def _on_example_changed(self, _):
        """Handle example selection change."""
        try:
            selected_name = self.example_dropdown.value
            if selected_name in ["Loading...", "No examples found"]:
                return
            
            # Find example info
            if self.example_loader:
                examples = self.example_loader.get_all_examples()
                if selected_name in examples:
                    self.current_example = selected_name
                    
                    # Get the example path from example_loader
                    self.current_example_path = self.example_loader.get_example_path(selected_name)
                    
                    # Update UI sections
                    self._update_robot_section()
                    self._update_data_section()
                    
                    self.setup_status.value = f"✅ Example: {selected_name}"
                    
                    # Notify callback
                    if self.on_example_selected:
                        self.on_example_selected(self.current_example)
                else:
                    self.setup_status.value = f"❌ Example not found: {selected_name}"
            
        except Exception as e:
            self.setup_status.value = f"❌ Error loading example: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _on_robot_method_changed(self, _):
        """Handle robot loading method change."""
        method = self.robot_method.value
        
        if method == "Manual selection":
            self.auto_robot_folder.visible = False
            self.manual_robot_folder.visible = True
            self._discover_paths()
        else:  # Auto mode
            self.auto_robot_folder.visible = True
            self.manual_robot_folder.visible = False
            self._on_auto_loader_changed(None)
    
    def _on_auto_loader_changed(self, _):
        """Handle auto loader selection change."""
        try:
            selected_loader = self.auto_loader_dropdown.value
            self._update_sources_for_loader(selected_loader, self.auto_source_dropdown)
        except Exception as e:
            print(f"Error changing auto loader: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_auto_source_changed(self, _):
        """Handle auto source selection change."""
        try:
            selected_source = self.auto_source_dropdown.value
            if selected_source != "None":
                self.robot_info.value = f"Source selected: {selected_source}"
            else:
                self.robot_info.value = "No robot source selected"
        except Exception as e:
            print(f"Error changing auto source: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_manual_loader_changed(self, _):
        """Handle manual loader selection change."""
        try:
            selected_loader = self.manual_loader_dropdown.value
            self.robot_info.value = f"Manual loader: {selected_loader}"
        except Exception as e:
            print(f"Error changing manual loader: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_urdf_changed(self, _):
        """Handle URDF path selection change."""
        try:
            selected_urdf = self.urdf_dropdown.value
            if selected_urdf in self.available_urdf_paths:
                urdf_path = self.available_urdf_paths[selected_urdf]
                self.robot_info.value = f"URDF: {os.path.basename(urdf_path)}"
        except Exception as e:
            print(f"Error changing URDF: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_model_changed(self, _):
        """Handle model path selection change."""
        try:
            selected_model = self.model_dropdown.value
            if selected_model in self.available_model_paths:
                model_path = self.available_model_paths[selected_model]
                self.robot_info.value = f"Model: {os.path.basename(model_path)}"
        except Exception as e:
            print(f"Error changing model: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_refresh_paths(self, _):
        """Handle path refresh."""
        self._discover_paths()
    
    def _update_sources_for_loader(self, loader_name: str, source_dropdown):
        """Update available robot sources based on selected loader."""
        try:
            if loader_name == "robot_description":
                if list_available_robots:
                    try:
                        sources = list_available_robots()
                        if sources:
                            source_dropdown.options = sources
                            source_dropdown.value = sources[0]
                        else:
                            source_dropdown.options = ["No robots found"]
                            source_dropdown.value = "No robots found"
                    except Exception as e:
                        print(f"Error listing robots: {e}")
                        source_dropdown.options = ["Error loading robots"]
                        source_dropdown.value = "Error loading robots"
                else:
                    source_dropdown.options = ["robot_description not available"]
                    source_dropdown.value = "robot_description not available"
            
            elif loader_name in ["figaroh", "yourdfpy"]:
                # For figaroh/yourdfpy, use example names as sources
                if self.example_loader:
                    examples = self.example_loader.get_all_examples()
                    if examples:
                        source_dropdown.options = examples
                        source_dropdown.value = examples[0]
                    else:
                        source_dropdown.options = ["No examples found"]
                        source_dropdown.value = "No examples found"
                else:
                    source_dropdown.options = ["No example loader"]
                    source_dropdown.value = "No example loader"
            
            else:
                source_dropdown.options = ["Unknown loader type"]
                source_dropdown.value = "Unknown loader type"
                
        except Exception as e:
            print(f"Error updating sources for loader {loader_name}: {e}")
            if self.debug:
                traceback.print_exc()
            source_dropdown.options = [f"Error: {str(e)}"]
            source_dropdown.value = f"Error: {str(e)}"
    
    def _discover_paths(self):
        """Discover available URDF and model paths for manual selection."""
        try:
            self._discover_urdf_paths()
            self._discover_model_paths()
            self._update_path_dropdowns()
        except Exception as e:
            print(f"Error discovering paths: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _discover_urdf_paths(self):
        """Discover available URDF files."""
        self.available_urdf_paths = {}
        
        # Get project root directory (3 levels up from this file)
        project_root = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(__file__))
        ))
        
        # Search in examples directories
        examples_path = os.path.join(project_root, "examples")
        if os.path.exists(examples_path):
            for example_dir in os.listdir(examples_path):
                example_full_path = os.path.join(examples_path, example_dir)
                if os.path.isdir(example_full_path):
                    self._find_urdf_files(example_full_path, f"examples/{example_dir}")
        
        # Search in models directories
        models_path = os.path.join(project_root, "models")
        if os.path.exists(models_path):
            for model_dir in os.listdir(models_path):
                model_full_path = os.path.join(models_path, model_dir)
                if os.path.isdir(model_full_path):
                    self._find_urdf_files(model_full_path, f"models/{model_dir}")
    
    def _find_urdf_files(self, search_path: str, prefix: str):
        """Find URDF files in a directory."""
        try:
            for root, dirs, files in os.walk(search_path):
                for file in files:
                    if file.endswith('.urdf'):
                        full_path = os.path.join(root, file)
                        relative_path = os.path.relpath(root, search_path)
                        if relative_path == ".":
                            display_name = f"{prefix}/{file}"
                        else:
                            display_name = f"{prefix}/{relative_path}/{file}"
                        self.available_urdf_paths[display_name] = full_path
        except Exception as e:
            print(f"Error finding URDF files in {search_path}: {e}")
    
    def _discover_model_paths(self):
        """Discover available model directories."""
        self.available_model_paths = {}
        
        # Get project root directory
        project_root = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(__file__))
        ))
        
        # Search in models directories (these are the main model packages)
        models_path = os.path.join(project_root, "models")
        if os.path.exists(models_path):
            for model_dir in os.listdir(models_path):
                model_full_path = os.path.join(models_path, model_dir)
                if os.path.isdir(model_full_path):
                    display_name = f"models/{model_dir}"
                    self.available_model_paths[display_name] = model_full_path
        
        # Also search in examples for model directories
        examples_path = os.path.join(project_root, "examples")
        if os.path.exists(examples_path):
            for example_dir in os.listdir(examples_path):
                example_full_path = os.path.join(examples_path, example_dir)
                if os.path.isdir(example_full_path):
                    # Look for common model subdirectories
                    for subdir in ["models", "meshes", "urdf"]:
                        subdir_path = os.path.join(example_full_path, subdir)
                        if os.path.exists(subdir_path):
                            display_name = f"examples/{example_dir}/{subdir}"
                            self.available_model_paths[display_name] = subdir_path
    
    def _update_path_dropdowns(self):
        """Update the path selection dropdowns."""
        try:
            # Update URDF dropdown
            if self.available_urdf_paths:
                urdf_options = list(self.available_urdf_paths.keys())
                self.urdf_dropdown.options = urdf_options
                self.urdf_dropdown.value = urdf_options[0]
            else:
                self.urdf_dropdown.options = ["No URDF files found"]
                self.urdf_dropdown.value = "No URDF files found"
            
            # Update model dropdown
            if self.available_model_paths:
                model_options = list(self.available_model_paths.keys())
                self.model_dropdown.options = model_options
                self.model_dropdown.value = model_options[0]
            else:
                self.model_dropdown.options = ["No model paths found"]
                self.model_dropdown.value = "No model paths found"
                
        except Exception as e:
            print(f"Error updating path dropdowns: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _update_robot_section(self):
        """Update robot section based on current example."""
        if not self.current_example:
            return
            
        try:
            # Check if example has robot files
            robot_files = self._find_example_robot_files()
            
            if robot_files:
                self.robot_info.value = f"Found: {', '.join(robot_files.keys())}"
            else:
                self.robot_info.value = "No robot files found in example"
                
        except Exception as e:
            self.robot_info.value = f"Error checking robot files: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _update_data_section(self):
        """Update data section based on current example."""
        if not self.current_example_path:
            return
            
        try:
            # Discover data files
            self.available_data_files = {}
            data_dir = os.path.join(self.current_example_path, "data")
            
            if os.path.exists(data_dir):
                supported_exts = ['.csv', '.txt', '.json', '.yaml', '.yml', 
                                '.npy', '.npz', '.pkl', '.h5', '.hdf5']
                
                data_files = []
                for filename in os.listdir(data_dir):
                    filepath = os.path.join(data_dir, filename)
                    if os.path.isfile(filepath):
                        _, ext = os.path.splitext(filename)
                        if ext.lower() in supported_exts:
                            self.available_data_files[filename] = filepath
                            data_files.append(filename)
                
                if data_files:
                    options = ["Select a file..."] + sorted(data_files)
                    self.data_dropdown.options = options
                    self.data_dropdown.value = "Select a file..."
                else:
                    self.data_dropdown.options = ["No data files found"]
                    self.data_dropdown.value = "No data files found"
            else:
                self.data_dropdown.options = ["No data directory"]
                self.data_dropdown.value = "No data directory"
                
        except Exception as e:
            self.data_dropdown.options = [f"Error: {str(e)}"]
            self.data_dropdown.value = f"Error: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _find_example_robot_files(self):
        """Find robot files in the current example."""
        robot_files = {}
        
        if not self.current_example_path or not os.path.exists(self.current_example_path):
            return robot_files
        
        # Look for URDF files in common locations
        search_dirs = [
            os.path.join(self.current_example_path, "urdf"),
            os.path.join(self.current_example_path, "robots"),
            self.current_example_path
        ]
        
        for search_dir in search_dirs:
            if os.path.exists(search_dir):
                for filename in os.listdir(search_dir):
                    if filename.endswith('.urdf'):
                        robot_files[filename] = os.path.join(search_dir, filename)
        
        return robot_files
    
    def _discover_robot_paths(self):
        """Discover available robot paths for manual selection."""
        try:
            # This would be similar to the original path discovery logic
            # but simplified for the consolidated panel
            pass
        except Exception as e:
            if self.debug:
                print(f"Error discovering robot paths: {e}")
    
    def _on_add_data_file(self, _):
        """Add selected data file to selection."""
        try:
            selected_file = self.data_dropdown.value
            
            if (selected_file not in ["Select a file...", "No data files found", 
                                    "No data directory"] and
                selected_file not in self.selected_data_files):
                
                self.selected_data_files.append(selected_file)
                self._update_selected_data_display()
                
        except Exception as e:
            if self.debug:
                print(f"Error adding data file: {e}")
    
    def _on_clear_data_files(self, _):
        """Clear all selected data files."""
        self.selected_data_files = []
        self._update_selected_data_display()
    
    def _update_selected_data_display(self):
        """Update the selected data files display."""
        if not self.selected_data_files:
            self.selected_data_text.value = "None selected"
        else:
            files_text = ", ".join(self.selected_data_files)
            self.selected_data_text.value = f"{len(self.selected_data_files)} files: {files_text}"
    
    def _on_load_robot(self, _):
        """Load robot based on current method."""
        try:
            if self.robot_method.value == "Auto (from example)":
                self._load_robot_auto()
            else:
                self._load_robot_manual()
                
        except Exception as e:
            self.robot_info.value = f"❌ Error loading robot: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _load_robot_auto(self):
        """Auto-load robot using selected loader and source."""
        try:
            loader = self.auto_loader_dropdown.value
            source = self.auto_source_dropdown.value
            add_fext = self.add_fext_checkbox.value
            
            if source == "None" or source in ["No robots found", "Error loading robots"]:
                self.robot_info.value = "❌ No valid robot source selected"
                return
            
            if not load_robot:
                self.robot_info.value = "❌ Robot loading not available"
                return
            
            # Load robot using the selected loader and source
            print(f"Loading robot '{source}' with loader '{loader}' (fext: {add_fext})")
            
            if loader == "robot_description":
                # For robot_description, source is the robot name
                robot_obj = load_robot(
                    robot_urdf=source,
                    loader="robot_description",
                    isFext=add_fext
                )
            
            elif loader in ["figaroh", "yourdfpy"]:
                # For figaroh/yourdfpy, source is the example name
                if not self.example_loader:
                    raise ValueError("Example loader not available")
                
                # Get example path
                example_path = self.example_loader.get_example_path(source)
                if not example_path:
                    raise ValueError(f"Example '{source}' not found")
                
                # Get paths using helper method
                paths = self._get_robot_paths_for_example(example_path, source)
                if not paths['urdf_path']:
                    raise FileNotFoundError(f"No URDF file found for {source}")
                
                # Adjust package directories for loader
                package_dirs = self._adjust_package_dirs_for_loader(
                    paths['package_dirs'] or "models", loader
                )
                
                robot_obj = load_robot(
                    robot_urdf=paths['urdf_path'],
                    package_dirs=package_dirs,
                    loader=loader,
                    isFext=add_fext
                )
            
            else:
                raise ValueError(f"Unknown loader: {loader}")
            
            # Store and display robot
            self.current_robot = {
                'object': robot_obj,
                'name': source,
                'loader': loader,
                'type': type(robot_obj).__name__
            }
            
            self.robot_info.value = (
                f"✅ Loaded: {source}\n"
                f"Loader: {loader}\n"
                f"Type: {type(robot_obj).__name__}"
            )
            
            # Notify callback
            if self.on_robot_loaded:
                self.on_robot_loaded(self.current_robot)
            
            print(f"Successfully loaded robot '{source}' using '{loader}' loader")
            
        except Exception as e:
            self.robot_info.value = f"❌ Failed to load robot: {str(e)}"
            print(f"Error loading robot: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _get_robot_paths_for_example(self, example_path: str, source: str) -> dict:
        """Get URDF and package paths for an example."""
        try:
            # Find URDF in example directory
            urdf_path = self._find_urdf_in_example_dir(example_path)
            if urdf_path:
                return {
                    'urdf_path': urdf_path,
                    'package_dirs': self._get_intelligent_package_dirs(source)
                }
            
            # Otherwise, search in models directory based on source
            if source:
                project_root = os.path.dirname(os.path.dirname(
                    os.path.dirname(os.path.dirname(__file__))
                ))
                models_path = os.path.join(project_root, "models")
                
                if os.path.exists(models_path):
                    for model_name in os.listdir(models_path):
                        model_dir = os.path.join(models_path, model_name)
                        if (os.path.isdir(model_dir) and 
                            self._is_source_match(source, model_name)):
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
            print(f"Error getting robot paths for example: {e}")
            return {'urdf_path': None, 'package_dirs': None}
    
    def _find_urdf_in_example_dir(self, example_path: str) -> str:
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
            
        project_root = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(__file__))
        ))
        models_path = os.path.join(project_root, "models")
        
        if os.path.exists(models_path):
            # Look for matching model directory
            for model_name in os.listdir(models_path):
                model_dir = os.path.join(models_path, model_name)
                if (os.path.isdir(model_dir) and 
                    self._is_source_match(selected_source, model_name)):
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
    
    def _load_robot_manual(self):
        """Load robot from manual path selection."""
        try:
            loader = self.manual_loader_dropdown.value
            selected_urdf = self.urdf_dropdown.value
            selected_model = self.model_dropdown.value
            
            # Validate selections
            if selected_urdf not in self.available_urdf_paths:
                self.robot_info.value = "❌ No valid URDF file selected"
                return
            
            if selected_model not in self.available_model_paths:
                self.robot_info.value = "❌ No valid model path selected"
                return
            
            urdf_path = self.available_urdf_paths[selected_urdf]
            model_path = self.available_model_paths[selected_model]
            
            if not load_robot:
                self.robot_info.value = "❌ Robot loading not available"
                return
            
            # Extract robot name from URDF filename
            robot_name = os.path.splitext(os.path.basename(urdf_path))[0]
            
            print(f"Loading robot from paths: URDF={urdf_path}, Model={model_path}")
            
            # Validate inputs
            self._validate_path_loading_inputs(urdf_path, model_path, loader)
            
            # Adjust package directories and load robot
            adjusted_model_path = self._adjust_package_dirs_for_loader(model_path, loader)
            robot_obj = load_robot(
                robot_urdf=urdf_path,
                package_dirs=adjusted_model_path,
                loader=loader,
                isFext=False  # Manual loading doesn't need fext
            )
            
            # Store and display robot
            self.current_robot = {
                'object': robot_obj,
                'name': robot_name,
                'loader': loader,
                'type': type(robot_obj).__name__,
                'urdf_path': urdf_path,
                'model_path': model_path
            }
            
            self.robot_info.value = (
                f"✅ Loaded: {robot_name}\n"
                f"URDF: {os.path.basename(urdf_path)}\n"
                f"Loader: {loader}\n"
                f"Type: {type(robot_obj).__name__}"
            )
            
            # Notify callback
            if self.on_robot_loaded:
                self.on_robot_loaded(self.current_robot)
            
            print(f"Successfully loaded robot '{robot_name}' using '{loader}' loader")
            
        except Exception as e:
            self.robot_info.value = f"❌ Failed to load robot: {str(e)}"
            print(f"Error loading robot manually: {e}")
            if self.debug:
                traceback.print_exc()
    
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
    
    def _on_load_data(self, _):
        """Load selected data files."""
        try:
            if not self.selected_data_files:
                self.data_info.value = "⚠️ No files selected"
                return
            
            loaded_data = {}
            success_count = 0
            
            for filename in self.selected_data_files:
                if filename in self.available_data_files:
                    filepath = self.available_data_files[filename]
                    try:
                        data = self._load_file_data(filepath)
                        loaded_data[filename] = {
                            'data': data,
                            'filepath': filepath,
                            'loaded_successfully': True
                        }
                        success_count += 1
                    except Exception as e:
                        loaded_data[filename] = {
                            'data': None,
                            'filepath': filepath,
                            'error': str(e),
                            'loaded_successfully': False
                        }
            
            self.loaded_data = loaded_data
            total_count = len(self.selected_data_files)
            
            if success_count == total_count:
                self.data_info.value = f"✅ All {total_count} files loaded"
            else:
                self.data_info.value = f"⚠️ {success_count}/{total_count} files loaded"
            
            # Notify callback
            if self.on_data_loaded:
                self.on_data_loaded({
                    'loaded_data': loaded_data,
                    'success_count': success_count,
                    'total_count': total_count
                })
                
        except Exception as e:
            self.data_info.value = f"❌ Error loading data: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _load_file_data(self, filepath):
        """Load data from a file based on its extension."""
        _, ext = os.path.splitext(filepath)
        ext = ext.lower()
        
        if ext == '.csv':
            import pandas as pd
            return pd.read_csv(filepath)
        elif ext == '.txt':
            try:
                import numpy as np
                return np.loadtxt(filepath)
            except:
                with open(filepath, 'r') as f:
                    return f.read()
        elif ext == '.json':
            import json
            with open(filepath, 'r') as f:
                return json.load(f)
        elif ext in ['.yaml', '.yml']:
            import yaml
            with open(filepath, 'r') as f:
                return yaml.safe_load(f)
        elif ext == '.npy':
            import numpy as np
            return np.load(filepath)
        elif ext == '.npz':
            import numpy as np
            return np.load(filepath)
        elif ext == '.pkl':
            import pickle
            with open(filepath, 'rb') as f:
                return pickle.load(f)
        else:
            with open(filepath, 'r') as f:
                return f.read()
    
    def _on_auto_setup(self, _):
        """Perform automatic setup of robot and data."""
        try:
            # Load robot automatically
            self._load_robot_auto()
            
            # Select all available data files
            if self.available_data_files:
                self.selected_data_files = list(self.available_data_files.keys())
                self._update_selected_data_display()
                
                # Load data
                self._on_load_data(None)
            
            self.setup_status.value = "🚀 Auto setup completed"
            
        except Exception as e:
            self.setup_status.value = f"❌ Auto setup failed: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _on_reset_all(self, _):
        """Reset all selections and loaded data."""
        try:
            self.current_robot = None
            self.selected_data_files = []
            self.loaded_data = None
            
            self.robot_info.value = "No robot loaded"
            self.data_info.value = "No data loaded"
            self._update_selected_data_display()
            self.setup_status.value = "🔄 Reset completed"
            
        except Exception as e:
            self.setup_status.value = f"❌ Reset failed: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    # Public interface methods
    def get_current_example(self):
        """Get the currently selected example."""
        return self.current_example
    
    def get_current_robot(self):
        """Get the currently loaded robot."""
        return self.current_robot
    
    def get_loaded_data(self):
        """Get the currently loaded data."""
        return self.loaded_data
    
    def get_selected_data_files(self):
        """Get the list of selected data files."""
        return self.selected_data_files.copy()
