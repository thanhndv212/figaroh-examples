"""Configuration panel component for Viser interface."""

import viser
from typing import Dict, Any
import traceback
import yaml
import os
from pathlib import Path


class ConfigPanel:
    """Configuration panel using Viser."""
    
    def __init__(self, server: viser.ViserServer, example_loader=None, debug=False):
        self.server = server
        self.example_loader = example_loader
        self.debug = debug
        self.current_config = {}
        self.config_widgets = {}
        self.current_example_path = ""
        self.setup_ui()

        self.calib_param_folder = None
        self.iden_param_folder = None
        self.general_param_folder = None
    
    def setup_ui(self):
        """Setup the configuration panel UI components."""
        try:
            # Example selection dropdown
            self.example_dropdown = self.server.gui.add_dropdown(
                "Example",
                options=["Default"],
                initial_value="Default",
            )
            self.example_dropdown.on_update(self._on_example_changed)
            
            # Project configuration file dropdown
            self.config_file_dropdown = self.server.gui.add_dropdown(
                "Project Config File",
                options=["None"],
                initial_value="None",
            )
            self.config_file_dropdown.on_update(self._on_config_file_changed)
            
            # Load config button
            self.load_config_button = self.server.gui.add_button("Load Configuration")
            self.load_config_button.on_click(self._on_load_config)
            
            # Save config button
            self.save_config_button = self.server.gui.add_button("Save Configuration")
            self.save_config_button.on_click(self._on_save_config)
            
            # Configuration info
            self.config_info = self.server.gui.add_text(
                "Config Status",
                initial_value="No configuration loaded",
                disabled=True,
                visible=False
            )
            
        except Exception as e:
            if self.debug:
                print(f"Error setting up config panel UI: {e}")
                traceback.print_exc()
    
    def _on_example_changed(self, _):
        """Handle example selection change."""
        try:
            selected_example = self.example_dropdown.value
            if selected_example != "Default" and self.example_loader:
                # Get the path for this example
                self.current_example_path = self.example_loader.get_example_path(selected_example)
                self._discover_config_files()
                if self.debug:
                    print(f"Example changed to: {selected_example}")
                    print(f"Example path: {self.current_example_path}")
        except Exception as e:
            if self.debug:
                print(f"Error on example change: {e}")
                traceback.print_exc()
    
    def _discover_config_files(self):
        """Discover available configuration files in the current example."""
        try:
            if not self.current_example_path or not os.path.exists(self.current_example_path):
                self.config_file_dropdown.options = ["None"]
                return
            
            config_files = []
            config_dir = os.path.join(self.current_example_path, "config")
            
            # Look for YAML files in the config directory
            if os.path.exists(config_dir):
                for file in os.listdir(config_dir):
                    if file.endswith(('.yaml', '.yml')):
                        config_files.append(file)
            
            # Also look for config files in the root example directory
            for file in os.listdir(self.current_example_path):
                if file.endswith(('.yaml', '.yml')) and 'config' in file.lower():
                    config_files.append(file)
            
            if config_files:
                self.config_file_dropdown.options = ["None"] + sorted(config_files)
                if self.debug:
                    print(f"Found config files: {config_files}")
            else:
                self.config_file_dropdown.options = ["None"]
                if self.debug:
                    print("No config files found")
                    
        except Exception as e:
            if self.debug:
                print(f"Error discovering config files: {e}")
                traceback.print_exc()
    
    def _on_config_file_changed(self, _):
        """Handle configuration file selection change."""
        try:
            selected_file = self.config_file_dropdown.value
            if selected_file != "None":
                self.config_info.value = f"Selected: {selected_file}"
                if self.debug:
                    print(f"Config file selected: {selected_file}")
            else:
                self.config_info.value = "No config file selected"
        except Exception as e:
            if self.debug:
                print(f"Error on config file change: {e}")
                traceback.print_exc()
    
    def _on_load_config(self, _):
        """Handle configuration loading."""
        try:
            selected_file = self.config_file_dropdown.value
            if selected_file != "None" and self.current_example_path:
                self.load_config_file(selected_file)
        except Exception as e:
            if self.debug:
                print(f"Error loading config: {e}")
                traceback.print_exc()
    
    def _on_save_config(self, _):
        """Handle configuration saving."""
        try:
            selected_file = self.config_file_dropdown.value
            if selected_file != "None" and self.current_example_path:
                self.save_config_file(selected_file)
        except Exception as e:
            if self.debug:
                print(f"Error saving config: {e}")
                traceback.print_exc()
    
    def load_config_file(self, filename: str):
        """Load and parse a YAML configuration file."""
        try:
            # Try config directory first
            config_path = os.path.join(self.current_example_path, "config", filename)
            if not os.path.exists(config_path):
                # Try root example directory
                config_path = os.path.join(self.current_example_path, filename)
            
            if not os.path.exists(config_path):
                raise FileNotFoundError(f"Config file {filename} not found")
            
            with open(config_path, 'r') as f:
                self.current_config = yaml.safe_load(f)
            
            self.create_config_widgets()
            self.config_info.value = f"Loaded: {filename}"
            
            if self.debug:
                print(f"Successfully loaded config: {filename}")
                print(f"Config keys: {list(self.current_config.keys())}")
                
        except Exception as e:
            self.config_info.value = f"Error loading {filename}: {str(e)}"
            if self.debug:
                print(f"Error loading config file {filename}: {e}")
                traceback.print_exc()
    
    def create_config_widgets(self):
        """Create GUI widgets for configuration parameters with explanations."""
        try:
            # Clear existing widgets
            for widget_name, widget in self.config_widgets.items():
                try:
                    widget.remove()
                except:
                    pass
            self.config_widgets.clear()
            
            if not self.current_config:
                return
            
            # Create widgets for calibration parameters
            if 'calibration' in self.current_config:
                if self.calib_param_folder is not None:
                    self.calib_param_folder.remove()

                self._create_calibration_widgets()
            
            # Create widgets for identification parameters  
            if 'identification' in self.current_config:
                if self.iden_param_folder is not None:
                    self.iden_param_folder.remove()
                self._create_identification_widgets()

            # Create widgets for other top-level parameters
            if self.general_param_folder is not None:
                self.general_param_folder.remove()
            self._create_general_widgets()
            
        except Exception as e:
            if self.debug:
                print(f"Error creating config widgets: {e}")
                traceback.print_exc()
    
    def _create_calibration_widgets(self):
        """Create widgets for calibration parameters."""
        calib_config = self.current_config['calibration']
        self.calib_param_folder = self.server.gui.add_folder("📏 Calibration Parameters")
        with self.calib_param_folder:
            if 'calib_level' in calib_config:
                self.config_widgets['calib_level'] = self.server.gui.add_dropdown(
                    "Calibration Level",
                    options=["full_params", "joint_offset"],
                    initial_value=calib_config['calib_level'],
                )
                # Add explanation
                self.server.gui.add_markdown(
                    "Level of calibration: full_params (all parameters) or joint_offset (joint offsets only)"
                )
            
            if 'base_frame' in calib_config:
                self.config_widgets['base_frame'] = self.server.gui.add_text(
                    "Base Frame",
                    initial_value=calib_config['base_frame'],
                )
                self.server.gui.add_markdown("Robot base reference frame")
            
            if 'tool_frame' in calib_config:
                self.config_widgets['tool_frame'] = self.server.gui.add_text(
                    "Tool Frame", 
                    initial_value=calib_config['tool_frame'],
                )
                self.server.gui.add_markdown("End-effector tool frame")
            
            # Camera pose parameters with explanations
            if 'camera_pose' in calib_config:
                pose = calib_config['camera_pose']
                self.server.gui.add_markdown("📷 Camera Pose (X,Y,Z,Roll,Pitch,Yaw)")
                
                self.config_widgets['camera_px'] = self.server.gui.add_number(
                    "Camera X (m)", initial_value=pose[0], step=0.001
                )
                self.config_widgets['camera_py'] = self.server.gui.add_number(
                    "Camera Y (m)", initial_value=pose[1], step=0.001
                )
                self.config_widgets['camera_pz'] = self.server.gui.add_number(
                    "Camera Z (m)", initial_value=pose[2], step=0.001
                )
                self.config_widgets['camera_rx'] = self.server.gui.add_number(
                    "Camera Roll (rad)", initial_value=pose[3], step=0.01
                )
                self.config_widgets['camera_ry'] = self.server.gui.add_number(
                    "Camera Pitch (rad)", initial_value=pose[4], step=0.01
                )
                self.config_widgets['camera_rz'] = self.server.gui.add_number(
                    "Camera Yaw (rad)", initial_value=pose[5], step=0.01
                )
            
            # Other calibration parameters
            if 'outlier_eps' in calib_config:
                self.config_widgets['outlier_eps'] = self.server.gui.add_number(
                    "Outlier Threshold (m)",
                    initial_value=calib_config['outlier_eps'],
                    step=0.001,
                )
                self.server.gui.add_markdown("Threshold for outlier detection in meters")
            
            if 'nb_sample' in calib_config:
                self.config_widgets['nb_sample'] = self.server.gui.add_number(
                    "Number of Samples",
                    initial_value=calib_config['nb_sample'],
                    step=1,
                )
                self.server.gui.add_markdown("Number of data samples to use for calibration")
    
    def _create_identification_widgets(self):
        """Create widgets for identification parameters."""
        ident_config = self.current_config['identification']
        self.iden_param_folder = self.server.gui.add_folder("🔍 Identification Parameters")
        with self.iden_param_folder:
            if 'problem_params' in ident_config and ident_config['problem_params']:
                prob_params = ident_config['problem_params'][0]
                
                self.server.gui.add_markdown("Parameter Estimation Options:")
                
                self.config_widgets['has_friction'] = self.server.gui.add_checkbox(
                    "Include Friction", initial_value=prob_params.get('has_friction', False)
                )
                self.server.gui.add_markdown("Estimate friction parameters")
                
                self.config_widgets['has_joint_offset'] = self.server.gui.add_checkbox(
                    "Include Joint Offset", initial_value=prob_params.get('has_joint_offset', False)
                )
                self.server.gui.add_markdown("Estimate joint position offsets")
                
                self.config_widgets['has_actuator_inertia'] = self.server.gui.add_checkbox(
                    "Include Actuator Inertia", initial_value=prob_params.get('has_actuator_inertia', False)
                )
                self.server.gui.add_markdown("Estimate actuator inertia parameters")
    
    def _create_general_widgets(self):
        """Create widgets for general configuration parameters."""
        self.general_param_folder = self.server.gui.add_folder("⚙️ General Parameters")
        with self.general_param_folder:
            # Add widgets for other top-level config parameters
            for key, value in self.current_config.items():
                if key not in ['calibration', 'identification'] and isinstance(value, (str, int, float, bool)):
                    if isinstance(value, bool):
                        self.config_widgets[key] = self.server.gui.add_checkbox(
                            key.replace('_', ' ').title(), initial_value=value
                        )
                    elif isinstance(value, (int, float)):
                        self.config_widgets[key] = self.server.gui.add_number(
                            key.replace('_', ' ').title(), initial_value=value
                        )
                    else:
                        self.config_widgets[key] = self.server.gui.add_text(
                            key.replace('_', ' ').title(), initial_value=str(value)
                        )
    
    def save_config_file(self, filename: str):
        """Save the current configuration to a YAML file."""
        try:
            # Update config with current widget values
            self._update_config_from_widgets()
            
            # Save to config directory
            config_path = os.path.join(self.current_example_path, "config", filename)
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            
            with open(config_path, 'w') as f:
                yaml.dump(self.current_config, f, default_flow_style=False, indent=2)
            
            self.config_info.value = f"Saved: {filename}"
            if self.debug:
                print(f"Configuration saved to: {config_path}")
                
        except Exception as e:
            self.config_info.value = f"Error saving: {str(e)}"
            if self.debug:
                print(f"Error saving config: {e}")
                traceback.print_exc()
    
    def _update_config_from_widgets(self):
        """Update the configuration dictionary from current widget values."""
        try:
            # Update calibration parameters
            if 'calibration' in self.current_config:
                calib_config = self.current_config['calibration']
                
                if 'calib_level' in self.config_widgets:
                    calib_config['calib_level'] = self.config_widgets['calib_level'].value
                
                if 'base_frame' in self.config_widgets:
                    calib_config['base_frame'] = self.config_widgets['base_frame'].value
                    
                if 'tool_frame' in self.config_widgets:
                    calib_config['tool_frame'] = self.config_widgets['tool_frame'].value
                
                # Update camera pose
                if 'camera_pose' in calib_config:
                    pose = calib_config['camera_pose']
                    if 'camera_px' in self.config_widgets:
                        pose[0] = self.config_widgets['camera_px'].value
                    if 'camera_py' in self.config_widgets:
                        pose[1] = self.config_widgets['camera_py'].value
                    if 'camera_pz' in self.config_widgets:
                        pose[2] = self.config_widgets['camera_pz'].value
                    if 'camera_rx' in self.config_widgets:
                        pose[3] = self.config_widgets['camera_rx'].value
                    if 'camera_ry' in self.config_widgets:
                        pose[4] = self.config_widgets['camera_ry'].value
                    if 'camera_rz' in self.config_widgets:
                        pose[5] = self.config_widgets['camera_rz'].value
                
                if 'outlier_eps' in self.config_widgets:
                    calib_config['outlier_eps'] = self.config_widgets['outlier_eps'].value
                    
                if 'nb_sample' in self.config_widgets:
                    calib_config['nb_sample'] = int(self.config_widgets['nb_sample'].value)
            
            # Update identification parameters
            if 'identification' in self.current_config:
                ident_config = self.current_config['identification']
                if 'problem_params' in ident_config and ident_config['problem_params']:
                    prob_params = ident_config['problem_params'][0]
                    
                    if 'has_friction' in self.config_widgets:
                        prob_params['has_friction'] = self.config_widgets['has_friction'].value
                    if 'has_joint_offset' in self.config_widgets:
                        prob_params['has_joint_offset'] = self.config_widgets['has_joint_offset'].value
                    if 'has_actuator_inertia' in self.config_widgets:
                        prob_params['has_actuator_inertia'] = self.config_widgets['has_actuator_inertia'].value
            
            # Update general parameters
            for key, widget in self.config_widgets.items():
                if key not in ['calib_level', 'base_frame', 'tool_frame', 'camera_px', 'camera_py', 'camera_pz', 
                               'camera_rx', 'camera_ry', 'camera_rz', 'outlier_eps', 'nb_sample',
                               'has_friction', 'has_joint_offset', 'has_actuator_inertia']:
                    if hasattr(widget, 'value'):
                        self.current_config[key] = widget.value
                        
        except Exception as e:
            if self.debug:
                print(f"Error updating config from widgets: {e}")
                traceback.print_exc()

    def update_examples(self, examples: list):
        """Update the available examples list."""
        try:
            print(f"DEBUG: update_examples called with: {examples}")
            if examples:
                # Update dropdown with available examples
                self.example_dropdown.options = ["Default"] + examples
                if self.debug:
                    print(f"Updated examples list with {len(examples)} examples")
                    print(f"New dropdown options: {self.example_dropdown.options}")
            else:
                if self.debug:
                    print("No examples found to update")
        except Exception as e:
            if self.debug:
                print(f"Error updating examples: {e}")
                traceback.print_exc()

    def load_config(self, config_name: str):
        """Load a configuration (legacy method for compatibility)."""
        try:
            self.current_config = {"name": config_name}
            self.config_info.value = f"Legacy config loaded: {config_name}"
            if self.debug:
                print(f"Legacy configuration {config_name} loaded successfully")
        except Exception as e:
            if self.debug:
                print(f"Error loading legacy config {config_name}: {e}")
                traceback.print_exc()

    def update_robot_types(self, robot_types: list):
        """Update the available robot types."""
        try:
            if robot_types and self.debug:
                print(f"Available robot types: {robot_types}")
        except Exception as e:
            if self.debug:
                print(f"Error updating robot types: {e}")
                traceback.print_exc()
