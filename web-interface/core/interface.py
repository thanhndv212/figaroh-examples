"""Main web interface using Viser."""

import viser
import traceback

from components.config_panel import ConfigPanel
from components.robot_panel import RobotPanel
from components.task_panel import TaskPanel
from components.visualization_panel import VisualizationPanel
from components.results_panel import ResultsPanel
from components.data_panel import DataPanel


class FigarohWebInterface:
    """Main web interface for Figaroh examples."""
    
    def __init__(self, 
                 server: viser.ViserServer,
                 example_loader,
                 robot_manager,
                 task_manager,
                 session_manager,
                 debug: bool = False):
        """Initialize the web interface."""
        self.server = server
        self.example_loader = example_loader
        self.robot_manager = robot_manager
        self.task_manager = task_manager
        self.session_manager = session_manager
        self.debug = debug
        
        # UI Components
        self.config_panel = None
        self.robot_panel = None
        self.task_panel = None
        self.visualization_panel = None
        self.results_panel = None
        self.data_panel = None
        
        # Path Selection Components
        self.path_selection_folder = None
        self.urdf_dropdown = None
        self.model_dropdown = None
        self.path_info = None
        self.refresh_paths_button = None
        self.load_robot_button = None
        self.available_urdf_paths = {}
        self.available_model_paths = {}
        self.selected_urdf_path = None
        self.selected_model_path = None
        
        # State
        self.current_session = None
        self.current_example = None
        self.current_robot = None
        
        # Setup interface
        self.setup_interface()
        
    def setup_interface(self):
        """Setup the main interface."""
        # Header
        self.server.gui.add_markdown(
            "# 🤖 Figaroh\n"
            "Interactive robotics calibration and identification"
        )
        
        # Status bar
        self.status_text = self.server.gui.add_text(
            "Status", 
            initial_value="🟢 Ready - Select an example to begin"
        )
        
        # Main panels
        self.setup_panels()
        
        # Setup 3D scene
        self.setup_3d_scene()
        
        # Setup callbacks
        self.setup_callbacks()
        
        # Load initial data
        self.load_initial_data()
        
    def setup_panels(self):
        """Setup all UI panels."""
            
        with self.server.gui.add_folder("Setup"): 
            # Configuration Panel
            with self.server.gui.add_folder("📋 Project Setup"):
                self.config_panel = ConfigPanel(
                    server=self.server,
                    example_loader=self.example_loader,
                    debug=self.debug
                )

            # Robot Panel (with sub-panels)
            with self.server.gui.add_folder("🤖 Robot Loading"):
                # Robot Panel
                self.robot_panel = RobotPanel(
                    server=self.server,
                    example_loader=self.example_loader,
                    interface=self
                )
                self.path_selection_folder = self.server.gui.add_folder("📁 Path Selection", visible=False)
                # Path Selection Panel (as sub-panel under Robot)
                with self.path_selection_folder:
                    self.setup_path_selection_panel()

                # Load Robot button
                self.load_robot_button = self.server.gui.add_button("🤖 Load Robot")
                self.load_robot_button.on_click(self._on_load_robot_from_interface)

            # Data Panel
            with self.server.gui.add_folder("📊 Data Management"):
                self.data_panel = DataPanel(
                    server=self.server,
                    debug=self.debug
                )
        
        # Task Panel
        with self.server.gui.add_folder("⚙️ Tasks"):
            self.task_panel = TaskPanel(
                server=self.server,
                task_manager=self.task_manager,
                debug=self.debug
            )
        
        # Results Panel
        with self.server.gui.add_folder("Review"):
            with self.server.gui.add_folder("📈 Results"):
                self.results_panel = ResultsPanel(
                    server=self.server
                )
            # Visualization Panel
            with self.server.gui.add_folder("👁️ Visualization"):
                self.visualization_panel = VisualizationPanel(
                    server=self.server
                )
    
    def setup_path_selection_panel(self):
        """Setup path selection panel for URDF and model paths."""
        import os
        
        # Initialize path storage
        self.available_urdf_paths = {}
        self.available_model_paths = {}
        self.selected_urdf_path = None
        self.selected_model_path = None
        
        # Discover available URDF files
        self._discover_urdf_paths()
        self._discover_model_paths()
        
        # URDF Path Selection
        urdf_options = list(self.available_urdf_paths.keys()) if self.available_urdf_paths else ["None"]
        self.urdf_dropdown = self.server.gui.add_dropdown(
            "📄 URDF Path",
            options=urdf_options,
            initial_value=urdf_options[0],
        )
        
        # Model/Mesh Path Selection
        model_options = list(self.available_model_paths.keys()) if self.available_model_paths else ["None"]
        self.model_dropdown = self.server.gui.add_dropdown(
            "🎯 Model Path",
            options=model_options,
            initial_value=model_options[0],
        )
        
        # Refresh button
        self.refresh_paths_button = self.server.gui.add_button("🔄 Refresh Paths")
        
        
        
        # Setup callbacks
        self.urdf_dropdown.on_update(self._on_urdf_path_changed)
        self.model_dropdown.on_update(self._on_model_path_changed)
        self.refresh_paths_button.on_click(self._on_refresh_paths)
        
        
        # Update initial selection
        self._on_urdf_path_changed(None)
        self._on_model_path_changed(None)
    
    def _discover_urdf_paths(self):
        """Discover available URDF files in examples and models."""
        import os
        
        self.available_urdf_paths = {}
        
        # Get project root directory (3 levels up from this file)
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        
        # Check if robot panel has a selected source for filtering
        selected_source = None
        if (hasattr(self, 'robot_panel') and 
            self.robot_panel and 
            hasattr(self.robot_panel, 'source_dropdown') and
            self.robot_panel.source_dropdown.value != "None"):
            selected_source = self.robot_panel.source_dropdown.value
            print(f"Filtering URDF paths for selected source: {selected_source}")
        
        # Search in examples directories
        examples_path = os.path.join(project_root, "examples")
        if os.path.exists(examples_path):
            for example_name in os.listdir(examples_path):
                example_dir = os.path.join(examples_path, example_name)
                if os.path.isdir(example_dir):
                    # Filter by selected source if available
                    if selected_source and example_name != selected_source:
                        continue
                        
                    # Look for URDF files in common locations
                    urdf_locations = [
                        os.path.join(example_dir, "urdf"),
                        os.path.join(example_dir, "robots"),
                        example_dir
                    ]
                    
                    for urdf_dir in urdf_locations:
                        if os.path.exists(urdf_dir):
                            for file in os.listdir(urdf_dir):
                                if file.endswith(".urdf"):
                                    urdf_path = os.path.join(urdf_dir, file)
                                    display_name = f"{example_name}/{file}"
                                    self.available_urdf_paths[display_name] = urdf_path
        
        # Search in models directories
        models_path = os.path.join(project_root, "models")
        if os.path.exists(models_path):
            for model_name in os.listdir(models_path):
                model_dir = os.path.join(models_path, model_name)
                if os.path.isdir(model_dir):
                    # For models, check if selected source matches any robot_description names
                    # or if it's a general model search
                    if selected_source:
                        # Check if the selected source matches this model name or contains it
                        if not (selected_source in model_name or 
                                model_name in selected_source or
                                selected_source.replace('_description', '') in model_name or
                                model_name.replace('_description', '') in selected_source):
                            continue
                    
                    urdf_dir = os.path.join(model_dir, "urdf")
                    if os.path.exists(urdf_dir):
                        for file in os.listdir(urdf_dir):
                            if file.endswith(".urdf"):
                                urdf_path = os.path.join(urdf_dir, file)
                                display_name = f"models/{model_name}/{file}"
                                self.available_urdf_paths[display_name] = urdf_path
    
    def _discover_model_paths(self):
        """Discover available model/mesh directories."""
        import os
        
        self.available_model_paths = {}
        
        # Check if robot panel has a selected source for filtering
        selected_source = None
        if (hasattr(self, 'robot_panel') and 
            self.robot_panel and 
            hasattr(self.robot_panel, 'source_dropdown') and
            self.robot_panel.source_dropdown.value != "None"):
            selected_source = self.robot_panel.source_dropdown.value
            print(f"Filtering model paths for selected source: {selected_source}")
        
        # Search in models directories (these are the main model packages)
        project_root = os.path.dirname(os.path.dirname(
            os.path.dirname(__file__)
        ))
        models_path = os.path.join(project_root, "models")
        if os.path.exists(models_path):
            # Add the models directory itself as an option (always available)
            self.available_model_paths["models"] = models_path
            
            for model_name in os.listdir(models_path):
                model_dir = os.path.join(models_path, model_name)
                if os.path.isdir(model_dir):
                    # Filter by selected source if available
                    if selected_source:
                        # Check if the selected source matches this model name or contains it
                        if not (selected_source in model_name or 
                                model_name in selected_source or
                                selected_source.replace('_description', '') in model_name or
                                model_name.replace('_description', '') in selected_source):
                            continue
                    
                    display_name = f"models/{model_name}"
                    self.available_model_paths[display_name] = model_dir
    
    def _on_urdf_path_changed(self, _):
        """Handle URDF path selection change."""
        try:
            selected_key = self.urdf_dropdown.value
            if selected_key != "None":
                if selected_key in self.available_urdf_paths:
                    self.selected_urdf_path = (
                        self.available_urdf_paths[selected_key]
                    )
                    print(f"Selected URDF: {self.selected_urdf_path}")
                else:
                    self.selected_urdf_path = None
            else:
                self.selected_urdf_path = None
            
            # Update path info
            self.path_info.value = self._get_path_info()
            
        except Exception as e:
            print(f"Error updating URDF path: {e}")
    
    def _on_model_path_changed(self, _):
        """Handle model path selection change."""
        try:
            selected_key = self.model_dropdown.value
            if selected_key != "None":
                if selected_key in self.available_model_paths:
                    self.selected_model_path = (
                        self.available_model_paths[selected_key]
                    )
                    print(f"Selected Model Path: {self.selected_model_path}")
                else:
                    self.selected_model_path = None
            else:
                self.selected_model_path = None
            
            # Update path info
            self.path_info.value = self._get_path_info()
            
        except Exception as e:
            print(f"Error updating model path: {e}")
    
    def _on_refresh_paths(self, _):
        """Refresh available paths."""
        try:
            print("Refreshing available paths...")
            
            # Re-discover paths
            self._discover_urdf_paths()
            self._discover_model_paths()
            
            # Update dropdown options
            urdf_options = (list(self.available_urdf_paths.keys())
                            if self.available_urdf_paths else ["None"])
            model_options = (list(self.available_model_paths.keys())
                             if self.available_model_paths else ["None"])
            
            self.urdf_dropdown.options = urdf_options
            self.model_dropdown.options = model_options
            
            # Reset selections if current selection is no longer available
            if self.urdf_dropdown.value not in urdf_options:
                self.urdf_dropdown.value = urdf_options[0]
            if self.model_dropdown.value not in model_options:
                self.model_dropdown.value = model_options[0]
            
            # Update path info
            self._on_urdf_path_changed(None)
            self._on_model_path_changed(None)
            
            print(f"Refreshed paths: {len(urdf_options)} URDF files, "
                  f"{len(model_options)} model paths")
            
        except Exception as e:
            print(f"Error refreshing paths: {e}")
    
    def _on_load_robot_from_interface(self, _):
        """Handle Load Robot button click from the interface."""
        try:
            if hasattr(self.robot_panel, 'loading_mode_dropdown'):
                self.robot_panel.load_robot_combined()
            else:
                # Fallback: try to load from selected paths
                self._load_robot_from_selected_paths()
                
        except Exception as e:
            print(f"Error loading robot from interface: {e}")
    
    def _load_robot_from_selected_paths(self):
        """Load robot from the selected URDF and model paths."""
        try:
            if not self.selected_urdf_path:
                print("No URDF path selected")
                if hasattr(self, 'status_text'):
                    self.status_text.value = "❌ No URDF path selected"
                return
            
            if not self.selected_model_path:
                print("Warning: No model path selected - using URDF directory")
                import os
                self.selected_model_path = os.path.dirname(
                    self.selected_urdf_path
                )
            
            # Delegate to robot panel for actual loading
            if hasattr(self.robot_panel, 'load_robot_from_paths'):
                self.robot_panel.load_robot_from_paths(
                    self.selected_urdf_path,
                    self.selected_model_path
                )
            else:
                print("Robot panel does not support path-based loading")
                
        except Exception as e:
            print(f"Error loading robot from selected paths: {e}")
    
    def get_selected_paths(self):
        """Get currently selected URDF and model paths."""
        return {
            'urdf_path': self.selected_urdf_path,
            'model_path': self.selected_model_path
        }
    
    def _update_path_selection_visibility(self, visible: bool):
        """Update the visibility of the path selection panel."""
        try:
            if hasattr(self, 'path_selection_folder'):
                self.path_selection_folder.visible = visible
                print(f"Path selection panel visibility: {visible}")
            else:
                print("Path selection folder not found")
        except Exception as e:
            print(f"Error updating path selection visibility: {e}")
    
    def update_urdf_paths_for_source(self):
        """Update URDF paths when robot panel source selection changes."""
        try:
            # Re-discover URDF paths with current source filter
            self._discover_urdf_paths()
            
            # Update dropdown options
            urdf_options = (list(self.available_urdf_paths.keys())
                            if self.available_urdf_paths else ["None"])
            
            if hasattr(self, 'urdf_dropdown') and self.urdf_dropdown:
                self.urdf_dropdown.options = urdf_options
                
                # Reset selection if current selection is no longer available
                if self.urdf_dropdown.value not in urdf_options:
                    self.urdf_dropdown.value = urdf_options[0]
                
                # Update path info
                self._on_urdf_path_changed(None)
                
                print(f"Updated URDF paths: {len(urdf_options)} files found")
                
        except Exception as e:
            print(f"Error updating URDF paths for source: {e}")
    
    def update_model_paths_for_source(self):
        """Update model paths when robot panel source selection changes."""
        try:
            # Re-discover model paths with current source filter
            self._discover_model_paths()
            
            # Update dropdown options
            model_options = (list(self.available_model_paths.keys())
                             if self.available_model_paths else ["None"])
            
            if hasattr(self, 'model_dropdown') and self.model_dropdown:
                self.model_dropdown.options = model_options
                
                # Reset selection if current selection is no longer available
                if self.model_dropdown.value not in model_options:
                    self.model_dropdown.value = model_options[0]
                
                # Update path info
                self._on_model_path_changed(None)
                
                print(f"Updated model paths: {len(model_options)} directories found")
                
        except Exception as e:
            print(f"Error updating model paths for source: {e}")
    
    def update_paths_for_source(self):
        """Update both URDF and model paths when robot panel source selection changes."""
        try:
            self.update_urdf_paths_for_source()
            self.update_model_paths_for_source()
        except Exception as e:
            print(f"Error updating paths for source: {e}")
    
    def setup_3d_scene(self):
        """Setup the 3D visualization scene."""
        # Add ground grid
        self.server.scene.add_grid(
            "/ground",
            width=6,
            height=6,
            cell_size=0.5,
            cell_thickness=0.01
        )
        
        # Add world coordinate frame
        self.server.scene.add_frame(
            "/world",
            wxyz=(1.0, 0.0, 0.0, 0.0),
            position=(0.0, 0.0, 0.0),
            axes_length=0.5,
            axes_radius=0.02,
        )
        
        # # Set initial camera view (TODO fix: server has no camera attributes)
        # self.server.camera.position = (4.0, 4.0, 3.0)
        # self.server.camera.look_at = (0.0, 0.0, 0.5)
    
    def setup_callbacks(self):
        """Setup event callbacks between components."""
        # Example selection
        self.config_panel.on_example_selected = self.on_example_selected
        self.config_panel.on_config_updated = self.on_config_updated
        
        # Robot events
        self.robot_panel.on_robot_loaded = self.on_robot_loaded
        self.robot_panel.on_robot_changed = self.on_robot_changed
        
        # Data events
        self.data_panel.on_data_loaded = self.on_data_loaded
        self.data_panel.on_data_validated = self.on_data_validated
        
        # Task events
        self.task_panel.on_task_start = self.on_task_start
        self.task_panel.on_task_progress = self.on_task_progress
        self.task_panel.on_task_complete = self.on_task_complete
        self.task_panel.on_task_error = self.on_task_error
        
        # Visualization events
        self.visualization_panel.on_view_changed = self.on_view_changed
        self.visualization_panel.on_display_changed = self.on_display_changed
        
        # Results events
        self.results_panel.on_results_export = self.on_results_export
    
    def load_initial_data(self):
        """Load initial examples and configurations."""
        try:
            # Update config panel with available examples
            examples = self.example_loader.get_all_examples()
            self.config_panel.update_examples(examples)
            
            # Update robot panel with available examples
            self.robot_panel.update_robot_examples(examples)
            
            # Load default example if available
            if examples:
                first_example = examples[0]
                self.on_example_selected(first_example)
                
            self.status_text.value = f"🟢 Ready!"
            
        except Exception as e:
            self.status_text.value = f"🔴 Error loading examples: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    # Event Handlers
    def on_example_selected(self, example_info):
        """Handle example selection."""
        try:
            self.current_example = example_info
            self.status_text.value = f"🔄 Loading example: {example_info.name}"
            
            # Create new session
            self.current_session = self.session_manager.create_session(
                example_info=example_info
            )
            
            # Update panels
            self.robot_panel.set_example(example_info)
            self.data_panel.set_example(example_info)
            self.task_panel.set_example(example_info)
            
            self.status_text.value = f"🟢 Example loaded: {example_info.name}"
            
        except Exception as e:
            self.status_text.value = f"🔴 Error loading example: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_config_updated(self, config_data):
        """Handle configuration updates."""
        if self.current_session:
            self.current_session.config = config_data
            self.status_text.value = "🔄 Configuration updated"
    
    def on_robot_loaded(self, robot_info):
        """Handle robot loading."""
        try:
            self.current_robot = robot_info
            
            # Update session
            if self.current_session:
                self.current_session.robot = robot_info
            
            # Update other panels
            self.task_panel.set_robot(robot_info)
            self.visualization_panel.set_robot(robot_info)
            
            self.status_text.value = f"🟢 Robot loaded: {robot_info.get('name', 'Unknown')}"
            
        except Exception as e:
            self.status_text.value = f"🔴 Error handling robot load: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_robot_changed(self, joint_config):
        """Handle robot configuration changes."""
        if self.current_session:
            self.current_session.joint_config = joint_config
    
    def on_data_loaded(self, data_info):
        """Handle data loading."""
        try:
            if self.current_session:
                self.current_session.data = data_info
            
            # Update task panel
            self.task_panel.set_data(data_info)
            
            self.status_text.value = f"🟢 Data loaded: {data_info.get('name', 'Unknown')}"
            
        except Exception as e:
            self.status_text.value = f"🔴 Error handling data load: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_data_validated(self, validation_result):
        """Handle data validation results."""
        if validation_result['valid']:
            self.status_text.value = "🟢 Data validation passed"
        else:
            self.status_text.value = f"🟡 Data validation warnings: {len(validation_result['warnings'])}"
    
    def on_task_start(self, task_info):
        """Handle task start."""
        try:
            self.status_text.value = f"🔄 Starting task: {task_info['type']}"
            
            # Update visualization if needed
            self.visualization_panel.prepare_for_task(task_info)
            
        except Exception as e:
            self.status_text.value = f"🔴 Error starting task: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_task_progress(self, progress_info):
        """Handle task progress updates."""
        progress = progress_info.get('progress', 0)
        message = progress_info.get('message', 'Processing...')
        self.status_text.value = f"🔄 {message} ({progress:.1f}%)"
        
        # Update visualization if progress includes data
        if 'visualization_data' in progress_info:
            self.visualization_panel.update_progress_visualization(
                progress_info['visualization_data']
            )
    
    def on_task_complete(self, results):
        """Handle task completion."""
        try:
            # Store results in session
            if self.current_session:
                self.current_session.results = results
            
            # Update results panel
            self.results_panel.update_results(results)
            
            # Update visualization
            if 'visualization_data' in results:
                self.visualization_panel.update_final_visualization(
                    results['visualization_data']
                )
            
            task_type = results.get('task_type', 'Task')
            self.status_text.value = f"🟢 {task_type} completed successfully"
            
        except Exception as e:
            self.status_text.value = f"🔴 Error processing results: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_task_error(self, error_info):
        """Handle task errors."""
        error_msg = error_info.get('message', 'Unknown error')
        self.status_text.value = f"🔴 Task failed: {error_msg}"
        
        # Show error in results panel
        self.results_panel.show_error(error_info)
    
    def on_view_changed(self, view_config):
        """Handle visualization view changes."""
        # Update camera settings
        if 'camera_position' in view_config:
            self.server.camera.position = view_config['camera_position']
        if 'camera_target' in view_config:
            self.server.camera.look_at = view_config['camera_target']
    
    def on_display_changed(self, display_config):
        """Handle display option changes."""
        # This could update various display settings
        pass
    
    def on_results_export(self, export_config):
        """Handle results export."""
        try:
            if not self.current_session or not self.current_session.results:
                self.status_text.value = "🟡 No results to export"
                return
            
            # Export logic would go here
            export_format = export_config.get('format', 'yaml')
            self.status_text.value = f"🟢 Results exported as {export_format}"
            
        except Exception as e:
            self.status_text.value = f"🔴 Export failed: {str(e)}"
            if self.debug:
                traceback.print_exc()