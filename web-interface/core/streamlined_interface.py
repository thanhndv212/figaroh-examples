"""Streamlined web interface with consolidated panels."""

import viser
import traceback

from components.setup_panel import SetupPanel
from components.streamlined_task_panel import StreamlinedTaskPanel
from components.results_visualization_panel import ResultsVisualizationPanel


class StreamlinedFigarohInterface:
    """Streamlined web interface for Figaroh examples."""
    
    def __init__(self, 
                 server: viser.ViserServer,
                 example_loader,
                 robot_manager,
                 task_manager,
                 session_manager,
                 debug: bool = False):
        """Initialize the streamlined interface."""
        self.server = server
        self.example_loader = example_loader
        self.robot_manager = robot_manager
        self.task_manager = task_manager
        self.session_manager = session_manager
        self.debug = debug
        
        # UI Components
        self.setup_panel = None
        self.task_panel = None
        self.results_panel = None
        
        # State
        self.current_session = None
        self.current_example = None
        self.current_robot = None
        self.current_data = None
        
        # Setup interface
        self.setup_interface()
    
    def setup_interface(self):
        """Setup the streamlined interface."""
        # Header
        self.server.gui.add_markdown(
            "# 🤖 Figaroh Examples\n"
            "**Streamlined Interface** - Robotics Calibration & Identification"
        )
        
        # Global status
        self.status_text = self.server.gui.add_text(
            "🔄 Status", 
            initial_value="Ready - Use Setup panel to begin"
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
        """Setup the three main panels."""
        try:
            # === PANEL 1: SETUP (Robot + Data) ===
            with self.server.gui.add_folder("🔧 Setup", visible=True):
                self.setup_panel = SetupPanel(
                    server=self.server,
                    example_loader=self.example_loader,
                    robot_manager=self.robot_manager,
                    debug=self.debug
                )
            
            # === PANEL 2: TASK EXECUTION ===
            with self.server.gui.add_folder("⚙️ Execute", visible=True):
                self.task_panel = StreamlinedTaskPanel(
                    server=self.server,
                    task_manager=self.task_manager,
                    debug=self.debug
                )
            
            # === PANEL 3: RESULTS & VISUALIZATION ===
            with self.server.gui.add_folder("📊 Review", visible=True):
                self.results_panel = ResultsVisualizationPanel(
                    server=self.server,
                    debug=self.debug
                )
                
        except Exception as e:
            print(f"Error setting up streamlined panels: {e}")
            if self.debug:
                traceback.print_exc()
    
    def setup_3d_scene(self):
        """Setup the 3D visualization scene."""
        try:
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
            
        except Exception as e:
            print(f"Error setting up 3D scene: {e}")
            if self.debug:
                traceback.print_exc()
    
    def setup_callbacks(self):
        """Setup event callbacks between panels."""
        try:
            # Setup panel callbacks
            self.setup_panel.on_example_selected = self.on_example_selected
            self.setup_panel.on_robot_loaded = self.on_robot_loaded
            self.setup_panel.on_data_loaded = self.on_data_loaded
            
            # Task panel callbacks
            self.task_panel.on_task_start = self.on_task_start
            self.task_panel.on_task_progress = self.on_task_progress
            self.task_panel.on_task_complete = self.on_task_complete
            self.task_panel.on_task_error = self.on_task_error
            
            # Results panel callbacks
            self.results_panel.on_results_export = self.on_results_export
            
        except Exception as e:
            print(f"Error setting up callbacks: {e}")
            if self.debug:
                traceback.print_exc()
    
    def load_initial_data(self):
        """Load initial examples and configurations."""
        try:
            # Get available examples
            if self.example_loader:
                examples = self.example_loader.get_all_examples()
                self.setup_panel.update_examples(examples)
                
                self.status_text.value = f"Ready - {len(examples)} examples available"
            else:
                self.status_text.value = "Warning - No example loader available"
                
        except Exception as e:
            self.status_text.value = f"Error loading examples: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    # Event Handlers
    def on_example_selected(self, example_info):
        """Handle example selection from setup panel."""
        try:
            self.current_example = example_info
            
            # Create new session
            if self.session_manager:
                self.current_session = self.session_manager.create_session(
                    user_id=f"user_{example_info}"
                )
            
            # Update task panel
            self.task_panel.set_example(example_info)
            
            example_name = example_info if isinstance(example_info, str) else getattr(example_info, 'name', str(example_info))
            self.status_text.value = f"Example selected: {example_name}"
            
        except Exception as e:
            self.status_text.value = f"Error handling example selection: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_robot_loaded(self, robot_info):
        """Handle robot loading from setup panel."""
        try:
            self.current_robot = robot_info
            
            # Update session
            if self.current_session:
                self.current_session.robot = robot_info
            
            # Update other panels
            self.task_panel.set_robot(robot_info)
            self.results_panel.set_robot(robot_info)
            
            robot_name = robot_info.get('name', 'Unknown')
            self.status_text.value = f"Robot loaded: {robot_name}"
            
        except Exception as e:
            self.status_text.value = f"Error handling robot load: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_data_loaded(self, data_info):
        """Handle data loading from setup panel."""
        try:
            self.current_data = data_info
            
            # Update session
            if self.current_session:
                self.current_session.data = data_info
            
            # Update task panel
            self.task_panel.set_data(data_info)
            
            success_count = data_info.get('success_count', 0)
            total_count = data_info.get('total_count', 0)
            self.status_text.value = f"Data loaded: {success_count}/{total_count} files"
            
        except Exception as e:
            self.status_text.value = f"Error handling data load: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_task_start(self, task_info):
        """Handle task start from task panel."""
        try:
            # Prepare results panel for task
            self.results_panel.prepare_for_task(task_info)
            
            task_type = task_info.get('type', 'Unknown')
            self.status_text.value = f"Executing {task_type} task..."
            
        except Exception as e:
            self.status_text.value = f"Error handling task start: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_task_progress(self, progress_info):
        """Handle task progress updates."""
        try:
            progress = progress_info.get('progress', 0)
            message = progress_info.get('message', 'Processing...')
            
            self.status_text.value = f"🔄 {message} ({progress:.1f}%)"
            
            # Update results panel with progress visualization
            if 'visualization_data' in progress_info:
                self.results_panel.update_progress_visualization(
                    progress_info['visualization_data']
                )
                
        except Exception as e:
            if self.debug:
                print(f"Error handling task progress: {e}")
    
    def on_task_complete(self, results):
        """Handle task completion."""
        try:
            # Store results in session
            if self.current_session:
                self.current_session.results = results
            
            # Update results panel
            self.results_panel.update_results(results)
            
            # Update status
            task_type = results.get('task_type', 'Task')
            self.status_text.value = f"✅ {task_type} completed successfully"
            
        except Exception as e:
            self.status_text.value = f"Error handling task completion: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_task_error(self, error_info):
        """Handle task errors."""
        try:
            # Show error in results panel
            self.results_panel.show_error(error_info)
            
            # Update status
            error_msg = error_info.get('message', 'Unknown error')
            task_type = error_info.get('task_type', 'Task')
            self.status_text.value = f"❌ {task_type} failed: {error_msg}"
            
        except Exception as e:
            self.status_text.value = f"Error handling task error: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def on_results_export(self, export_config):
        """Handle results export."""
        try:
            format_type = export_config.get('format', 'YAML')
            self.status_text.value = f"📤 Results exported as {format_type}"
            
        except Exception as e:
            self.status_text.value = f"Export failed: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    # Public interface methods
    def get_current_state(self):
        """Get current interface state."""
        return {
            'example': self.current_example,
            'robot': self.current_robot,
            'data': self.current_data,
            'session': self.current_session
        }
