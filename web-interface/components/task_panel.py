"""Task panel component for Viser interface."""

import viser
import threading
from typing import Dict, Any, Optional, Callable
import traceback


class TaskPanel:
    """Task execution panel using Viser."""
    
    def __init__(self, server: viser.ViserServer, task_manager=None,
                 debug: bool = False):
        """Initialize the task panel."""
        self.server = server
        self.task_manager = task_manager
        self.debug = debug
        
        # State
        self.current_example = None
        self.current_robot = None
        self.current_data = None
        self.current_task = None
        self.task_running = False
        self.task_thread = None
        
        # Callbacks
        self.on_task_start: Optional[Callable] = None
        self.on_task_progress: Optional[Callable] = None
        self.on_task_complete: Optional[Callable] = None
        self.on_task_error: Optional[Callable] = None
        
        # Setup UI
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the task panel UI components."""
        try:
            # Task selection
            self.task_type = self.server.gui.add_dropdown(
                "Task Type",
                options=[
                    "Select task type...",
                    "calibration",
                    "identification",
                    "optimization",
                    "trajectory_generation",
                    "execution",
                    "generic"
                ],
                initial_value="Select task type...",
            )
            self.task_type.on_update(self.on_task_type_changed)
            
            # Task configuration
            self.task_config_folder = self.server.gui.add_folder(
                "⚙️ Task Configuration"
            )
            
            # Execution controls
            with self.server.gui.add_folder("🎮 Execution"):
                self.run_button = self.server.gui.add_button("▶️ Run Task")
                self.run_button.on_click(self.run_task_callback)
                
                self.stop_button = self.server.gui.add_button("⏹️ Stop Task")
                self.stop_button.on_click(self.stop_task_callback)
                
                self.progress_bar = self.server.gui.add_slider(
                    "Progress",
                    min=0.0,
                    max=100.0,
                    step=0.1,
                    initial_value=0.0,
                    disabled=True
                )
                
            # Task status
            self.task_status = self.server.gui.add_text(
                "Task Status",
                initial_value="No task selected"
            )
            
            # Task parameters (will be populated based on task type)
            self.task_params_folder = self.server.gui.add_folder(
                "📝 Parameters"
            )
            
        except Exception as e:
            print(f"Error setting up task panel UI: {e}")
            traceback.print_exc()
    
    def set_example(self, example_info):
        """Set the current example."""
        self.current_example = example_info
        
        # Update available tasks
        if example_info.tasks:
            available_tasks = example_info.tasks
        else:
            available_tasks = ["execution"]
        task_options = ["Select task type..."] + available_tasks
        
        self.task_type.options = task_options
        self.task_type.value = "Select task type..."
        
        status_msg = f"Ready - Available tasks: {', '.join(available_tasks)}"
        self.task_status.value = status_msg
        
    def set_robot(self, robot_info):
        """Set the current robot."""
        self.current_robot = robot_info
        
    def set_data(self, data_info):
        """Set the current data."""
        self.current_data = data_info
        
    def on_task_type_changed(self, _):
        """Handle task type selection."""
        selected_task = self.task_type.value
        
        if selected_task == "Select task type...":
            return
            
        # Clear previous configuration
        try:
            for child in self.task_config_folder._children:
                child.remove()
        except Exception:
            pass
            
        try:
            for child in self.task_params_folder._children:
                child.remove()
        except Exception:
            pass
        
        # Setup task-specific configuration
        self.setup_task_configuration(selected_task)
        
        self.task_status.value = f"Task selected: {selected_task}"
        
    def setup_task_configuration(self, task_type: str):
        """Setup configuration for specific task type."""
        if task_type == "calibration":
            self.setup_calibration_config()
        elif task_type == "identification":
            self.setup_identification_config()
        elif task_type == "optimization":
            self.setup_optimization_config()
        elif task_type == "trajectory_generation":
            self.setup_trajectory_config()
        elif task_type == "execution":
            self.setup_execution_config()
        else:
            self.setup_generic_config()
            
    def setup_calibration_config(self):
        """Setup calibration task configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Calibration Configuration**")
            
            self.calib_method = self.server.gui.add_dropdown(
                "Method",
                options=["Kinematic", "Camera", "Force"],
                initial_value="Kinematic"
            )
            
            self.calib_iterations = self.server.gui.add_number(
                "Max Iterations",
                initial_value=100,
                min=1,
                max=1000,
                step=1
            )
            
            self.calib_tolerance = self.server.gui.add_number(
                "Tolerance",
                initial_value=1e-6,
                min=1e-12,
                max=1e-2,
                step=1e-7
            )
            
        with self.task_params_folder:
            self.use_regularization = self.server.gui.add_checkbox(
                "Use Regularization",
                initial_value=True
            )
            
            self.regularization_weight = self.server.gui.add_slider(
                "Regularization Weight",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=0.1
            )
            
    def setup_identification_config(self):
        """Setup parameter identification configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Parameter Identification**")
            
            self.ident_type = self.server.gui.add_dropdown(
                "Identification Type",
                options=["Dynamic", "Friction", "Inertial"],
                initial_value="Dynamic"
            )
            
            self.use_base_params = self.server.gui.add_checkbox(
                "Use Base Parameters",
                initial_value=True
            )
            
            self.qr_tolerance = self.server.gui.add_number(
                "QR Tolerance",
                initial_value=1e-8,
                min=1e-12,
                max=1e-4,
                step=1e-9
            )
            
        with self.task_params_folder:
            self.filter_data = self.server.gui.add_checkbox(
                "Filter Data",
                initial_value=True
            )
            
            self.filter_cutoff = self.server.gui.add_slider(
                "Filter Cutoff (Hz)",
                min=1.0,
                max=100.0,
                step=1.0,
                initial_value=10.0
            )
            
    def setup_optimization_config(self):
        """Setup optimization configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Optimization Configuration**")
            
            self.optim_method = self.server.gui.add_dropdown(
                "Method",
                options=["L-BFGS-B", "SLSQP", "Trust-NCG"],
                initial_value="L-BFGS-B"
            )
            
            self.max_iter = self.server.gui.add_number(
                "Max Iterations",
                initial_value=1000,
                min=10,
                max=10000,
                step=10
            )
            
        with self.task_params_folder:
            self.use_bounds = self.server.gui.add_checkbox(
                "Use Parameter Bounds",
                initial_value=True
            )
            
            self.ftol = self.server.gui.add_number(
                "Function Tolerance",
                initial_value=1e-9,
                min=1e-15,
                max=1e-3,
                step=1e-10
            )
            
    def setup_trajectory_config(self):
        """Setup trajectory generation configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Trajectory Configuration**")
            
            self.traj_type = self.server.gui.add_dropdown(
                "Trajectory Type",
                options=["Random", "Optimal", "Fourier"],
                initial_value="Random"
            )
            
            self.duration = self.server.gui.add_slider(
                "Duration (s)",
                min=1.0,
                max=60.0,
                step=1.0,
                initial_value=10.0
            )
            
            self.frequency = self.server.gui.add_slider(
                "Frequency (Hz)",
                min=0.1,
                max=10.0,
                step=0.1,
                initial_value=1.0
            )
            
        with self.task_params_folder:
            self.joint_limits = self.server.gui.add_checkbox(
                "Respect Joint Limits",
                initial_value=True
            )
            
            self.smooth_trajectory = self.server.gui.add_checkbox(
                "Smooth Trajectory",
                initial_value=True
            )
            
    def setup_execution_config(self):
        """Setup generic execution configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Execution Configuration**")
            
            self.execution_mode = self.server.gui.add_dropdown(
                "Mode",
                options=["Simulation", "Visualization", "Analysis"],
                initial_value="Simulation"
            )
            
        with self.task_params_folder:
            self.save_results = self.server.gui.add_checkbox(
                "Save Results",
                initial_value=True
            )
            
    def setup_generic_config(self):
        """Setup generic task configuration."""
        with self.task_config_folder:
            self.server.gui.add_markdown("**Generic Task Configuration**")
            
        with self.task_params_folder:
            self.verbose = self.server.gui.add_checkbox(
                "Verbose Output",
                initial_value=True
            )
            
    def run_task_callback(self, _):
        """Callback for run task button."""
        if self.task_running:
            self.task_status.value = "⚠️ Task already running"
            return
            
        if self.task_type.value == "Select task type...":
            self.task_status.value = "⚠️ Please select a task type"
            return
            
        if not self.current_robot:
            self.task_status.value = "⚠️ No robot loaded"
            return
            
        # Collect task configuration
        task_config = self.collect_task_config()
        
        # Start task execution
        self.start_task_execution(task_config)
        
    def collect_task_config(self) -> Dict[str, Any]:
        """Collect current task configuration."""
        config = {
            'type': self.task_type.value,
            'example': self.current_example,
            'robot': self.current_robot,
            'data': self.current_data
        }
        
        # Add task-specific parameters
        task_type = self.task_type.value
        
        if task_type == "calibration":
            config.update({
                'method': getattr(self, 'calib_method', None).value if hasattr(self, 'calib_method') else 'Kinematic',
                'max_iterations': getattr(self, 'calib_iterations', None).value if hasattr(self, 'calib_iterations') else 100,
                'tolerance': getattr(self, 'calib_tolerance', None).value if hasattr(self, 'calib_tolerance') else 1e-6,
                'use_regularization': getattr(self, 'use_regularization', None).value if hasattr(self, 'use_regularization') else True,
                'regularization_weight': getattr(self, 'regularization_weight', None).value if hasattr(self, 'regularization_weight') else 0.1
            })
            
        elif task_type == "identification":
            config.update({
                'identification_type': getattr(self, 'ident_type', None).value if hasattr(self, 'ident_type') else 'Dynamic',
                'use_base_params': getattr(self, 'use_base_params', None).value if hasattr(self, 'use_base_params') else True,
                'qr_tolerance': getattr(self, 'qr_tolerance', None).value if hasattr(self, 'qr_tolerance') else 1e-8,
                'filter_data': getattr(self, 'filter_data', None).value if hasattr(self, 'filter_data') else True,
                'filter_cutoff': getattr(self, 'filter_cutoff', None).value if hasattr(self, 'filter_cutoff') else 10.0
            })
            
        elif task_type == "optimization":
            config.update({
                'method': getattr(self, 'optim_method', None).value if hasattr(self, 'optim_method') else 'L-BFGS-B',
                'max_iterations': getattr(self, 'max_iter', None).value if hasattr(self, 'max_iter') else 1000,
                'use_bounds': getattr(self, 'use_bounds', None).value if hasattr(self, 'use_bounds') else True,
                'ftol': getattr(self, 'ftol', None).value if hasattr(self, 'ftol') else 1e-9
            })
            
        elif task_type == "trajectory_generation":
            config.update({
                'trajectory_type': getattr(self, 'traj_type', None).value if hasattr(self, 'traj_type') else 'Random',
                'duration': getattr(self, 'duration', None).value if hasattr(self, 'duration') else 10.0,
                'frequency': getattr(self, 'frequency', None).value if hasattr(self, 'frequency') else 1.0,
                'joint_limits': getattr(self, 'joint_limits', None).value if hasattr(self, 'joint_limits') else True,
                'smooth_trajectory': getattr(self, 'smooth_trajectory', None).value if hasattr(self, 'smooth_trajectory') else True
            })
            
        elif task_type == "execution":
            config.update({
                'mode': getattr(self, 'execution_mode', None).value if hasattr(self, 'execution_mode') else 'Simulation',
                'save_results': getattr(self, 'save_results', None).value if hasattr(self, 'save_results') else True
            })
            
        elif task_type == "generic":
            config.update({
                'verbose': getattr(self, 'verbose', None).value if hasattr(self, 'verbose') else True
            })
        
        return config
        
    def start_task_execution(self, task_config: Dict[str, Any]):
        """Start task execution in separate thread."""
        self.task_running = True
        self.progress_bar.value = 0.0
        self.task_status.value = f"🔄 Starting {task_config['type']} task..."
        
        # Notify callback
        if self.on_task_start:
            self.on_task_start(task_config)
        
        # Start execution thread
        self.task_thread = threading.Thread(
            target=self._execute_task_thread,
            args=(task_config,)
        )
        self.task_thread.start()
        
    def _execute_task_thread(self, task_config: Dict[str, Any]):
        """Execute task in separate thread."""
        try:
            # Use task manager to execute task
            results = self.task_manager.execute_task(
                task_config,
                progress_callback=self._task_progress_callback
            )
            
            # Task completed successfully
            self.task_running = False
            self.progress_bar.value = 100.0
            self.task_status.value = f"✅ {task_config['type']} completed"
            
            # Notify callback
            if self.on_task_complete:
                self.on_task_complete(results)
                
        except Exception as e:
            # Task failed
            self.task_running = False
            self.task_status.value = f"❌ Task failed: {str(e)}"
            
            if self.debug:
                traceback.print_exc()
            
            # Notify error callback
            if self.on_task_error:
                self.on_task_error({
                    'message': str(e),
                    'traceback': traceback.format_exc() if self.debug else None
                })
                
    def _task_progress_callback(self, progress_info: Dict[str, Any]):
        """Handle task progress updates."""
        progress = progress_info.get('progress', 0)
        message = progress_info.get('message', 'Processing...')
        
        self.progress_bar.value = progress
        self.task_status.value = f"🔄 {message}"
        
        # Notify callback
        if self.on_task_progress:
            self.on_task_progress(progress_info)
            
    def stop_task_callback(self, _):
        """Callback for stop task button."""
        if not self.task_running:
            self.task_status.value = "⚠️ No task running"
            return
            
        # Stop task
        self.task_manager.stop_current_task()
        self.task_running = False
        self.task_status.value = "⏹️ Task stopped by user"
