"""Streamlined task execution panel."""

import viser
import threading
import traceback
from typing import Dict, Any, Optional, Callable


class StreamlinedTaskPanel:
    """Streamlined task execution panel using Viser."""
    
    def __init__(self, server: viser.ViserServer, task_manager=None, debug: bool = False):
        self.server = server
        self.task_manager = task_manager
        self.debug = debug
        
        # State
        self.current_example = None
        self.current_robot = None
        self.current_data = None
        self.task_running = False
        self.task_thread = None
        
        # Callbacks
        self.on_task_start: Optional[Callable] = None
        self.on_task_progress: Optional[Callable] = None
        self.on_task_complete: Optional[Callable] = None
        self.on_task_error: Optional[Callable] = None
        
        self.setup_ui()
    
    def setup_ui(self):
        """Setup streamlined task UI."""
        try:
            # === TASK SELECTION ===
            with self.server.gui.add_folder("🎯 Task Selection"):
                self.task_type = self.server.gui.add_dropdown(
                    "Task Type",
                    options=[
                        "Select task...",
                        "calibration",
                        "identification", 
                        "optimization",
                        "trajectory_generation",
                        "execution"
                    ],
                    initial_value="Select task...",
                )
                self.task_type.on_update(self._on_task_type_changed)
                
                # Task description
                self.task_description = self.server.gui.add_text(
                    "Description",
                    initial_value="Select a task to see description",
                    disabled=True
                )
            
            # === QUICK CONFIGURATION ===
            self.config_folder = self.server.gui.add_folder("⚙️ Configuration")
            
            # === EXECUTION ===
            with self.server.gui.add_folder("🚀 Execution"):
                # Main action button
                self.execute_button = self.server.gui.add_button("▶️ Execute Task")
                self.execute_button.on_click(self._on_execute_task)
                
                # Progress and status
                self.progress_bar = self.server.gui.add_slider(
                    "Progress",
                    min=0.0,
                    max=100.0,
                    step=0.1,
                    initial_value=0.0,
                    disabled=True
                )
                
                self.task_status = self.server.gui.add_text(
                    "Status",
                    initial_value="Ready to execute tasks",
                    disabled=True
                )
                
                # Stop button (initially hidden)
                self.stop_button = self.server.gui.add_button("⏹️ Stop Task")
                self.stop_button.on_click(self._on_stop_task)
                self.stop_button.visible = False
            
        except Exception as e:
            print(f"Error setting up streamlined task panel: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_task_type_changed(self, _):
        """Handle task type selection change."""
        task_type = self.task_type.value
        
        # Update description
        descriptions = {
            "calibration": "Calibrate robot kinematic parameters using measurement data",
            "identification": "Identify dynamic parameters (inertia, friction, etc.)",
            "optimization": "Optimize robot parameters using various optimization methods", 
            "trajectory_generation": "Generate optimal trajectories for robot motion",
            "execution": "Execute and visualize robot motions"
        }
        
        if task_type in descriptions:
            self.task_description.value = descriptions[task_type]
            self._setup_task_config(task_type)
        else:
            self.task_description.value = "Select a task to see description"
            self._clear_config()
    
    def _setup_task_config(self, task_type: str):
        """Setup minimal configuration for task type."""
        # Clear previous config
        self._clear_config()
        
        with self.config_folder:
            if task_type == "calibration":
                self.calib_method = self.server.gui.add_dropdown(
                    "Method",
                    options=["Kinematic", "Camera", "Force"],
                    initial_value="Kinematic"
                )
                
                self.calib_iterations = self.server.gui.add_number(
                    "Max Iterations",
                    initial_value=100,
                    min=1,
                    max=1000
                )
                
            elif task_type == "identification":
                self.ident_type = self.server.gui.add_dropdown(
                    "Type",
                    options=["Dynamic", "Friction", "Inertial"],
                    initial_value="Dynamic"
                )
                
                self.use_base_params = self.server.gui.add_checkbox(
                    "Use Base Parameters",
                    initial_value=True
                )
                
            elif task_type == "optimization":
                self.optim_method = self.server.gui.add_dropdown(
                    "Method",
                    options=["L-BFGS-B", "SLSQP", "Trust-NCG"],
                    initial_value="L-BFGS-B"
                )
                
                self.max_iter = self.server.gui.add_number(
                    "Max Iterations",
                    initial_value=1000,
                    min=10,
                    max=10000
                )
                
            elif task_type == "trajectory_generation":
                self.traj_type = self.server.gui.add_dropdown(
                    "Type",
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
                
            elif task_type == "execution":
                self.execution_mode = self.server.gui.add_dropdown(
                    "Mode",
                    options=["Simulation", "Visualization", "Analysis"],
                    initial_value="Simulation"
                )
    
    def _clear_config(self):
        """Clear current configuration widgets."""
        try:
            # Remove all children from config folder
            if hasattr(self.config_folder, '_children'):
                for child in list(self.config_folder._children):
                    child.remove()
        except Exception as e:
            if self.debug:
                print(f"Error clearing config: {e}")
    
    def _on_execute_task(self, _):
        """Execute the selected task."""
        try:
            # Validation
            if self.task_type.value == "Select task...":
                self.task_status.value = "⚠️ Please select a task type"
                return
                
            if not self.current_robot:
                self.task_status.value = "⚠️ No robot loaded"
                return
            
            if self.task_running:
                self.task_status.value = "⚠️ Task already running"
                return
            
            # Collect configuration
            task_config = self._collect_task_config()
            
            # Start execution
            self._start_task_execution(task_config)
            
        except Exception as e:
            self.task_status.value = f"❌ Error starting task: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _collect_task_config(self) -> Dict[str, Any]:
        """Collect current task configuration."""
        config = {
            'type': self.task_type.value,
            'example': self.current_example,
            'robot': self.current_robot,
            'data': self.current_data
        }
        
        task_type = self.task_type.value
        
        if task_type == "calibration":
            config.update({
                'method': getattr(self, 'calib_method', None).value if hasattr(self, 'calib_method') else 'Kinematic',
                'max_iterations': getattr(self, 'calib_iterations', None).value if hasattr(self, 'calib_iterations') else 100
            })
            
        elif task_type == "identification":
            config.update({
                'identification_type': getattr(self, 'ident_type', None).value if hasattr(self, 'ident_type') else 'Dynamic',
                'use_base_params': getattr(self, 'use_base_params', None).value if hasattr(self, 'use_base_params') else True
            })
            
        elif task_type == "optimization":
            config.update({
                'method': getattr(self, 'optim_method', None).value if hasattr(self, 'optim_method') else 'L-BFGS-B',
                'max_iterations': getattr(self, 'max_iter', None).value if hasattr(self, 'max_iter') else 1000
            })
            
        elif task_type == "trajectory_generation":
            config.update({
                'trajectory_type': getattr(self, 'traj_type', None).value if hasattr(self, 'traj_type') else 'Random',
                'duration': getattr(self, 'duration', None).value if hasattr(self, 'duration') else 10.0
            })
            
        elif task_type == "execution":
            config.update({
                'mode': getattr(self, 'execution_mode', None).value if hasattr(self, 'execution_mode') else 'Simulation'
            })
        
        return config
    
    def _start_task_execution(self, task_config: Dict[str, Any]):
        """Start task execution in separate thread."""
        self.task_running = True
        self.progress_bar.value = 0.0
        self.task_status.value = f"🔄 Starting {task_config['type']} task..."
        
        # Update UI for running state
        self.execute_button.visible = False
        self.stop_button.visible = True
        
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
            if self.task_manager:
                results = self.task_manager.execute_task(
                    task_config,
                    progress_callback=self._task_progress_callback
                )
                
                # Task completed successfully
                self._task_completed(results, task_config)
            else:
                # Simulate task execution for demo
                self._simulate_task_execution(task_config)
                
        except Exception as e:
            self._task_failed(e, task_config)
    
    def _simulate_task_execution(self, task_config: Dict[str, Any]):
        """Simulate task execution for demo purposes."""
        import time
        
        for i in range(101):
            if not self.task_running:  # Check if stopped
                return
                
            time.sleep(0.05)  # 5 second total simulation
            progress = i
            
            self._task_progress_callback({
                'progress': progress,
                'message': f"Processing {task_config['type']} step {i}"
            })
        
        # Simulate successful completion
        results = {
            'task_type': task_config['type'],
            'success': True,
            'message': f"{task_config['type']} completed successfully",
            'data': {'simulated': True}
        }
        self._task_completed(results, task_config)
    
    def _task_progress_callback(self, progress_info: Dict[str, Any]):
        """Handle task progress updates."""
        progress = progress_info.get('progress', 0)
        message = progress_info.get('message', 'Processing...')
        
        self.progress_bar.value = progress
        self.task_status.value = f"🔄 {message}"
        
        # Notify callback
        if self.on_task_progress:
            self.on_task_progress(progress_info)
    
    def _task_completed(self, results, task_config):
        """Handle task completion."""
        self.task_running = False
        self.progress_bar.value = 100.0
        self.task_status.value = f"✅ {task_config['type']} completed successfully"
        
        # Reset UI
        self.execute_button.visible = True
        self.stop_button.visible = False
        
        # Notify callback
        if self.on_task_complete:
            self.on_task_complete(results)
    
    def _task_failed(self, error, task_config):
        """Handle task failure."""
        self.task_running = False
        self.task_status.value = f"❌ {task_config['type']} failed: {str(error)}"
        
        # Reset UI
        self.execute_button.visible = True
        self.stop_button.visible = False
        
        if self.debug:
            traceback.print_exc()
        
        # Notify error callback
        if self.on_task_error:
            self.on_task_error({
                'message': str(error),
                'task_type': task_config['type'],
                'traceback': traceback.format_exc() if self.debug else None
            })
    
    def _on_stop_task(self, _):
        """Stop the currently running task."""
        if self.task_running:
            self.task_running = False
            
            if self.task_manager:
                self.task_manager.stop_current_task()
            
            self.task_status.value = "⏹️ Task stopped by user"
            
            # Reset UI
            self.execute_button.visible = True
            self.stop_button.visible = False
    
    # Public interface methods
    def set_example(self, example_info):
        """Set the current example."""
        self.current_example = example_info
        
        # Update available tasks based on example
        if hasattr(example_info, 'tasks') and example_info.tasks:
            available_tasks = ["Select task..."] + example_info.tasks
        else:
            available_tasks = [
                "Select task...",
                "calibration",
                "identification", 
                "optimization",
                "trajectory_generation",
                "execution"
            ]
        
        self.task_type.options = available_tasks
        self.task_type.value = "Select task..."
        
        # Handle both string and object example info
        example_name = example_info if isinstance(example_info, str) else getattr(example_info, 'name', str(example_info))
        self.task_status.value = f"Ready for {example_name} tasks"
    
    def set_robot(self, robot_info):
        """Set the current robot."""
        self.current_robot = robot_info
        
        if robot_info:
            robot_name = robot_info.get('name', 'Unknown')
            self.task_status.value = f"Robot ready: {robot_name}"
    
    def set_data(self, data_info):
        """Set the current data."""
        self.current_data = data_info
        
        if data_info:
            success_count = data_info.get('success_count', 0)
            total_count = data_info.get('total_count', 0)
            self.task_status.value = f"Data ready: {success_count}/{total_count} files"
