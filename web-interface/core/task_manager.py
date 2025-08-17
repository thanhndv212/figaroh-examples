class TaskManager:
    def __init__(self):
        self.tasks = []
        self.current_example_path = None
        self.current_task = None
        self.task_stopped = False

    def set_example_path(self, example_path: str):
        """Set the current example directory for task execution."""
        self.current_example_path = example_path
        print(f"📂 Task execution directory set to: {example_path}")

    def execute_task(self, task_config: dict, progress_callback=None):
        """Execute a task with the given configuration and callback."""
        import os
        
        self.task_stopped = False
        task_type = task_config.get('type', 'execution')
        example = task_config.get('example')
        
        # Determine example path
        if example and hasattr(example, 'path'):
            example_path = example.path
        elif self.current_example_path:
            example_path = self.current_example_path
        else:
            raise ValueError("No example path specified for task execution")
            
        if not os.path.exists(example_path):
            raise FileNotFoundError(
                f"Example directory {example_path} not found"
            )
        
        # Update progress
        if progress_callback:
            progress_callback({
                'progress': 0,
                'message': f'Starting {task_type} task...'
            })
        
        # Save current directory
        original_dir = os.getcwd()
        
        try:
            # Change to example directory
            os.chdir(example_path)
            print(f"🔄 Executing task '{task_type}' in {example_path}")
            
            # Look for common task scripts
            task_scripts = [
                f"{task_type}.py",
                "calibration.py",
                "identification.py",
                "main.py"
            ]
            
            if progress_callback:
                progress_callback({
                    'progress': 20,
                    'message': 'Locating task script...'
                })
            
            for script in task_scripts:
                if self.task_stopped:
                    raise InterruptedError("Task execution stopped by user")
                    
                script_path = os.path.join(example_path, script)
                if os.path.exists(script_path):
                    print(f"🚀 Running script: {script}")
                    
                    if progress_callback:
                        progress_callback({
                            'progress': 40,
                            'message': f'Executing {script}...'
                        })
                    
                    # Simulate task execution with progress updates
                    result = self._execute_script_with_progress(
                        script, example_path, task_config, progress_callback
                    )
                    
                    if result["success"]:
                        if progress_callback:
                            progress_callback({
                                'progress': 100,
                                'message': 'Task completed successfully'
                            })
                        print("✅ Task completed successfully")
                        return {
                            "success": True,
                            "output": result.get("output", "")
                        }
                    else:
                        error_msg = result.get('error',
                                               'Task execution failed')
                        print(f"❌ Task failed with error: {error_msg}")
                        raise RuntimeError(error_msg)
            
            # If no script found, return info
            raise FileNotFoundError(
                f"No executable script found for task '{task_type}'"
            )
            
        finally:
            # Restore original directory
            os.chdir(original_dir)

    def _execute_script_with_progress(self, script, example_path,
                                      task_config, progress_callback):
        """Execute script with simulated progress updates."""
        import subprocess
        import threading
        
        # Start the actual script execution
        process = subprocess.Popen(
            ["python", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=example_path
        )
        
        # Simulate progress updates while process runs
        progress_thread = threading.Thread(
            target=self._simulate_progress,
            args=(process, progress_callback)
        )
        progress_thread.start()
        
        # Wait for completion
        stdout, stderr = process.communicate()
        progress_thread.join()
        
        if process.returncode == 0:
            return {"success": True, "output": stdout}
        else:
            return {"success": False, "error": stderr}

    def _simulate_progress(self, process, progress_callback):
        """Simulate progress updates for running task."""
        import time
        
        progress = 40
        while process.poll() is None and not self.task_stopped:
            time.sleep(0.5)
            progress = min(95, progress + 5)
            if progress_callback:
                progress_callback({
                    'progress': progress,
                    'message': 'Processing task...'
                })

    def stop_current_task(self):
        """Stop the currently running task."""
        self.task_stopped = True
        print("🛑 Task stop requested")

    def execute_task_in_example(self, task_name: str, example_path: str):
        """Execute a specific task in the given example directory."""
        import os
        import subprocess
        
        if not os.path.exists(example_path):
            raise FileNotFoundError(
                f"Example directory {example_path} not found"
            )
        
        # Save current directory
        original_dir = os.getcwd()
        
        try:
            # Change to example directory
            os.chdir(example_path)
            print(f"🔄 Executing task '{task_name}' in {example_path}")
            
            # Look for common task scripts
            task_scripts = [
                f"{task_name}.py",
                "calibration.py",
                "identification.py",
                "main.py"
            ]
            
            for script in task_scripts:
                script_path = os.path.join(example_path, script)
                if os.path.exists(script_path):
                    print(f"🚀 Running script: {script}")
                    result = subprocess.run(
                        ["python", script],
                        capture_output=True,
                        text=True,
                        cwd=example_path
                    )
                    
                    if result.returncode == 0:
                        print("✅ Task completed successfully")
                        print(f"Output: {result.stdout}")
                        return {"success": True, "output": result.stdout}
                    else:
                        print(f"❌ Task failed with error: {result.stderr}")
                        return {"success": False, "error": result.stderr}
            
            # If no script found, return info
            return {
                "success": False,
                "error": f"No executable script found for task '{task_name}'"
            }
            
        finally:
            # Restore original directory
            os.chdir(original_dir)

    def add_task(self, task):
        self.tasks.append(task)

    def remove_task(self, task):
        if task in self.tasks:
            self.tasks.remove(task)

    def execute_tasks(self):
        for task in self.tasks:
            task.run()

    def clear_tasks(self):
        self.tasks.clear()

    def get_task_list(self):
        return self.tasks.copy()
    
    def shutdown(self):
        """Shutdown the task manager and clean up resources."""
        try:
            print("🔧 Shutting down task manager...")
            self.clear_tasks()
            print("✅ Task manager shutdown complete")
        except Exception as e:
            print(f"❌ Error during task manager shutdown: {e}")
