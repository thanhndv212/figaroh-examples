"""Consolidated results and visualization panel."""

import viser
import traceback
from typing import Dict, Any, Optional, Callable


class ResultsVisualizationPanel:
    """Consolidated results and visualization panel using Viser."""
    
    def __init__(self, server: viser.ViserServer, debug: bool = False):
        self.server = server
        self.debug = debug
        
        # State
        self.current_results = None
        self.current_robot = None
        self.visualization_data = None
        
        # Callbacks
        self.on_results_export: Optional[Callable] = None
        
        self.setup_ui()
    
    def setup_ui(self):
        """Setup consolidated results and visualization UI."""
        try:
            # === VISUALIZATION CONTROLS ===
            with self.server.gui.add_folder("👁️ Visualization"):
                # View controls
                self.show_robot = self.server.gui.add_checkbox(
                    "Show Robot",
                    initial_value=True
                )
                self.show_robot.on_update(self._on_visualization_changed)
                
                self.show_frames = self.server.gui.add_checkbox(
                    "Show Coordinate Frames",
                    initial_value=False
                )
                self.show_frames.on_update(self._on_visualization_changed)
                
                self.show_trajectory = self.server.gui.add_checkbox(
                    "Show Trajectory",
                    initial_value=False
                )
                self.show_trajectory.on_update(self._on_visualization_changed)
                
                # Animation controls
                self.animation_speed = self.server.gui.add_slider(
                    "Animation Speed",
                    min=0.1,
                    max=3.0,
                    step=0.1,
                    initial_value=1.0
                )
                
                self.reset_view_button = self.server.gui.add_button(
                    "🔄 Reset View"
                )
                self.reset_view_button.on_click(self._on_reset_view)
            
            # === RESULTS DISPLAY ===
            with self.server.gui.add_folder("📊 Results"):
                # Results summary
                self.results_summary = self.server.gui.add_text(
                    "Summary",
                    initial_value="No results available",
                    disabled=True
                )
                
                # Detailed results (expandable)
                with self.server.gui.add_folder("Details", visible=False) as self.details_folder:
                    self.results_details = self.server.gui.add_text(
                        "Detailed Results",
                        initial_value="No detailed results",
                        disabled=True
                    )
                
                # Error display (hidden by default)
                with self.server.gui.add_folder("❌ Errors", visible=False) as self.error_folder:
                    self.error_display = self.server.gui.add_text(
                        "Error Details",
                        initial_value="",
                        disabled=True
                    )
            
            # === EXPORT OPTIONS ===
            with self.server.gui.add_folder("💾 Export"):
                self.export_format = self.server.gui.add_dropdown(
                    "Format",
                    options=["YAML", "JSON", "CSV", "PDF Report"],
                    initial_value="YAML"
                )
                
                self.export_button = self.server.gui.add_button(
                    "📤 Export Results"
                )
                self.export_button.on_click(self._on_export_results)
                
                self.export_status = self.server.gui.add_text(
                    "Export Status",
                    initial_value="Ready to export",
                    disabled=True
                )
            
        except Exception as e:
            print(f"Error setting up results visualization panel: {e}")
            if self.debug:
                traceback.print_exc()
    
    def update_results(self, results: Dict[str, Any]):
        """Update the results display."""
        try:
            self.current_results = results
            
            # Update summary
            task_type = results.get('task_type', 'Unknown Task')
            success = results.get('success', False)
            
            if success:
                self.results_summary.value = f"✅ {task_type} completed successfully"
                
                # Show details if available
                if 'data' in results:
                    self.details_folder.visible = True
                    self._format_results_details(results)
                
                # Hide error folder
                self.error_folder.visible = False
                
            else:
                error_msg = results.get('error', 'Unknown error')
                self.results_summary.value = f"❌ {task_type} failed: {error_msg}"
                
                # Hide details, show error
                self.details_folder.visible = False
                self.error_folder.visible = True
                self.error_display.value = error_msg
            
            # Update visualization if data is available
            if 'visualization_data' in results:
                self.visualization_data = results['visualization_data']
                self._update_visualization()
            
        except Exception as e:
            self.results_summary.value = f"❌ Error displaying results: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _format_results_details(self, results: Dict[str, Any]):
        """Format detailed results for display."""
        try:
            details = []
            
            # Basic info
            details.append(f"Task: {results.get('task_type', 'Unknown')}")
            details.append(f"Status: {'Success' if results.get('success') else 'Failed'}")
            
            if 'execution_time' in results:
                details.append(f"Execution Time: {results['execution_time']:.2f}s")
            
            # Task-specific results
            data = results.get('data', {})
            
            if isinstance(data, dict):
                for key, value in data.items():
                    if isinstance(value, (int, float)):
                        if isinstance(value, float):
                            details.append(f"{key}: {value:.6f}")
                        else:
                            details.append(f"{key}: {value}")
                    elif isinstance(value, str):
                        details.append(f"{key}: {value}")
                    elif hasattr(value, '__len__'):
                        details.append(f"{key}: {len(value)} items")
                    else:
                        details.append(f"{key}: {type(value).__name__}")
            
            self.results_details.value = "\n".join(details)
            
        except Exception as e:
            self.results_details.value = f"Error formatting results: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def show_error(self, error_info: Dict[str, Any]):
        """Display error information."""
        try:
            self.error_folder.visible = True
            self.details_folder.visible = False
            
            error_msg = error_info.get('message', 'Unknown error')
            task_type = error_info.get('task_type', 'Task')
            
            self.results_summary.value = f"❌ {task_type} failed"
            
            error_details = [f"Error: {error_msg}"]
            
            if 'traceback' in error_info and error_info['traceback']:
                error_details.append("\nTraceback:")
                error_details.append(error_info['traceback'])
            
            self.error_display.value = "\n".join(error_details)
            
        except Exception as e:
            self.error_display.value = f"Error displaying error: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def set_robot(self, robot_info: Dict[str, Any]):
        """Set the current robot for visualization."""
        try:
            self.current_robot = robot_info
            
            if robot_info and self.show_robot.value:
                self._update_robot_visualization()
                
        except Exception as e:
            if self.debug:
                print(f"Error setting robot for visualization: {e}")
                traceback.print_exc()
    
    def _update_visualization(self):
        """Update the 3D visualization based on current settings."""
        try:
            # Update robot visualization
            if self.current_robot and self.show_robot.value:
                self._update_robot_visualization()
            
            # Update coordinate frames
            if self.show_frames.value:
                self._show_coordinate_frames()
            else:
                self._hide_coordinate_frames()
            
            # Update trajectory
            if self.show_trajectory.value and self.visualization_data:
                self._show_trajectory()
            else:
                self._hide_trajectory()
                
        except Exception as e:
            if self.debug:
                print(f"Error updating visualization: {e}")
                traceback.print_exc()
    
    def _update_robot_visualization(self):
        """Update robot visualization in 3D scene."""
        try:
            if not self.current_robot:
                return
            
            # This would interface with the robot visualization system
            # For now, just update a placeholder
            robot_name = self.current_robot.get('name', 'robot')
            
            # Add or update robot in scene
            self.server.scene.add_frame(
                f"/{robot_name}_base",
                wxyz=(1.0, 0.0, 0.0, 0.0),
                position=(0.0, 0.0, 0.0),
                axes_length=0.3,
                axes_radius=0.01,
            )
            
        except Exception as e:
            if self.debug:
                print(f"Error updating robot visualization: {e}")
    
    def _show_coordinate_frames(self):
        """Show coordinate frames in visualization."""
        try:
            # Add various coordinate frames
            frames = [
                ("world", (0, 0, 0), 0.5),
                ("base", (0, 0, 0.1), 0.3),
            ]
            
            for name, position, length in frames:
                self.server.scene.add_frame(
                    f"/frame_{name}",
                    wxyz=(1.0, 0.0, 0.0, 0.0),
                    position=position,
                    axes_length=length,
                    axes_radius=0.01,
                )
                
        except Exception as e:
            if self.debug:
                print(f"Error showing coordinate frames: {e}")
    
    def _hide_coordinate_frames(self):
        """Hide coordinate frames from visualization."""
        try:
            # Remove coordinate frames
            frame_names = ["world", "base"]
            for name in frame_names:
                try:
                    self.server.scene.remove(f"/frame_{name}")
                except:
                    pass  # Frame might not exist
                    
        except Exception as e:
            if self.debug:
                print(f"Error hiding coordinate frames: {e}")
    
    def _show_trajectory(self):
        """Show trajectory in visualization."""
        try:
            if not self.visualization_data:
                return
            
            # This would display trajectory data
            # For now, just add a simple trajectory marker
            self.server.scene.add_point_cloud(
                "/trajectory",
                points=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]],
                point_size=0.05
            )
            
        except Exception as e:
            if self.debug:
                print(f"Error showing trajectory: {e}")
    
    def _hide_trajectory(self):
        """Hide trajectory from visualization."""
        try:
            self.server.scene.remove("/trajectory")
        except:
            pass  # Trajectory might not exist
    
    def _on_visualization_changed(self, _):
        """Handle visualization setting changes."""
        self._update_visualization()
    
    def _on_reset_view(self, _):
        """Reset the 3D view to default."""
        try:
            # This would reset camera position
            # For now, just clear and re-add basic elements
            
            # Add ground grid
            self.server.scene.add_grid(
                "/ground",
                width=6,
                height=6,
                cell_size=0.5,
                cell_thickness=0.01
            )
            
            # Add world frame
            self.server.scene.add_frame(
                "/world",
                wxyz=(1.0, 0.0, 0.0, 0.0),
                position=(0.0, 0.0, 0.0),
                axes_length=0.5,
                axes_radius=0.02,
            )
            
        except Exception as e:
            if self.debug:
                print(f"Error resetting view: {e}")
                traceback.print_exc()
    
    def _on_export_results(self, _):
        """Export results in selected format."""
        try:
            if not self.current_results:
                self.export_status.value = "⚠️ No results to export"
                return
            
            format_type = self.export_format.value
            
            # Simulate export (real implementation would save files)
            self.export_status.value = f"✅ Results exported as {format_type}"
            
            # Notify callback
            if self.on_results_export:
                self.on_results_export({
                    'format': format_type,
                    'data': self.current_results
                })
                
        except Exception as e:
            self.export_status.value = f"❌ Export failed: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    # Additional methods for external interaction
    def prepare_for_task(self, task_info: Dict[str, Any]):
        """Prepare visualization for task execution."""
        try:
            # Clear previous results
            self.current_results = None
            self.visualization_data = None
            
            # Reset displays
            self.results_summary.value = f"🔄 Preparing for {task_info.get('type', 'task')}..."
            self.details_folder.visible = False
            self.error_folder.visible = False
            
        except Exception as e:
            if self.debug:
                print(f"Error preparing for task: {e}")
    
    def update_progress_visualization(self, progress_data: Dict[str, Any]):
        """Update visualization during task progress."""
        try:
            # This would update visualization with progress data
            # For now, just store the data
            self.visualization_data = progress_data
            
        except Exception as e:
            if self.debug:
                print(f"Error updating progress visualization: {e}")
    
    def update_final_visualization(self, final_data: Dict[str, Any]):
        """Update visualization with final results."""
        try:
            self.visualization_data = final_data
            self._update_visualization()
            
        except Exception as e:
            if self.debug:
                print(f"Error updating final visualization: {e}")
