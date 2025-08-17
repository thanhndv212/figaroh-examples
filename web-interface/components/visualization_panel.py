"""Visualization panel component for Viser interface."""

import viser


class VisualizationPanel:
    """3D visualization panel using Viser."""
    
    def __init__(self, server: viser.ViserServer):
        self.server = server
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the visualization panel UI components."""
        try:
            # View controls
            self.view_button = self.server.gui.add_button("Reset View")
            
            # Visualization options
            self.show_frames = self.server.gui.add_checkbox("Show Frames", True)
            
        except Exception as e:
            print(f"Error setting up visualization panel: {e}")
