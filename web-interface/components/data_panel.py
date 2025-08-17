"""Data panel component for Viser interface."""

import viser


class DataPanel:
    """Data management panel using Viser."""
    
    def __init__(self, server: viser.ViserServer):
        self.server = server
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the data panel UI components."""
        try:
            # Data source dropdown
            self.data_dropdown = self.server.gui.add_dropdown(
                "Data Source",
                options=["None", "Example Data"],
                initial_value="None",
            )
            
            # Load data button
            self.load_button = self.server.gui.add_button("Load Data")
            
        except Exception as e:
            print(f"Error setting up data panel: {e}")
