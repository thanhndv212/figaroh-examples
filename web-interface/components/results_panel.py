"""Results panel component for Viser interface."""

import viser


class ResultsPanel:
    """Results display panel using Viser."""
    
    def __init__(self, server: viser.ViserServer):
        self.server = server
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the results panel UI components."""
        try:
            # Results display
            self.results_text = self.server.gui.add_text(
                "Results",
                initial_value="No results yet",
                disabled=True
            )
            
            # Export button
            self.export_button = self.server.gui.add_button("Export Results")
            
        except Exception as e:
            print(f"Error setting up results panel: {e}")
