"""Data panel component for Viser interface."""

import viser
import os
import traceback
from typing import Dict, Any, Optional, Callable, List
from pathlib import Path


class DataPanel:
    """Data management panel using Viser."""
    
    def __init__(self, server: viser.ViserServer, debug: bool = False):
        self.server = server
        self.debug = debug
        
        # State
        self.current_example = None
        self.current_example_path = None
        self.available_data_files = {}
        self.selected_data_files = []
        self.loaded_data = None
        
        # Callbacks
        self.on_data_loaded: Optional[Callable] = None
        self.on_data_validated: Optional[Callable] = None
        self.on_data_selected: Optional[Callable] = None
        
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the data panel UI components."""
        try:
            # Data file selection dropdown (multi-select style)
            self.data_file_dropdown = self.server.gui.add_dropdown(
                "📁 Available Data Files",
                options=["No example selected"],
                initial_value="No example selected",
            )
            self.data_file_dropdown.on_update(self._on_data_file_changed)
            
            # Selected files display
            self.selected_files_text = self.server.gui.add_text(
                "Selected Files",
                initial_value="None selected",
                disabled=True
            )
            
            # Add/Remove file buttons
            self.add_file_button = self.server.gui.add_button("➕ Add File")
            self.add_file_button.on_click(self._on_add_file)
            
            self.remove_file_button = self.server.gui.add_button("➖ Remove File")
            self.remove_file_button.on_click(self._on_remove_file)
            
            self.clear_files_button = self.server.gui.add_button("🗑️ Clear All")
            self.clear_files_button.on_click(self._on_clear_files)
            
            # Refresh data files button
            self.refresh_button = self.server.gui.add_button("🔄 Refresh Data Files")
            self.refresh_button.on_click(self._on_refresh_data_files)
            
            # Data file info
            self.data_info_text = self.server.gui.add_text(
                "Data Info",
                initial_value="No data files available",
                disabled=True
            )
            
            # Load data button
            self.load_button = self.server.gui.add_button("📊 Load Selected Data")
            self.load_button.on_click(self._on_load_data)
            
            # Data validation status
            self.validation_text = self.server.gui.add_text(
                "Validation Status",
                initial_value="No data loaded",
                disabled=True
            )
            
        except Exception as e:
            print(f"Error setting up data panel: {e}")
            if self.debug:
                traceback.print_exc()
    
    def set_example(self, example_info):
        """Set the current example and discover available data files."""
        try:
            self.current_example = example_info
            
            # Get example path
            if hasattr(example_info, 'path'):
                self.current_example_path = example_info.path
            elif hasattr(example_info, 'name'):
                # Try to construct path from example name
                project_root = os.path.dirname(os.path.dirname(
                    os.path.dirname(__file__)
                ))
                self.current_example_path = os.path.join(
                    project_root, "examples", example_info.name
                )
            else:
                self.current_example_path = None
            
            # Discover available data files
            self._discover_data_files()
            
            # Clear previous selections
            self.selected_data_files = []
            self._update_selected_files_display()
            
            # Update UI
            self._update_data_info()
            
        except Exception as e:
            print(f"Error setting example in data panel: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _discover_data_files(self):
        """Discover available data files in the current example's data directory."""
        self.available_data_files = {}
        
        if not self.current_example_path or not os.path.exists(self.current_example_path):
            self.data_file_dropdown.options = ["No example selected"]
            self.data_file_dropdown.value = "No example selected"
            return
        
        data_dir = os.path.join(self.current_example_path, "data")
        
        if not os.path.exists(data_dir):
            self.data_file_dropdown.options = ["No data directory found"]
            self.data_file_dropdown.value = "No data directory found"
            return
        
        # Find all data files
        data_files = []
        supported_extensions = ['.csv', '.txt', '.json', '.yaml', '.yml', '.npy', '.npz', '.pkl', '.h5', '.hdf5']
        
        try:
            for filename in os.listdir(data_dir):
                filepath = os.path.join(data_dir, filename)
                if os.path.isfile(filepath):
                    _, ext = os.path.splitext(filename)
                    if ext.lower() in supported_extensions:
                        self.available_data_files[filename] = filepath
                        data_files.append(filename)
        except Exception as e:
            print(f"Error reading data directory: {e}")
            if self.debug:
                traceback.print_exc()
        
        # Update dropdown options
        if data_files:
            options = ["Select a data file..."] + sorted(data_files)
            self.data_file_dropdown.options = options
            self.data_file_dropdown.value = "Select a data file..."
        else:
            self.data_file_dropdown.options = ["No data files found"]
            self.data_file_dropdown.value = "No data files found"
    
    def _on_data_file_changed(self, _):
        """Handle data file dropdown selection change."""
        try:
            selected_file = self.data_file_dropdown.value
            if selected_file in ["Select a data file...", "No data files found", "No example selected", "No data directory found"]:
                return
            
            # Update info for selected file
            if selected_file in self.available_data_files:
                self._update_data_info(selected_file)
            
        except Exception as e:
            print(f"Error handling data file selection: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_add_file(self, _):
        """Add the currently selected file to the selection list."""
        try:
            selected_file = self.data_file_dropdown.value
            if (selected_file not in ["Select a data file...", "No data files found", 
                                     "No example selected", "No data directory found"] and
                selected_file not in self.selected_data_files):
                
                self.selected_data_files.append(selected_file)
                self._update_selected_files_display()
                
                # Notify callback
                if self.on_data_selected:
                    self.on_data_selected({
                        'action': 'added',
                        'file': selected_file,
                        'selected_files': self.selected_data_files.copy()
                    })
                
        except Exception as e:
            print(f"Error adding data file: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_remove_file(self, _):
        """Remove the currently selected file from the selection list."""
        try:
            selected_file = self.data_file_dropdown.value
            if selected_file in self.selected_data_files:
                self.selected_data_files.remove(selected_file)
                self._update_selected_files_display()
                
                # Notify callback
                if self.on_data_selected:
                    self.on_data_selected({
                        'action': 'removed',
                        'file': selected_file,
                        'selected_files': self.selected_data_files.copy()
                    })
                
        except Exception as e:
            print(f"Error removing data file: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_clear_files(self, _):
        """Clear all selected files."""
        try:
            self.selected_data_files = []
            self._update_selected_files_display()
            
            # Notify callback
            if self.on_data_selected:
                self.on_data_selected({
                    'action': 'cleared',
                    'file': None,
                    'selected_files': []
                })
                
        except Exception as e:
            print(f"Error clearing data files: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _on_refresh_data_files(self, _):
        """Refresh the list of available data files."""
        try:
            self._discover_data_files()
            self._update_data_info()
            
        except Exception as e:
            print(f"Error refreshing data files: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _update_selected_files_display(self):
        """Update the display of selected files."""
        if not self.selected_data_files:
            self.selected_files_text.value = "None selected"
        else:
            files_text = "\n".join([f"• {f}" for f in self.selected_data_files])
            self.selected_files_text.value = files_text
    
    def _update_data_info(self, filename=None):
        """Update the data info display."""
        try:
            if not filename and not self.selected_data_files:
                if self.available_data_files:
                    count = len(self.available_data_files)
                    self.data_info_text.value = f"📁 {count} data files available in {os.path.basename(self.current_example_path) if self.current_example_path else 'current'} example"
                else:
                    self.data_info_text.value = "No data files available"
                return
            
            # Show info for specific file or selected files
            info_lines = []
            
            if filename and filename in self.available_data_files:
                filepath = self.available_data_files[filename]
                try:
                    file_size = os.path.getsize(filepath)
                    size_str = self._format_file_size(file_size)
                    info_lines.append(f"📄 {filename}")
                    info_lines.append(f"📏 Size: {size_str}")
                    info_lines.append(f"📂 Path: {filepath}")
                except Exception as e:
                    info_lines.append(f"📄 {filename}")
                    info_lines.append(f"⚠️ Error reading file info: {str(e)}")
            
            if self.selected_data_files:
                info_lines.append(f"✅ {len(self.selected_data_files)} file(s) selected")
            
            self.data_info_text.value = "\n".join(info_lines) if info_lines else "No data files available"
            
        except Exception as e:
            self.data_info_text.value = f"Error getting file info: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _format_file_size(self, size_bytes):
        """Format file size in human readable format."""
        if size_bytes == 0:
            return "0 B"
        
        size_names = ["B", "KB", "MB", "GB"]
        import math
        i = int(math.floor(math.log(size_bytes, 1024)))
        p = math.pow(1024, i)
        s = round(size_bytes / p, 2)
        return f"{s} {size_names[i]}"
    
    def _on_load_data(self, _):
        """Load the selected data files."""
        try:
            if not self.selected_data_files:
                self.validation_text.value = "⚠️ No files selected"
                return
            
            # Load data from selected files
            loaded_data = {}
            validation_results = []
            
            for filename in self.selected_data_files:
                if filename in self.available_data_files:
                    filepath = self.available_data_files[filename]
                    try:
                        data = self._load_file_data(filepath)
                        loaded_data[filename] = {
                            'data': data,
                            'filepath': filepath,
                            'loaded_successfully': True
                        }
                        validation_results.append(f"✅ {filename}: OK")
                    except Exception as e:
                        loaded_data[filename] = {
                            'data': None,
                            'filepath': filepath,
                            'error': str(e),
                            'loaded_successfully': False
                        }
                        validation_results.append(f"❌ {filename}: {str(e)}")
            
            self.loaded_data = loaded_data
            
            # Update validation status
            success_count = sum(1 for f in loaded_data.values() if f.get('loaded_successfully'))
            total_count = len(self.selected_data_files)
            
            if success_count == total_count:
                self.validation_text.value = f"✅ All {total_count} files loaded successfully"
            else:
                self.validation_text.value = f"⚠️ {success_count}/{total_count} files loaded successfully"
            
            # Notify callback
            if self.on_data_loaded:
                self.on_data_loaded({
                    'loaded_data': loaded_data,
                    'success_count': success_count,
                    'total_count': total_count,
                    'validation_results': validation_results
                })
            
            # Validate data
            self._validate_loaded_data(loaded_data)
            
        except Exception as e:
            self.validation_text.value = f"❌ Error loading data: {str(e)}"
            if self.debug:
                traceback.print_exc()
    
    def _load_file_data(self, filepath):
        """Load data from a file based on its extension."""
        _, ext = os.path.splitext(filepath)
        ext = ext.lower()
        
        if ext == '.csv':
            import pandas as pd
            return pd.read_csv(filepath)
        elif ext == '.txt':
            # Try to load as structured data first, fallback to raw text
            try:
                import numpy as np
                return np.loadtxt(filepath)
            except:
                with open(filepath, 'r') as f:
                    return f.read()
        elif ext in ['.json']:
            import json
            with open(filepath, 'r') as f:
                return json.load(f)
        elif ext in ['.yaml', '.yml']:
            import yaml
            with open(filepath, 'r') as f:
                return yaml.safe_load(f)
        elif ext == '.npy':
            import numpy as np
            return np.load(filepath)
        elif ext == '.npz':
            import numpy as np
            return np.load(filepath)
        elif ext == '.pkl':
            import pickle
            with open(filepath, 'rb') as f:
                return pickle.load(f)
        elif ext in ['.h5', '.hdf5']:
            import h5py
            return h5py.File(filepath, 'r')
        else:
            # Fallback to raw text
            with open(filepath, 'r') as f:
                return f.read()
    
    def _validate_loaded_data(self, loaded_data):
        """Validate the loaded data and check for common issues."""
        try:
            validation_result = {
                'valid': True,
                'warnings': [],
                'errors': [],
                'file_summaries': {}
            }
            
            for filename, file_data in loaded_data.items():
                if not file_data.get('loaded_successfully'):
                    validation_result['errors'].append(f"{filename}: Failed to load")
                    validation_result['valid'] = False
                    continue
                
                data = file_data['data']
                summary = self._get_data_summary(data, filename)
                validation_result['file_summaries'][filename] = summary
                
                # Add any warnings from the summary
                if 'warnings' in summary:
                    validation_result['warnings'].extend(summary['warnings'])
            
            # Notify validation callback
            if self.on_data_validated:
                self.on_data_validated(validation_result)
                
        except Exception as e:
            print(f"Error validating data: {e}")
            if self.debug:
                traceback.print_exc()
    
    def _get_data_summary(self, data, filename):
        """Get a summary of the loaded data."""
        try:
            summary = {'filename': filename}
            
            if hasattr(data, 'shape'):  # NumPy array or similar
                summary['type'] = 'array'
                summary['shape'] = data.shape
                summary['dtype'] = str(data.dtype)
            elif hasattr(data, 'columns'):  # Pandas DataFrame
                summary['type'] = 'dataframe'
                summary['shape'] = data.shape
                summary['columns'] = list(data.columns)
            elif isinstance(data, dict):
                summary['type'] = 'dictionary'
                summary['keys'] = list(data.keys())
            elif isinstance(data, list):
                summary['type'] = 'list'
                summary['length'] = len(data)
            elif isinstance(data, str):
                summary['type'] = 'text'
                summary['length'] = len(data)
            else:
                summary['type'] = type(data).__name__
            
            return summary
            
        except Exception as e:
            return {'filename': filename, 'error': str(e)}
    
    def get_loaded_data(self):
        """Get the currently loaded data."""
        return self.loaded_data
    
    def get_selected_files(self):
        """Get the list of currently selected files."""
        return self.selected_data_files.copy()
    
    def clear_loaded_data(self):
        """Clear all loaded data."""
        self.loaded_data = None
        self.validation_text.value = "No data loaded"
