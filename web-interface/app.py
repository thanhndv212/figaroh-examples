"""Main web application for Figaroh Examples using Viser."""

import viser
import time
import threading
from pathlib import Path
from typing import Optional, Dict, Any
import traceback

from core.interface import FigarohWebInterface
from core.streamlined_interface import StreamlinedFigarohInterface
from core.example_loader import ExampleLoader
from core.robot_manager import RobotManager
from core.task_manager import TaskManager
from core.session_manager import SessionManager
from utils.file_utils import find_examples_root, find_models_root

class FigarohWebApp:
    """Main application for the Figaroh Examples Web Interface."""
    
    def __init__(self, 
                 port: int = 8080,
                 host: str = "localhost",
                 examples_path: Optional[str] = None,
                 models_path: Optional[str] = None,
                 config_path: Optional[str] = None,
                 debug: bool = False,
                 auto_reload: bool = False,
                 use_streamlined: bool = True):
        """Initialize the web application."""
        self.port = port
        self.host = host
        self.debug = debug
        self.auto_reload = auto_reload
        self.use_streamlined = use_streamlined
        
        # Initialize paths
        self.examples_path = self._resolve_examples_path(examples_path)
        self.models_path = self._resolve_models_path(models_path)
        self.config_path = config_path
        
        # Initialize Viser server
        self.server = viser.ViserServer(port=port, verbose=debug)
        
        # Core managers
        self.example_loader = None
        self.robot_manager = None
        self.task_manager = None
        self.session_manager = None
        
        # Main interface
        self.interface = None
        
        # State
        self.running = False
        self.shutdown_event = threading.Event()
        
    def _resolve_examples_path(self, provided_path: Optional[str]) -> Path:
        """Resolve the examples directory path."""
        if provided_path:
            return Path(provided_path)
        
        try:
            return find_examples_root()
        except FileNotFoundError as e:
            print(f"Warning: {e}")
            # Use current directory as fallback
            return Path.cwd().parent / "examples"
    
    def _resolve_models_path(self, provided_path: Optional[str]) -> Path:
        """Resolve the models directory path."""
        if provided_path:
            return Path(provided_path)
        
        try:
            return find_models_root()
        except FileNotFoundError as e:
            print(f"Warning: {e}")
            # Use current directory as fallback
            return Path.cwd().parent / "models"
    
    def initialize(self):
        """Initialize all components."""
        try:
            print("🔧 Initializing components...")
            
            # Initialize managers
            self.example_loader = ExampleLoader(
                example_path=self.examples_path
            )
            
            self.robot_manager = RobotManager(
                models_path=self.models_path
            )
            
            self.task_manager = TaskManager()
            
            self.session_manager = SessionManager()
            
            # Initialize main interface
            if self.use_streamlined:
                self.interface = StreamlinedFigarohInterface(
                    server=self.server,
                    example_loader=self.example_loader,
                    robot_manager=self.robot_manager,
                    task_manager=self.task_manager,
                    session_manager=self.session_manager,
                    debug=self.debug
                )
                print("🎯 Using streamlined interface")
            else:
                self.interface = FigarohWebInterface(
                    server=self.server,
                    example_loader=self.example_loader,
                    robot_manager=self.robot_manager,
                    task_manager=self.task_manager,
                    session_manager=self.session_manager,
                    debug=self.debug
                )
                print("🔧 Using full interface")
            
            # Discover examples and models
            self.example_loader.discover_examples()
            
            # Update the interface with discovered examples
            if hasattr(self, 'interface') and self.interface:
                examples = self.example_loader.get_all_examples()
                
                # Handle different interface types
                if self.use_streamlined:
                    # Streamlined interface uses setup_panel
                    if hasattr(self.interface, 'setup_panel') and self.interface.setup_panel:
                        self.interface.setup_panel.update_examples(examples)
                        print(f"🔄 Updated streamlined interface with {len(examples)} examples")
                else:
                    # Classic interface uses config_panel and robot_panel
                    if hasattr(self.interface, 'config_panel'):
                        self.interface.config_panel.update_examples(examples)
                    if hasattr(self.interface, 'robot_panel'):
                        self.interface.robot_panel.update_robot_examples(examples)
                    print(f"🔄 Updated classic interface with {len(examples)} examples")
            
            print(f"✅ Found {len(self.example_loader.get_all_examples())} examples")
            print(f"✅ Found {len(self.example_loader.get_robot_types())} robot types")
            
        except Exception as e:
            print(f"❌ Error initializing components: {e}")
            if self.debug:
                traceback.print_exc()
            raise
    
    def run(self):
        """Run the web application."""
        try:
            # Initialize components
            self.initialize()
            
            print(f"🚀 Figaroh Examples Web Interface starting...")
            print(f"📱 Open http://{self.host}:{self.port} in your browser")
            print(f"📊 Ready to serve {len(self.example_loader.get_all_examples())} examples")
            print("Press Ctrl+C to stop")
            
            self.running = True
            
            # Main loop
            while self.running and not self.shutdown_event.is_set():
                time.sleep(1.0)
                
                # Auto-reload check (if enabled)
                if self.auto_reload:
                    self._check_for_changes()
                    
        except KeyboardInterrupt:
            print("\n🛑 Received shutdown signal")
        except Exception as e:
            print(f"❌ Runtime error: {e}")
            if self.debug:
                traceback.print_exc()
        finally:
            self.shutdown()
    
    def _check_for_changes(self):
        """Check for file changes and reload if necessary."""
        # This could be implemented to watch for changes in examples/configs
        # and trigger reloads automatically
        pass
    
    def shutdown(self):
        """Shutdown the application gracefully."""
        if not self.running:
            return
            
        print("🛑 Shutting down web interface...")
        self.running = False
        self.shutdown_event.set()
        
        # Cleanup components
        if self.task_manager:
            self.task_manager.shutdown()
            
        if self.robot_manager:
            self.robot_manager.cleanup()
            
        if self.session_manager:
            self.session_manager.cleanup()
            
        print("✅ Shutdown complete")