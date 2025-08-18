"""Main entry point for the Figaroh Examples Web Interface."""

import argparse
import sys
import signal
from pathlib import Path

from app import FigarohWebApp

def signal_handler(sig, frame):
    """Handle shutdown signals gracefully."""
    print("\n🛑 Shutting down web interface...")
    sys.exit(0)

def main():
    """Main function to start the web interface."""
    parser = argparse.ArgumentParser(description="Figaroh Examples Web Interface")
    parser.add_argument("--port", type=int, default=8080, help="Port to run the server on")
    parser.add_argument("--host", type=str, default="localhost", help="Host to bind to")
    parser.add_argument("--examples-path", type=str, help="Path to examples directory")
    parser.add_argument("--models-path", type=str, help="Path to models directory")
    parser.add_argument("--config", type=str, help="Configuration file path")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--auto-reload", action="store_true", help="Auto-reload on file changes")
    parser.add_argument("--classic", action="store_true", help="Use classic interface (default: streamlined)")
    
    args = parser.parse_args()
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run the web application
    try:
        app = FigarohWebApp(
            port=args.port,
            host=args.host,
            examples_path=args.examples_path,
            models_path=args.models_path,
            config_path=args.config,
            debug=args.debug,
            auto_reload=args.auto_reload,
            use_streamlined=not args.classic
        )
        
        app.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error starting web interface: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()