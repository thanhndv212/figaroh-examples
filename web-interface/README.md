# Web Interface for Figaroh Examples

This project provides a modern web interface for robotics calibration and identification using the Figaroh framework. The interface allows users to load robot models, configure parameters, execute tasks, and visualize results in an interactive 3D environment using Viser.

## Project Structure

The project is organized into several directories and files:

- **web-interface/**: The main package containing all the components of the web interface.
  - **__init__.py**: Marks the directory as a Python package.
  - **main.py**: Entry point for the application, initializing the web interface and starting the server.
  - **app.py**: Contains the main application logic using Viser server setup.
  - **core/**: Contains core functionalities of the application.
    - **__init__.py**: Marks the core directory as a Python package.
    - **interface.py**: Defines the main interface class for handling user interactions.
    - **robot_manager.py**: Enhanced robot manager with multiple loader support.
    - **task_manager.py**: Handles task execution and management with progress tracking.
    - **example_loader.py**: Provides functionality to load example configurations.
    - **session_manager.py**: Manages user sessions and state.
  - **components/**: Contains UI components of the web interface using Viser.
    - **__init__.py**: Marks the components directory as a Python package.
    - **config_panel.py**: Defines the configuration panel component.
    - **robot_panel.py**: Defines the robot panel component with advanced loader support.
    - **task_panel.py**: Defines the task panel component with comprehensive task management.
    - **visualization_panel.py**: Defines the visualization panel component.
    - **results_panel.py**: Defines the results panel component.
    - **data_panel.py**: Defines the data panel component.
  - **utils/**: Contains utility functions for various tasks.
    - **__init__.py**: Marks the utils directory as a Python package.
    - **file_utils.py**: Provides utility functions for file handling and path discovery.

## Technology Stack

- **Viser**: 3D web interface framework for interactive robotics visualization
- **Figaroh**: Robot loading and manipulation framework
- **Python**: Backend logic and component management
- **Threading**: Asynchronous task execution and progress tracking

## Features

- **Interactive 3D Visualization**: Real-time robot model visualization using Viser
- **Multiple Robot Loaders**: Support for figaroh, robot_description, and yourdfpy loaders
- **Comprehensive Task Management**: Execute calibration, identification, optimization, and trajectory generation tasks
- **Progress Tracking**: Real-time task progress monitoring with threaded execution
- **Path Discovery**: Intelligent URDF and model path discovery with source-based filtering
- **Session Management**: Maintain state across robot loading and task execution
- **Advanced Configuration**: Task-specific parameter configuration for different robot operations

## Installation

To set up the project, clone the repository and install the required dependencies:

```bash
git clone <repository-url>
cd figaroh-examples/web-interface
pip install -r requirements.txt
```

## Usage

To run the web interface, execute the following command:

```bash
python main.py
```

Open your web browser and navigate to `http://localhost:8080` to access the interactive interface.

### Command Line Options

```bash
python main.py --help
```

Available options:
- `--port`: Port to run the server on (default: 8080)
- `--host`: Host to bind to (default: localhost)
- `--examples-path`: Path to examples directory
- `--models-path`: Path to models directory
- `--config`: Configuration file path
- `--debug`: Enable debug mode
- `--auto-reload`: Auto-reload on file changes

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the Apache License, Version 2.0. See the LICENSE file for more details.