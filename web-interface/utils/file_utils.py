"""File utilities for the web interface."""

import os
from pathlib import Path
from typing import Optional, List, Dict, Any
import json
import yaml

def find_examples_root(start_path: Optional[Path] = None) -> Path:
    """Find the examples root directory."""
    if start_path is None:
        start_path = Path(__file__).parent.parent.parent
    
    # Look for examples directory
    search_paths = [
        start_path / "examples",
        start_path.parent / "examples",
        start_path / ".." / "examples",
    ]
    
    for path in search_paths:
        if path.exists() and path.is_dir():
            return path.resolve()
    
    raise FileNotFoundError("Could not find examples directory")

def find_models_root(start_path: Optional[Path] = None) -> Path:
    """Find the models root directory."""
    if start_path is None:
        start_path = Path(__file__).parent.parent.parent
    
    # Look for models directory
    search_paths = [
        start_path / "models",
        start_path.parent / "models",
        start_path / ".." / "models",
    ]
    
    for path in search_paths:
        if path.exists() and path.is_dir():
            return path.resolve()
    
    raise FileNotFoundError("Could not find models directory")

def load_config_file(config_path: Path) -> Dict[str, Any]:
    """Load configuration from JSON or YAML file."""
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        if config_path.suffix.lower() == '.json':
            return json.load(f)
        else:
            return yaml.safe_load(f)

def save_config_file(config: Dict[str, Any], config_path: Path):
    """Save configuration to JSON or YAML file."""
    config_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(config_path, 'w') as f:
        if config_path.suffix.lower() == '.json':
            json.dump(config, f, indent=2)
        else:
            yaml.dump(config, f, default_flow_style=False, indent=2)

def find_files_by_extension(directory: Path, extensions: List[str]) -> List[Path]:
    """Find files with specific extensions in directory."""
    files = []
    for ext in extensions:
        files.extend(directory.glob(f"*.{ext}"))
        files.extend(directory.glob(f"**/*.{ext}"))
    return files

def validate_urdf_file(urdf_path: Path) -> Dict[str, Any]:
    """Validate URDF file."""
    result = {
        'valid': True,
        'errors': [],
        'warnings': []
    }
    
    if not urdf_path.exists():
        result['valid'] = False
        result['errors'].append(f"URDF file not found: {urdf_path}")
        return result
    
    try:
        with open(urdf_path, 'r') as f:
            content = f.read()
            
        # Basic XML validation
        import xml.etree.ElementTree as ET
        ET.fromstring(content)
        
        # Check for required URDF elements
        if '<robot' not in content:
            result['warnings'].append("No <robot> tag found")
        if '<link' not in content:
            result['warnings'].append("No <link> tags found")
        if '<joint' not in content:
            result['warnings'].append("No <joint> tags found")
            
    except ET.ParseError as e:
        result['valid'] = False
        result['errors'].append(f"XML parse error: {e}")
    except Exception as e:
        result['valid'] = False
        result['errors'].append(f"Validation error: {e}")
    
    return result

def get_file_info(file_path: Path) -> Dict[str, Any]:
    """Get information about a file."""
    if not file_path.exists():
        return {'exists': False}
    
    stat = file_path.stat()
    return {
        'exists': True,
        'name': file_path.name,
        'size': stat.st_size,
        'modified': stat.st_mtime,
        'is_dir': file_path.is_dir(),
        'extension': file_path.suffix.lower()
    }