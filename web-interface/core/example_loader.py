# filepath: /figaroh-examples/web-interface/core/example_loader.py

class ExampleLoader:
    def __init__(self, example_path: str):
        self.example_path = example_path
        self.examples = []

    def discover_examples(self):
        """Discover available examples."""
        try:
            example_dict = self.list_examples()
            self.examples = example_dict  # Store the dictionary
            print(f"Discovered {len(self.examples)} examples")
        except Exception as e:
            print(f"Error discovering examples: {e}")
            self.examples = {}

    def get_all_examples(self) -> list:
        """Get all available example names as a list."""
        if isinstance(self.examples, dict):
            return list(self.examples.keys())
        return []

    def get_robot_types(self) -> list:
        """Get available robot types from example directories."""
        return self.get_all_examples()  # Example names are robot types
    
    def get_example_path(self, example_name: str) -> str:
        """Get the absolute path for a specific example."""
        if isinstance(self.examples, dict) and example_name in self.examples:
            return self.examples[example_name]
        return ""

    def load_example(self, example_name: str) -> dict:
        """Load an example configuration from the specified directory."""
        import os
        import yaml
        
        # Get the absolute path for this example
        example_dir = self.get_example_path(example_name)
        if not example_dir:
            raise FileNotFoundError(f"Example '{example_name}' not found.")
        
        if not os.path.exists(example_dir):
            raise FileNotFoundError(f"Example directory {example_dir} not found.")
        
        # Look for common configuration files in the example directory
        config_files = ['config.yaml', 'calibration.yaml', 'configuration.yaml', 'setup.yaml']
        
        for config_file in config_files:
            config_path = os.path.join(example_dir, config_file)
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as file:
                        config = yaml.safe_load(file)
                        config['example_name'] = example_name
                        config['example_path'] = example_dir
                        return config
                except Exception as e:
                    print(f"Error loading config file {config_path}: {e}")
                    continue
        
        # If no YAML config found, create a basic configuration
        return {
            'example_name': example_name,
            'example_path': example_dir,
            'robot_type': example_name,
            'description': f'Example for {example_name} robot'
        }

    def list_examples(self) -> dict:
        """List all available examples as a dictionary with names and paths."""
        import os
        try:
            if not os.path.exists(self.example_path):
                print(f"Example path {self.example_path} does not exist")
                return {}
            
            # Get all directories in the examples path
            items = os.listdir(self.example_path)
            example_dict = {}
            
            for item in items:
                item_path = os.path.join(self.example_path, item)
                # Check if it's a directory and not a hidden directory
                if os.path.isdir(item_path) and not item.startswith('.') and not item.startswith('__'):
                    # Use absolute path for the value
                    example_dict[item] = os.path.abspath(item_path)
            
            print(f"Found example directories: {list(example_dict.keys())}")
            return example_dict
            
        except Exception as e:
            print(f"Error listing examples: {e}")
            return {}