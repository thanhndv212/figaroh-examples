# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np


def reorder_inertial_parameters(pinocchio_params):
    """Reorder inertial parameters from Pinocchio format to desired format.
    
    Args:
        pinocchio_params: Parameters in Pinocchio order [m, mx, my, mz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
        
    Returns:
        list: Parameters in desired order [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
    """
    if len(pinocchio_params) != 10:
        raise ValueError(f"Expected 10 inertial parameters, got {len(pinocchio_params)}")
    
    # Mapping from Pinocchio indices to desired indices
    reordered = np.zeros_like(pinocchio_params)
    reordered[0] = pinocchio_params[4]  # Ixx
    reordered[1] = pinocchio_params[5]  # Ixy  
    reordered[2] = pinocchio_params[7]  # Ixz
    reordered[3] = pinocchio_params[6]  # Iyy
    reordered[4] = pinocchio_params[8]  # Iyz
    reordered[5] = pinocchio_params[9]  # Izz
    reordered[6] = pinocchio_params[1]  # mx
    reordered[7] = pinocchio_params[2]  # my
    reordered[8] = pinocchio_params[3]  # mz
    reordered[9] = pinocchio_params[0]  # m
    
    return reordered.tolist()


def add_standard_additional_parameters(phi, params, param_config, model):
    """Add standard additional parameters (actuator inertia, friction, offsets).
    
    Args:
        phi: Current parameter values list
        params: Current parameter names list  
        param_config: Configuration dictionary
        model: Robot model
        
    Returns:
        tuple: Updated (phi, params) lists
    """
    num_joints = len(model.inertias) - 1  # Exclude world link
    
    # Standard additional parameters configuration
    additional_params = [
        {
            'name': 'Ia',
            'enabled_key': 'has_actuator_inertia',
            'values_key': 'Ia',
            'default': 0.0,
            'description': 'actuator inertia'
        },
        {
            'name': 'fv', 
            'enabled_key': 'has_friction',
            'values_key': 'fv',
            'default': 0.0,
            'description': 'viscous friction'
        },
        {
            'name': 'fs',
            'enabled_key': 'has_friction', 
            'values_key': 'fs',
            'default': 0.0,
            'description': 'static friction'
        },
        {
            'name': 'off',
            'enabled_key': 'has_joint_offset',
            'values_key': 'off', 
            'default': 0.0,
            'description': 'joint offset'
        }
    ]
    
    for link_idx in range(1, num_joints + 1):
        for param_def in additional_params:
            param_name = f"{param_def['name']}{link_idx}"
            params.append(param_name)
            
            # Get parameter value
            if param_config.get(param_def['enabled_key'], False):
                try:
                    values_list = param_config.get(param_def['values_key'], [])
                    if len(values_list) >= link_idx:
                        value = values_list[link_idx - 1]
                    else:
                        value = param_def['default']
                        print(f"Warning: Missing {param_def['description']} for joint {link_idx}, using default: {value}")
                except (KeyError, IndexError, TypeError) as e:
                    value = param_def['default']
                    print(f"Warning: Error getting {param_def['description']} for joint {link_idx}: {e}, using default: {value}")
            else:
                value = param_def['default']
            
            phi.append(value)
    
    return phi, params


def add_custom_parameters(phi, params, custom_params, model):
    """Add custom user-defined parameters.
    
    Args:
        phi: Current parameter values list
        params: Current parameter names list
        custom_params: Custom parameter definitions
        model: Robot model
        
    Returns:
        tuple: Updated (phi, params) lists
    """
    num_joints = len(model.inertias) - 1  # Exclude world link
    
    for param_name, param_def in custom_params.items():
        if not isinstance(param_def, dict):
            print(f"Warning: Invalid custom parameter definition for '{param_name}', skipping")
            continue
            
        values = param_def.get('values', [])
        per_joint = param_def.get('per_joint', True)
        default_value = param_def.get('default', 0.0)
        
        if per_joint:
            # Add parameter for each joint
            for link_idx in range(1, num_joints + 1):
                param_full_name = f"{param_name}{link_idx}"
                params.append(param_full_name)
                
                try:
                    if len(values) >= link_idx:
                        value = values[link_idx - 1]
                    else:
                        value = default_value
                        if values:  # Only warn if values were provided but insufficient
                            print(f"Warning: Missing value for custom parameter '{param_name}' joint {link_idx}, using default: {value}")
                except (IndexError, TypeError):
                    value = default_value
                    print(f"Warning: Error accessing custom parameter '{param_name}' for joint {link_idx}, using default: {value}")
                
                phi.append(value)
        else:
            # Global parameter (not per joint)
            params.append(param_name)
            try:
                value = values[0] if values else default_value
            except (IndexError, TypeError):
                value = default_value
                print(f"Warning: Error accessing global custom parameter '{param_name}', using default: {value}")
            
            phi.append(value)
    
    return phi, params


def get_standard_parameters(model, param=None, include_additional=True, custom_params=None):
    """Get standard inertial parameters from robot model with extensible parameter support.
    
    Args:
        model: Robot model (Pinocchio model)
        param (dict, optional): Dictionary of parameter settings for additional parameters.
            Expected keys:
            - has_actuator_inertia (bool): Include actuator inertia parameters
            - has_friction (bool): Include friction parameters  
            - has_joint_offset (bool): Include joint offset parameters
            - Ia (list): Actuator inertia values
            - fv (list): Viscous friction coefficients
            - fs (list): Static friction coefficients  
            - off (list): Joint offset values
        include_additional (bool): Whether to include additional parameters beyond inertial
        custom_params (dict, optional): Custom parameter definitions
            Format: {param_name: {values: list, per_joint: bool, default: float}}
            
    Returns:
        dict: Parameter names mapped to their values
        
    Examples:
        # Basic usage - only inertial parameters
        params = get_standard_parameters(robot.model)
        
        # Include standard additional parameters
        param_config = {
            'has_actuator_inertia': True,
            'has_friction': True, 
            'Ia': [0.1, 0.2, 0.3],
            'fv': [0.01, 0.02, 0.03],
            'fs': [0.001, 0.002, 0.003]
        }
        params = get_standard_parameters(robot.model, param_config)
        
        # Add custom parameters
        custom = {
            'gear_ratio': {'values': [100, 50, 25], 'per_joint': True, 'default': 1.0},
            'temperature': {'values': [20.0], 'per_joint': False, 'default': 25.0}
        }
        params = get_standard_parameters(robot.model, param_config, custom_params=custom)
    """
    if param is None:
        param = {}
    
    if custom_params is None:
        custom_params = {}
    
    phi = []
    params = []
    
    # Standard inertial parameter names in desired order
    inertial_params = [
        "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz",
        "mx", "my", "mz", "m"
    ]
    
    # Extract and rearrange inertial parameters for each link
    for link_idx in range(1, len(model.inertias)):
        # Get dynamic parameters from Pinocchio (in Pinocchio order)
        pinocchio_params = model.inertias[link_idx].toDynamicParameters()
        
        # Rearrange from Pinocchio order [m, mx, my, mz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
        # to desired order [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
        reordered_params = reorder_inertial_parameters(pinocchio_params)
        
        # Add parameter names and values
        for param_name in inertial_params:
            params.append(f"{param_name}{link_idx}")
        phi.extend(reordered_params)
    
    # Add additional standard parameters if requested
    if include_additional:
        phi, params = add_standard_additional_parameters(phi, params, param, model)
    
    # Add custom parameters if provided
    if custom_params:
        phi, params = add_custom_parameters(phi, params, custom_params, model)
    
    return dict(zip(params, phi))


def get_parameter_info():
    """Get information about available parameter types.
    
    Returns:
        dict: Information about standard and custom parameter types
    """
    return {
        'inertial_parameters': [
            "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz",
            "mx", "my", "mz", "m"
        ],
        'standard_additional': {
            'actuator_inertia': {
                'name': 'Ia',
                'enabled_key': 'has_actuator_inertia',
                'values_key': 'Ia',
                'description': 'Actuator/rotor inertia'
            },
            'viscous_friction': {
                'name': 'fv', 
                'enabled_key': 'has_friction',
                'values_key': 'fv',
                'description': 'Viscous friction coefficient'
            },
            'static_friction': {
                'name': 'fs',
                'enabled_key': 'has_friction',
                'values_key': 'fs', 
                'description': 'Static friction coefficient'
            },
            'joint_offset': {
                'name': 'off',
                'enabled_key': 'has_joint_offset',
                'values_key': 'off',
                'description': 'Joint position offset'
            }
        },
        'custom_parameters_format': {
            'parameter_name': {
                'values': 'list of values',
                'per_joint': 'boolean - if True, creates param for each joint',
                'default': 'default value if not enough values provided'
            }
        }
    }



def _check_package_available(package_name):
    """Check if a package is available for import."""
    try:
        __import__(package_name)
        return True
    except ImportError:
        return False

