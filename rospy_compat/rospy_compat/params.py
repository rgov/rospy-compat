"""
Parameter management for compatibility with rospy.
"""

import threading
from rclpy.parameter import Parameter


# Global parameter cache for faster access
_params = {}
_param_lock = threading.Lock()


def _insert_nested_value(dict_obj, key_path, value):
    """
    Insert a value into a nested dictionary using dot-separated key path.

    Args:
        dict_obj (dict): Dictionary to insert into (modified in-place)
        key_path (str): Dot-separated path (e.g., 'controller.P')
        value: Value to insert

    Example:
        d = {}
        _insert_nested_value(d, 'P', 1.0)
        # d = {'P': 1.0}

        d = {}
        _insert_nested_value(d, 'controller.P', 2.0)
        # d = {'controller': {'P': 2.0}}
    """
    keys = key_path.split('.')
    current = dict_obj

    # Navigate/create nested dicts for all keys except the last
    for key in keys[:-1]:
        if key not in current:
            current[key] = {}
        elif not isinstance(current[key], dict):
            # Conflict: key exists as non-dict value
            # Keep the existing value, skip insertion
            return
        current = current[key]

    # Set the final value
    final_key = keys[-1]
    current[final_key] = value


def _construct_namespace_dict(node, prefix, depth=0):
    """
    Construct a nested dictionary from parameters matching a prefix.

    This function enables ROS1-style parameter namespaces in ROS2.
    For example, if ROS2 has parameters 'gains.P', 'gains.I', 'gains.D',
    calling this with prefix='gains' will return {'P': 1.0, 'I': 2.0, 'D': 3.0}.

    Args:
        node: ROS2 node instance
        prefix (str): Parameter prefix (e.g., 'gains')
        depth (int): Search depth (currently unused with get_parameters_by_prefix)

    Returns:
        dict or None: Nested dictionary of parameters, or None if no matches
    """
    try:
        # Use built-in get_parameters_by_prefix which handles all the complexity
        params_dict = node.get_parameters_by_prefix(prefix)

        if not params_dict:
            # No parameters found with this prefix
            return None

        # Build nested dictionary from the flat dictionary returned
        namespace_dict = {}
        for relative_name, param in params_dict.items():
            _insert_nested_value(namespace_dict, relative_name, param.value)

        return namespace_dict if namespace_dict else None

    except Exception as e:
        node.get_logger().warning(
            f"Error constructing namespace dict for '{prefix}': {e}"
        )
        return None


def get_param(param_name, default=None):
    """
    Get a parameter value from the parameter server.
    In ROS2, parameters are node-scoped, not global.

    If the exact parameter doesn't exist, this function will attempt to
    construct a namespace dictionary from parameters with matching prefix.
    For example, if 'gains.P', 'gains.I', 'gains.D' exist, get_param('gains')
    returns {'P': 1.0, 'I': 2.0, 'D': 3.0}.

    Args:
        param_name (str): Parameter name. If starts with '~', it's relative to node namespace.
        default: Default value if parameter doesn't exist

    Returns:
        The parameter value, or nested dict for namespaces, or default if not found
    """
    from .node import _get_node

    node = _get_node()

    # Handle private namespace (~param)
    if param_name.startswith('~'):
        param_name = param_name[1:]

    # Remove leading slash if present
    if param_name.startswith('/'):
        param_name = param_name[1:]

    # Check cache first
    with _param_lock:
        if param_name in _params:
            return _params[param_name]

    # Try to get direct parameter from node (STEP 1: Direct lookup)
    try:
        if node.has_parameter(param_name):
            # Get parameter value
            param = node.get_parameter(param_name)
            value = param.value

            # Cache the value
            with _param_lock:
                _params[param_name] = value

            return value
    except Exception as e:
        node.get_logger().warning(f"Error getting direct parameter '{param_name}': {e}")

    # STEP 2: Parameter doesn't exist directly, try namespace construction
    namespace_dict = _construct_namespace_dict(node, param_name, depth=0)

    if namespace_dict is not None:
        # Cache the constructed dictionary
        with _param_lock:
            _params[param_name] = namespace_dict

        return namespace_dict

    # STEP 3: No parameter or namespace found - handle default
    if default is not None:
        # Declare parameter with default value
        try:
            node.declare_parameter(param_name, default)
            with _param_lock:
                _params[param_name] = default
        except Exception:
            # Declaration failed (e.g., can't declare namespace as param)
            # Just return default without declaring
            pass

        return default

    # Nothing found, return None
    return None


def set_param(param_name, value):
    """
    Set a parameter value.

    Args:
        param_name (str): Parameter name
        value: Parameter value
    """
    from .node import _get_node

    node = _get_node()

    # Handle private namespace (~param)
    if param_name.startswith('~'):
        param_name = param_name[1:]

    # Remove leading slash if present
    if param_name.startswith('/'):
        param_name = param_name[1:]

    try:
        # Check if parameter exists
        if not node.has_parameter(param_name):
            # Declare parameter first
            node.declare_parameter(param_name, value)
        else:
            # Set the parameter
            node.set_parameters([Parameter(param_name, value=value)])

        # Update cache
        with _param_lock:
            _params[param_name] = value

    except Exception as e:
        node.get_logger().error(f"Error setting parameter '{param_name}': {e}")
        raise


def has_param(param_name):
    """
    Check if a parameter exists.

    Args:
        param_name (str): Parameter name

    Returns:
        bool: True if parameter exists, False otherwise
    """
    from .node import _get_node

    node = _get_node()

    # Handle private namespace (~param)
    if param_name.startswith('~'):
        param_name = param_name[1:]

    # Remove leading slash if present
    if param_name.startswith('/'):
        param_name = param_name[1:]

    return node.has_parameter(param_name)


def delete_param(param_name):
    """
    Delete a parameter.
    Note: ROS2 doesn't support deleting parameters after they're declared.
    This is a no-op for compatibility.

    Args:
        param_name (str): Parameter name
    """
    from .node import _get_node

    node = _get_node()

    # Handle private namespace (~param)
    if param_name.startswith('~'):
        param_name = param_name[1:]

    # Remove leading slash if present
    if param_name.startswith('/'):
        param_name = param_name[1:]

    # Remove from cache
    with _param_lock:
        _params.pop(param_name, None)

    node.get_logger().warning(
        f"delete_param('{param_name}'): ROS2 does not support deleting parameters. "
        "Parameter will remain declared."
    )


def search_param(param_name):
    """
    Search for a parameter in the parameter server.
    Note: ROS2 doesn't have hierarchical parameter search like ROS1.
    This returns the parameter name if it exists, None otherwise.

    Args:
        param_name (str): Parameter name

    Returns:
        str or None: Parameter name if found, None otherwise
    """
    if has_param(param_name):
        return param_name
    return None


def get_param_names():
    """
    Get list of all parameter names.

    Returns:
        list: List of parameter names
    """
    from .node import _get_node

    node = _get_node()

    # Get all declared parameters
    param_names = [param.name for param in node.get_parameters(
        [descriptor.name for descriptor in node.describe_parameters(
            node.list_parameters([], 0).names
        )]
    )]

    return param_names


def get_name():
    """
    Get the name of the current node.

    Returns:
        str: Node name
    """
    from .node import _get_node

    node = _get_node()
    return node.get_name()


def get_namespace():
    """
    Get the namespace of the current node.

    Returns:
        str: Node namespace
    """
    from .node import _get_node

    node = _get_node()
    return node.get_namespace()


def get_caller_id():
    """
    Get the caller ID (node name) for ROS communications.
    This is an alias for get_name() for compatibility.

    Returns:
        str: Caller ID (node name)
    """
    return get_name()


def resolve_name(name, caller_id=None):
    """
    Resolve a ROS name to its global form.

    Args:
        name (str): Name to resolve (can have ~, relative, or global prefix)
        caller_id (str): Caller ID (unused in this implementation)

    Returns:
        str: Resolved name

    Note:
        ROS2 has fundamentally different name resolution than ROS1.
        This is a best-effort compatibility shim that may not match ROS1 exactly.
    """
    from .node import _get_node

    node = _get_node()

    # Simple implementation - just handle basic cases
    if name.startswith('/'):
        # Already global
        return name
    elif name.startswith('~'):
        # Private namespace
        node_name = node.get_name()
        return f"/{node_name}/{name[1:]}"
    else:
        # Relative namespace
        namespace = node.get_namespace()
        if namespace == '/':
            return f"/{name}"
        else:
            # Remove trailing slash from namespace if present
            namespace = namespace.rstrip('/')
            return f"{namespace}/{name}"
