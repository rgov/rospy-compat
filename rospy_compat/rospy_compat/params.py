"""
Parameter management for compatibility with rospy.
"""

import threading
from rclpy.parameter import Parameter


# Global parameter cache for faster access
_params = {}
_param_lock = threading.Lock()


def get_param(param_name, default=None):
    """
    Get a parameter value from the parameter server.
    In ROS2, parameters are node-scoped, not global.

    Args:
        param_name (str): Parameter name. If starts with '~', it's relative to node namespace.
        default: Default value if parameter doesn't exist

    Returns:
        The parameter value, or default if not found
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

    # Try to get from node
    try:
        # Check if parameter exists
        if not node.has_parameter(param_name):
            # Declare parameter with default value if provided
            if default is not None:
                node.declare_parameter(param_name, default)
                with _param_lock:
                    _params[param_name] = default
                return default
            else:
                # No default, just return None
                return None

        # Get parameter value
        param = node.get_parameter(param_name)
        value = param.value

        # Cache the value
        with _param_lock:
            _params[param_name] = value

        return value

    except Exception as e:
        node.get_logger().warning(f"Error getting parameter '{param_name}': {e}")
        return default


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
