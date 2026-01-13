# Parameter management for rospy compatibility.

from rclpy.parameter import Parameter

from .exceptions import ROSInitException
from .impl.node import _get_node, _is_node_initialized
from .logging import logwarn_once

_deleted_params = set()  # Mask set for simulating parameter deletion
_unspecified = object()  # Sentinel for unspecified default


def _normalize(param_name):
    if param_name.startswith('~'):
        if not _is_node_initialized():
            raise ROSInitException(
                'Cannot resolve private param before init_node()'
            )
        node = _get_node()
        # ~foo -> /namespace/nodename/foo (like ROS1)
        ns = node.get_namespace().rstrip('/')
        if ns == '/' or ns == '':
            param_name = f'/{node.get_name()}/{param_name[1:]}'
        else:
            param_name = f'{ns}/{node.get_name()}/{param_name[1:]}'

    if param_name.startswith('/'):
        param_name = param_name[1:]

    # ROS1 uses / for param hierarchy, ROS2 uses dots internally
    # Convert remaining slashes to dots for ROS2 compatibility
    param_name = param_name.replace('/', '.')
    return param_name


def _insert_nested_value(d, key_path, value):
    keys = key_path.split('.')
    for key in keys[:-1]:
        if key not in d:
            d[key] = {}
        elif not isinstance(d[key], dict):
            return
        d = d[key]
    d[keys[-1]] = value


def _construct_namespace_dict(node, prefix):
    params = node.get_parameters_by_prefix(prefix)
    if not params:
        return None
    result = {}
    for rel_name, param in params.items():
        _insert_nested_value(result, rel_name, param.value)
    return result or None


def get_param(param_name, default=_unspecified):
    node = _get_node()
    name = _normalize(param_name)

    # Check delete mask
    if name in _deleted_params:
        if default is not _unspecified:
            return default
        raise KeyError(param_name)

    # Try direct lookup
    if node.has_parameter(name):
        return node.get_parameter(name).value

    # Try namespace construction
    ns_dict = _construct_namespace_dict(node, name)
    if ns_dict is not None:
        return ns_dict

    # Not found - return default or raise KeyError like ROS1
    if default is not _unspecified:
        return default
    raise KeyError(param_name)


def _flatten_dict(prefix, d, result):
    for key, value in d.items():
        full_key = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            _flatten_dict(full_key, value, result)
        else:
            result[full_key] = value


def set_param(param_name, value):
    node = _get_node()
    name = _normalize(param_name)

    # Clear delete mask on set
    _deleted_params.discard(name)

    # Handle dict values by flattening
    if isinstance(value, dict):
        flat_params = {}
        _flatten_dict(name, value, flat_params)
        for flat_name, flat_value in flat_params.items():
            if not node.has_parameter(flat_name):
                node.declare_parameter(flat_name, flat_value)
            else:
                node.set_parameters([Parameter(flat_name, value=flat_value)])
        return

    if not node.has_parameter(name):
        node.declare_parameter(name, value)
    else:
        node.set_parameters([Parameter(name, value=value)])


def has_param(param_name):
    name = _normalize(param_name)

    # Check delete mask
    if name in _deleted_params:
        return False

    return _get_node().has_parameter(name)


def delete_param(param_name):
    if not has_param(param_name):
        raise KeyError(param_name)
    name = _normalize(param_name)
    _deleted_params.add(name)
    logwarn_once(
        f'delete_param({param_name!r}): ROS2 cannot delete parameters. '
        f'Parameter is masked until re-set.'
    )


def search_param(param_name):
    # If absolute, just check directly
    if param_name.startswith('/'):
        if has_param(param_name):
            return param_name
        return None

    # Get node's namespace for upward search
    node = _get_node()
    ns = node.get_namespace().rstrip('/')

    # Build search path from current namespace upward
    parts = [p for p in ns.split('/') if p]

    # Search from most specific to least specific
    while True:
        candidate = '/' + '/'.join(parts + [param_name]) if parts else '/' + param_name
        if has_param(candidate):
            return candidate
        if not parts:
            break
        parts.pop()

    return None


def get_param_names():
    node = _get_node()
    names = node.list_parameters([], 0).names
    # Filter deleted and convert dots to slashes for ROS1 compatibility
    return ['/' + n.replace('.', '/') for n in names if n not in _deleted_params]
