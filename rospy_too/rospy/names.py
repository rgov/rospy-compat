# Name resolution functions for rospy compatibility.

from .impl.node import _get_node


def get_name():
    node = _get_node()
    ns = node.get_namespace().rstrip('/')
    name = node.get_name()
    if ns == '/' or ns == '':
        return '/' + name
    return ns + '/' + name


def get_namespace():
    return _get_node().get_namespace()


def get_caller_id():
    return get_name()


def _resolve_with_caller_id(name, caller_id):
    # Manual resolution when caller_id is provided (rclpy has no equivalent)
    if name.startswith('/'):
        return name
    elif name.startswith('~'):
        return caller_id.rstrip('/') + '/' + name[1:]
    else:
        caller_ns = '/'.join(caller_id.rstrip('/').split('/')[:-1]) or '/'
        return f'/{name}' if caller_ns == '/' else f'{caller_ns}/{name}'


def _resolve_manual(name, node):
    # Fallback for older rclpy without resolve_topic_name
    if name.startswith('/'):
        return name
    ns = node.get_namespace().rstrip('/')
    if name.startswith('~'):
        base = f'/{node.get_name()}' if ns in ('/', '') else f'{ns}/{node.get_name()}'
        return f'{base}/{name[1:]}'
    return f'/{name}' if ns in ('/', '') else f'{ns}/{name}'


def resolve_name(name, caller_id=None):
    node = _get_node()

    if caller_id is not None:
        return _resolve_with_caller_id(name, caller_id)

    # Use rclpy's resolution which handles ~, relative, absolute + remaps
    try:
        return node.resolve_topic_name(name, only_expand=False)
    except (AttributeError, Exception):
        return _resolve_manual(name, node)
