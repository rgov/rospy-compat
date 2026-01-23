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


def _normalize_name(name, warn=True):
    # Normalize a ROS name for ROS2 compatibility.
    # - Collapses double slashes (foo//bar -> foo/bar)
    # - Removes trailing slashes (topic/ -> topic)
    # - Emulates rospy quirk: ~/foo -> /foo (with warning)
    # - Converts ROS1-style private name (~foo) to ROS2-style (~/foo)
    # Collapse double slashes first (before pattern matching)
    while '//' in name:
        name = name.replace('//', '/')

    # Remove trailing slash (but preserve bare / and ~/)
    if len(name) > 1 and name.endswith('/') and name != '~/':
        name = name.rstrip('/')

    # Emulate rospy quirk: ~/foo resolves to /foo due to ns_join bug
    if name.startswith('~/') and name != '~/':
        if warn:
            from .logging import logwarn_once

            logwarn_once(
                "Emulating rospy quirk: '%s' resolves to '%s' due to ~/ prefix",
                name,
                name[1:],
            )
        return name[1:]  # ~/foo -> /foo

    # Convert ROS1-style private name (~, ~foo) to ROS2-style (~/, ~/foo)
    if name.startswith('~') and not name.startswith('~/'):
        name = '~/' + name[1:]

    return name


def _resolve_with_caller_id(name, caller_id):
    # Manual resolution when caller_id is provided (rclpy has no equivalent).
    name = _normalize_name(name)
    caller_id = caller_id.rstrip('/')

    if name.startswith('/'):
        return name
    if name.startswith('~/'):
        # Private name - after normalization, ~foo became ~/foo, ~ became ~/
        suffix = name[2:]
        return caller_id if not suffix else f'{caller_id}/{suffix}'
    # Relative name
    caller_ns = '/'.join(caller_id.split('/')[:-1]) or '/'
    return f'/{name}' if caller_ns == '/' else f'{caller_ns}/{name}'


def _resolve_manual(name, node):
    # Fallback for older rclpy without resolve_topic_name.
    # Input is already normalized by caller
    if name.startswith('/'):
        return name

    ns = node.get_namespace().rstrip('/')
    node_base = (
        f'/{node.get_name()}' if ns in ('/', '') else f'{ns}/{node.get_name()}'
    )

    if name.startswith('~/'):
        # Private name - after normalization, ~ became ~/, ~foo became ~/foo
        suffix = name[2:]
        return node_base if not suffix else f'{node_base}/{suffix}'

    # Relative name
    return f'/{name}' if ns in ('/', '') else f'{ns}/{name}'


def resolve_name(name, caller_id=None):
    # Resolve a ROS name to its fully qualified form.
    node = _get_node()

    if caller_id is not None:
        return _resolve_with_caller_id(name, caller_id)

    # Normalize ROS1 names for ROS2 compatibility
    name = _normalize_name(name)

    # Let rclpy handle resolution (supports ~/, relative, absolute + remaps)
    try:
        return node.resolve_topic_name(name, only_expand=False)
    except (AttributeError, Exception):
        return _resolve_manual(name, node)
