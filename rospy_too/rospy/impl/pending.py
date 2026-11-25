_pending = set()


def register_node_init_callback(obj):
    from .node import _get_node, _is_node_initialized, _node_lock

    with _node_lock:
        if _is_node_initialized():
            obj._after_node_init(_get_node())
        else:
            _pending.add(obj)


def cancel_node_init_callback(obj):
    from .node import _node_lock

    with _node_lock:
        _pending.discard(obj)


def fire_node_init_callbacks(node):
    while _pending:
        _pending.pop()._after_node_init(node)
