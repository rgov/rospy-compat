import weakref

_pending = weakref.WeakSet()


def register_node_init_callback(obj):
    from .node import _get_node, _is_node_initialized

    if _is_node_initialized():
        obj._after_node_init(_get_node())
    else:
        _pending.add(obj)


def fire_node_init_callbacks(node):
    for obj in list(_pending):
        obj._after_node_init(node)
    _pending.clear()
