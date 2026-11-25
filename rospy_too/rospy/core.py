# Core module for rospy.core compatibility.
# Provides shutdown hooks for modules that import rospy.core directly.

from .impl.node import (
    _is_node_initialized,
    add_preshutdown_hook,
    add_shutdown_hook,
    is_shutdown,
    is_shutdown_requested,
    on_shutdown,
    signal_shutdown,
)


def is_initialized():
    return _is_node_initialized()


# Export all for 'from rospy.core import *'
__all__ = [
    'is_initialized',
    'is_shutdown',
    'is_shutdown_requested',
    'signal_shutdown',
    'on_shutdown',
    'add_shutdown_hook',
    'add_preshutdown_hook',
]
