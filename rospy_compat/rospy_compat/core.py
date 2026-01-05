"""
Core module for rospy_compat.
Provides access to core functions, particularly preshutdown hooks.

This module exists for compatibility with rospy.core.add_preshutdown_hook().
"""

from .node import add_preshutdown_hook, on_shutdown

# Export for rospy.core.add_preshutdown_hook compatibility
__all__ = ['add_preshutdown_hook', 'on_shutdown']
