"""
rospy_compat: ROS1 rospy compatibility layer for ROS2

This package provides a compatibility shim that allows ROS1 Python nodes to run
in ROS2 with minimal changes. Simply change your imports from:
    import rospy
to:
    import rospy_compat as rospy

The package wraps rclpy to provide a rospy-compatible API.
"""

# Version information
__version__ = '1.0.0'

# Install message import hook for ROS1 compatibility
# This must be done early, before any message imports
try:
    from .message_hooks import install_message_hooks
    install_message_hooks()
except ImportError as e:
    import sys
    sys.stderr.write(f"Warning: Could not install message import hooks: {e}\n")

# Node lifecycle
from .node import (
    init_node,
    spin,
    is_shutdown,
    signal_shutdown,
    on_shutdown,
    get_time,
    get_rostime,
    get_node_uri,
    myargv,
)

# Publishers and Subscribers
from .publishers import Publisher
from .subscribers import Subscriber, AnyMsg

# Services
from .services import Service, ServiceProxy, ServiceException, wait_for_service

# Time
from .time import Time, Duration

# Rate and timers
from .rate import Rate, Timer, sleep

# Parameters
from .params import (
    get_param,
    set_param,
    has_param,
    delete_param,
    search_param,
    get_param_names,
    get_name,
    get_namespace,
    get_caller_id,
    resolve_name,
)

# Logging
from .logging import (
    logdebug,
    loginfo,
    logwarn,
    logerr,
    logfatal,
    logdebug_throttle,
    loginfo_throttle,
    logwarn_throttle,
    logerr_throttle,
    logfatal_throttle,
)

# Exceptions
from .exceptions import (
    ROSException,
    ROSInterruptException,
    ROSInitException,
    ROSSerializationException,
)

# Core module (for rospy.core.add_preshutdown_hook access)
from . import core

# Actionlib module (import as submodule for compatibility)
# Users can do: from rospy_compat import actionlib
# or: import rospy_compat.actionlib
try:
    from . import actionlib
except ImportError as e:
    # If actionlib import fails, provide a helpful error message
    import sys
    sys.stderr.write(f"Warning: actionlib module could not be imported: {e}\n")
    actionlib = None


# Define __all__ for cleaner imports
__all__ = [
    # Version
    '__version__',

    # Node lifecycle
    'init_node',
    'spin',
    'is_shutdown',
    'signal_shutdown',
    'on_shutdown',
    'get_time',
    'get_rostime',
    'get_node_uri',
    'myargv',

    # Publishers and Subscribers
    'Publisher',
    'Subscriber',
    'AnyMsg',

    # Services
    'Service',
    'ServiceProxy',
    'ServiceException',
    'wait_for_service',

    # Time
    'Time',
    'Duration',

    # Rate and timers
    'Rate',
    'Timer',
    'sleep',

    # Parameters
    'get_param',
    'set_param',
    'has_param',
    'delete_param',
    'search_param',
    'get_param_names',
    'get_name',
    'get_namespace',
    'get_caller_id',
    'resolve_name',

    # Logging
    'logdebug',
    'loginfo',
    'logwarn',
    'logerr',
    'logfatal',
    'logdebug_throttle',
    'loginfo_throttle',
    'logwarn_throttle',
    'logerr_throttle',
    'logfatal_throttle',

    # Exceptions
    'ROSException',
    'ROSInterruptException',
    'ROSInitException',
    'ROSSerializationException',

    # Modules
    'core',
    'actionlib',
]


# Monkey-patch builtin_interfaces.msg.Time to add ROS1-compatible methods
# This allows header.stamp.to_sec() to work
try:
    from builtin_interfaces.msg import Time as BuiltinTime

    def _time_to_sec(self):
        """Convert Time message to seconds (ROS1 compatibility)"""
        return self.sec + (self.nanosec / 1e9)

    BuiltinTime.to_sec = _time_to_sec
except ImportError:
    pass  # builtin_interfaces not available

# Monkey-patch std_msgs.msg.Header to add ROS1-compatible seq field
# In ROS2, the seq field was removed from Header
try:
    from std_msgs.msg import Header as StdHeader

    # Add a seq property that returns 0 for compatibility
    # Note: ROS2 removed seq field. This is a compatibility shim that returns 0.
    # Real sequence tracking would require wrapping all publishers.
    if not hasattr(StdHeader, 'seq'):
        StdHeader.seq = property(lambda self: 0)
except ImportError:
    pass  # std_msgs not available


# Package information message (only shown in debug mode)
def _debug_info():
    """Print package information for debugging"""
    import sys
    if '--rospy-compat-debug' in sys.argv:
        print(f"rospy_compat v{__version__} loaded successfully")
        print("This is a ROS1 rospy compatibility layer for ROS2")


_debug_info()
