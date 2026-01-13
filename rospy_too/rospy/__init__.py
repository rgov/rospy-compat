# rospy compatibility layer for ROS 2
# Allows unmodified ROS 1 rospy code to run on ROS 2

__version__ = '1.0.0'

# Install message import hooks early
from .impl.hooks import install_message_hooks

install_message_hooks()

# Apply runtime patches (builtin_interfaces, exceptions, rclpy.create_node hook)
# ServiceException is imported from .services above
# Submodules for rospy.core access
from . import core

# Exceptions
from .exceptions import (
    ROSException,
    ROSInitException,
    ROSInterruptException,
    ROSSerializationException,
    ROSTimeMovedBackwardsException,
)
from .impl import patches  # noqa: F401

# Node lifecycle (re-exports for public API)
# ruff: noqa: E402, F401
from .impl.node import (
    get_node_uri,
    get_rostime,
    get_time,
    init_node,
    is_shutdown,
    myargv,
    on_shutdown,
    signal_shutdown,
    spin,
)

# Logging
from .logging import (
    logdebug,
    logdebug_once,
    logdebug_throttle,
    logdebug_throttle_identical,
    logerr,
    logerr_once,
    logerr_throttle,
    logerr_throttle_identical,
    logerror,
    logfatal,
    logfatal_once,
    logfatal_throttle,
    logfatal_throttle_identical,
    loginfo,
    loginfo_once,
    loginfo_throttle,
    loginfo_throttle_identical,
    logout,
    logwarn,
    logwarn_once,
    logwarn_throttle,
    logwarn_throttle_identical,
)

# Messages
from .msg import AnyMsg

# Names
from .names import get_caller_id, get_name, get_namespace, resolve_name

# Parameters
from .params import (
    delete_param,
    get_param,
    get_param_names,
    has_param,
    search_param,
    set_param,
)

# Rate and timers
from .rate import Rate, Timer, TimerEvent, sleep

# Services
from .services import Service, ServiceException, ServiceProxy, wait_for_service

# Time
from .time import Duration, Time

# Topics
from .topics import Publisher, SubscribeListener, Subscriber, wait_for_message

# Log level constants
DEBUG = 1
INFO = 2
WARN = 4
ERROR = 8
FATAL = 16
