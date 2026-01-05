"""
Logging functions for compatibility with rospy.
"""

import threading
import time as python_time


# State for throttled logging
_log_throttle_state = {}
_log_lock = threading.Lock()


def logdebug(msg, *args):
    """
    Log a debug message.

    Args:
        msg (str): Message to log (can include format string)
        *args: Arguments for format string
    """
    from .node import _get_node
    node = _get_node()
    if args:
        msg = msg % args
    node.get_logger().debug(str(msg))


def loginfo(msg, *args):
    """
    Log an info message.

    Args:
        msg (str): Message to log (can include format string)
        *args: Arguments for format string
    """
    from .node import _get_node
    node = _get_node()
    if args:
        msg = msg % args
    node.get_logger().info(str(msg))


def logwarn(msg, *args):
    """
    Log a warning message.

    Args:
        msg (str): Message to log (can include format string)
        *args: Arguments for format string
    """
    from .node import _get_node
    node = _get_node()
    if args:
        msg = msg % args
    node.get_logger().warning(str(msg))


def logerr(msg, *args):
    """
    Log an error message.

    Args:
        msg (str): Message to log (can include format string)
        *args: Arguments for format string
    """
    from .node import _get_node
    node = _get_node()
    if args:
        msg = msg % args
    node.get_logger().error(str(msg))


def logfatal(msg, *args):
    """
    Log a fatal message.

    Args:
        msg (str): Message to log (can include format string)
        *args: Arguments for format string
    """
    from .node import _get_node
    node = _get_node()
    if args:
        msg = msg % args
    node.get_logger().fatal(str(msg))


def logdebug_throttle(period, msg, *args):
    """
    Log a debug message, but throttle it to only log once per period.

    Args:
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    _log_throttled('debug', period, msg, *args)


def loginfo_throttle(period, msg, *args):
    """
    Log an info message, but throttle it to only log once per period.

    Args:
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    _log_throttled('info', period, msg, *args)


def logwarn_throttle(period, msg, *args):
    """
    Log a warning message, but throttle it to only log once per period.

    Args:
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    _log_throttled('warn', period, msg, *args)


def logerr_throttle(period, msg, *args):
    """
    Log an error message, but throttle it to only log once per period.

    Args:
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    _log_throttled('error', period, msg, *args)


def logfatal_throttle(period, msg, *args):
    """
    Log a fatal message, but throttle it to only log once per period.

    Args:
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    _log_throttled('fatal', period, msg, *args)


def _log_throttled(level, period, msg, *args):
    """
    Internal helper for throttled logging.

    Args:
        level (str): Log level ('debug', 'info', 'warn', 'error', 'fatal')
        period (float): Minimum time in seconds between logs
        msg (str): Message to log
        *args: Arguments for format string
    """
    # Create a unique key for this log message
    key = f"{level}_{msg}"

    current_time = python_time.time()

    with _log_lock:
        last_time = _log_throttle_state.get(key, 0)

        # Check if enough time has passed
        if current_time - last_time >= period:
            _log_throttle_state[key] = current_time

            # Log the message using the appropriate function
            if level == 'debug':
                logdebug(msg, *args)
            elif level == 'info':
                loginfo(msg, *args)
            elif level == 'warn':
                logwarn(msg, *args)
            elif level == 'error':
                logerr(msg, *args)
            elif level == 'fatal':
                logfatal(msg, *args)


# Aliases for compatibility
logdebug_once = lambda msg, *args: logdebug_throttle(float('inf'), msg, *args)
loginfo_once = lambda msg, *args: loginfo_throttle(float('inf'), msg, *args)
logwarn_once = lambda msg, *args: logwarn_throttle(float('inf'), msg, *args)
logerr_once = lambda msg, *args: logerr_throttle(float('inf'), msg, *args)
logfatal_once = lambda msg, *args: logfatal_throttle(float('inf'), msg, *args)
