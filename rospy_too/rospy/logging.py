# Logging functions for rospy compatibility.

import hashlib
import threading
import time as python_time

from .impl.node import _get_node

_identical_state = {}
_lock = threading.Lock()


def _get_logger():
    return _get_node().get_logger()


def _format_msg(msg, args):
    if args:
        return str(msg) % args
    return str(msg)


def logdebug(msg, *args):
    _get_logger().debug(_format_msg(msg, args))


def loginfo(msg, *args):
    _get_logger().info(_format_msg(msg, args))


def logwarn(msg, *args):
    _get_logger().warning(_format_msg(msg, args))


def logerr(msg, *args):
    _get_logger().error(_format_msg(msg, args))


def logfatal(msg, *args):
    _get_logger().fatal(_format_msg(msg, args))


# Aliases
logout = loginfo
logerror = logerr


def logdebug_throttle(period, msg, *args):
    _get_logger().debug(_format_msg(msg, args), throttle_duration_sec=period)


def loginfo_throttle(period, msg, *args):
    _get_logger().info(_format_msg(msg, args), throttle_duration_sec=period)


def logwarn_throttle(period, msg, *args):
    _get_logger().warning(_format_msg(msg, args), throttle_duration_sec=period)


def logerr_throttle(period, msg, *args):
    _get_logger().error(_format_msg(msg, args), throttle_duration_sec=period)


def logfatal_throttle(period, msg, *args):
    _get_logger().fatal(_format_msg(msg, args), throttle_duration_sec=period)


def logdebug_once(msg, *args):
    _get_logger().debug(_format_msg(msg, args), once=True)


def loginfo_once(msg, *args):
    _get_logger().info(_format_msg(msg, args), once=True)


def logwarn_once(msg, *args):
    _get_logger().warning(_format_msg(msg, args), once=True)


def logerr_once(msg, *args):
    _get_logger().error(_format_msg(msg, args), once=True)


def logfatal_once(msg, *args):
    _get_logger().fatal(_format_msg(msg, args), once=True)


def _should_log_identical(level_name, period, formatted):
    msg_hash = hashlib.md5(formatted.encode()).hexdigest()
    now = python_time.time()
    with _lock:
        logger = _get_logger()
        state_key = f'{logger.name}:{level_name}'
        last_hash, last_time = _identical_state.get(state_key, (None, 0))
        if msg_hash != last_hash or now - last_time >= period:
            _identical_state[state_key] = (msg_hash, now)
            return True
    return False


def logdebug_throttle_identical(period, msg, *args):
    formatted = _format_msg(msg, args)
    if _should_log_identical('debug', period, formatted):
        _get_logger().debug(formatted)


def loginfo_throttle_identical(period, msg, *args):
    formatted = _format_msg(msg, args)
    if _should_log_identical('info', period, formatted):
        _get_logger().info(formatted)


def logwarn_throttle_identical(period, msg, *args):
    formatted = _format_msg(msg, args)
    if _should_log_identical('warning', period, formatted):
        _get_logger().warning(formatted)


def logerr_throttle_identical(period, msg, *args):
    formatted = _format_msg(msg, args)
    if _should_log_identical('error', period, formatted):
        _get_logger().error(formatted)


def logfatal_throttle_identical(period, msg, *args):
    formatted = _format_msg(msg, args)
    if _should_log_identical('fatal', period, formatted):
        _get_logger().fatal(formatted)
