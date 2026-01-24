# Logging functions for rospy compatibility.

import hashlib
import sys
import threading
import time as python_time
import traceback

from .impl.node import _get_node, _is_node_initialized

_throttle_state = {}  # caller_id -> last_log_time
_identical_state = {}  # state_key -> (msg_hash, last_log_time)
_once_logged = set()
_lock = threading.Lock()


def _get_logger():
    if not _is_node_initialized():
        return None
    return _get_node().get_logger()


def _caller_id():
    frame = sys._getframe(1)
    while frame and frame.f_code.co_filename == __file__:
        frame = frame.f_back
    if frame is None:
        frame = sys._getframe(1)
    return (frame.f_code.co_filename, frame.f_lineno, frame.f_code.co_name)


def _format_msg(msg, args):
    return str(msg) % args if args else str(msg)


def _format_exc_info(msg, exc_info):
    if not exc_info:
        return msg
    if exc_info is True:
        exc_info = sys.exc_info()
    if exc_info[0] is not None:
        msg += '\n' + ''.join(traceback.format_exception(*exc_info))
    return msg


def _format(msg, args, kwargs):
    return _format_exc_info(_format_msg(msg, args), kwargs.pop('exc_info', None))


# Level-specific log functions - each has unique call site for rclpy tracking
def _log_debug(formatted):
    logger = _get_logger()
    if logger:
        logger.debug(formatted)
    else:
        print(f'[DEBUG]: {formatted}', file=sys.stderr)


def _log_info(formatted):
    logger = _get_logger()
    if logger:
        logger.info(formatted)
    else:
        print(f'[INFO]: {formatted}', file=sys.stderr)


def _log_warning(formatted):
    logger = _get_logger()
    if logger:
        logger.warning(formatted)
    else:
        print(f'[WARN]: {formatted}', file=sys.stderr)


def _log_error(formatted):
    logger = _get_logger()
    if logger:
        logger.error(formatted)
    else:
        print(f'[ERROR]: {formatted}', file=sys.stderr)


def _log_fatal(formatted):
    logger = _get_logger()
    if logger:
        logger.fatal(formatted)
    else:
        print(f'[FATAL]: {formatted}', file=sys.stderr)


_LOG = {
    'debug': _log_debug,
    'info': _log_info,
    'warning': _log_warning,
    'error': _log_error,
    'fatal': _log_fatal,
}


def _log_once(level, formatted):
    """Log only once per call site."""
    caller = _caller_id()
    if caller in _once_logged:
        return
    _once_logged.add(caller)
    _LOG[level](formatted)


def _log_throttle(level, period, formatted):
    """Log at most once per period per call site."""
    caller = _caller_id()
    now = python_time.time()
    with _lock:
        last_time = _throttle_state.get(caller, 0)
        if now - last_time < period:
            return
        _throttle_state[caller] = now
    _LOG[level](formatted)


def _log_identical(level, period, formatted):
    """Log only when message changes or period elapsed."""
    logger = _get_logger()
    logger_name = logger.name if logger else '_preinit'
    msg_hash = hashlib.md5(formatted.encode()).hexdigest()
    now = python_time.time()
    with _lock:
        state_key = f'{logger_name}:{level}'
        last_hash, last_time = _identical_state.get(state_key, (None, 0))
        if msg_hash == last_hash and now - last_time < period:
            return
        _identical_state[state_key] = (msg_hash, now)
    _LOG[level](formatted)


# Base logging functions
def logdebug(msg, *args, **kwargs):
    _log_debug(_format(msg, args, kwargs))


def loginfo(msg, *args, **kwargs):
    _log_info(_format(msg, args, kwargs))


def logwarn(msg, *args, **kwargs):
    _log_warning(_format(msg, args, kwargs))


def logerr(msg, *args, **kwargs):
    _log_error(_format(msg, args, kwargs))


def logfatal(msg, *args, **kwargs):
    _log_fatal(_format(msg, args, kwargs))


# Aliases
logout = loginfo
logerror = logerr


# Throttle functions
def logdebug_throttle(period, msg, *args, **kwargs):
    _log_throttle('debug', period, _format(msg, args, kwargs))


def loginfo_throttle(period, msg, *args, **kwargs):
    _log_throttle('info', period, _format(msg, args, kwargs))


def logwarn_throttle(period, msg, *args, **kwargs):
    _log_throttle('warning', period, _format(msg, args, kwargs))


def logerr_throttle(period, msg, *args, **kwargs):
    _log_throttle('error', period, _format(msg, args, kwargs))


def logfatal_throttle(period, msg, *args, **kwargs):
    _log_throttle('fatal', period, _format(msg, args, kwargs))


# Once functions
def logdebug_once(msg, *args, **kwargs):
    _log_once('debug', _format(msg, args, kwargs))


def loginfo_once(msg, *args, **kwargs):
    _log_once('info', _format(msg, args, kwargs))


def logwarn_once(msg, *args, **kwargs):
    _log_once('warning', _format(msg, args, kwargs))


def logerr_once(msg, *args, **kwargs):
    _log_once('error', _format(msg, args, kwargs))


def logfatal_once(msg, *args, **kwargs):
    _log_once('fatal', _format(msg, args, kwargs))


# Throttle identical functions
def logdebug_throttle_identical(period, msg, *args, **kwargs):
    _log_identical('debug', period, _format(msg, args, kwargs))


def loginfo_throttle_identical(period, msg, *args, **kwargs):
    _log_identical('info', period, _format(msg, args, kwargs))


def logwarn_throttle_identical(period, msg, *args, **kwargs):
    _log_identical('warning', period, _format(msg, args, kwargs))


def logerr_throttle_identical(period, msg, *args, **kwargs):
    _log_identical('error', period, _format(msg, args, kwargs))


def logfatal_throttle_identical(period, msg, *args, **kwargs):
    _log_identical('fatal', period, _format(msg, args, kwargs))
