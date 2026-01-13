# Node lifecycle management and global node singleton.

import os
import random
import signal
import sys
import threading
import time as python_time

import rclpy
import rclpy.executors
from rclpy.logging import LoggingSeverity

from ..exceptions import ROSInitException
from ..time import Time
from .pending import fire_node_init_callbacks

# Mapping from rospy log levels to rclpy LoggingSeverity
LOG_LEVEL_MAP = {
    1: LoggingSeverity.DEBUG,   # rospy.DEBUG
    2: LoggingSeverity.INFO,    # rospy.INFO
    4: LoggingSeverity.WARN,    # rospy.WARN
    8: LoggingSeverity.ERROR,   # rospy.ERROR
    16: LoggingSeverity.FATAL,  # rospy.FATAL
}

_node = None
_node_lock = threading.RLock()
_executor = None
_executor_thread = None
_shutdown_hooks = []      # on_shutdown: zero-arg callbacks
_preshutdown_hooks = []   # add_preshutdown_hook: one-arg (reason)
_is_shutdown = False
_shutdown_requested = False


def _get_node():
    with _node_lock:
        if _node is None:
            raise ROSInitException(
                'rospy.init_node() or rclpy.create_node() must be called before using this API'
            )
        return _node


def _is_node_initialized():
    with _node_lock:
        return _node is not None


def _setup_executor():
    global _executor, _executor_thread
    if _executor is None:
        _executor = rclpy.executors.SingleThreadedExecutor()
        _executor.add_node(_node)
        _executor_thread = threading.Thread(target=_spin_executor, daemon=True)
        _executor_thread.start()


def _spin_executor():
    while rclpy.ok() and not _is_shutdown:
        try:
            _executor.spin_once(timeout_sec=0.1)
        except Exception:
            # Don't let callback exceptions kill the executor thread.
            # Exceptions are already stored in Futures before rclpy re-raises.
            pass


def _realize_pending():
    fire_node_init_callbacks(_node)


def init_node(
    name,
    argv=None,
    anonymous=False,
    log_level=None,
    disable_signals=False,
    **kwargs,
):
    global _node, _is_shutdown, _shutdown_requested

    with _node_lock:
        if _node is not None:
            raise RuntimeError('rospy.init_node() has already been called')

        # Reset shutdown flags (allows re-init after signal_shutdown)
        _is_shutdown = False
        _shutdown_requested = False

        if not rclpy.ok():
            rclpy.init(args=argv)

        # ROS1 anonymous naming: name_pid_random (skip if launch provides name)
        node_name = name
        if anonymous and not has_node_remap(argv):
            node_name = (
                f'{name}_{os.getpid()}_{random.randint(0, 0xFFFFFFFF):08x}'
            )

        # Let rclpy handle namespace from --ros-args -r __ns:=
        _node = rclpy.create_node(
            node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            **kwargs,
        )

        if log_level is not None and log_level in LOG_LEVEL_MAP:
            _node.get_logger().set_level(LOG_LEVEL_MAP[log_level])

        if not disable_signals:
            _register_signal_handlers()

        _setup_executor()
        _realize_pending()


def _shutdown_node_internal():
    # Clean up node, executor, and executor thread (before rclpy.shutdown())
    global _node, _executor, _executor_thread

    # Shutdown executor first (stops accepting new work)
    if _executor is not None:
        try:
            _executor.shutdown()
        except Exception:
            pass
        _executor = None

    # Join executor thread (wait for it to finish)
    if _executor_thread is not None and _executor_thread.is_alive():
        _executor_thread.join(timeout=2.0)
    _executor_thread = None

    # Destroy node last
    if _node is not None:
        try:
            _node.destroy_node()
        except Exception:
            pass
        _node = None


def _register_signal_handlers():
    def handler(signum, frame):
        signal_shutdown(f'Signal {signum} received')
        # Do NOT call sys.exit() - let the main loop (spin) detect shutdown
        # and exit gracefully. Calling sys.exit() immediately would kill the
        # executor thread before it can clean up.

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def spin():
    while not _is_shutdown and rclpy.ok():
        python_time.sleep(0.1)


def is_shutdown():
    return _is_shutdown or not rclpy.ok()


def signal_shutdown(reason):
    # Shutdown order: set flag, run hooks, cleanup node/executor, shutdown rclpy
    global _is_shutdown, _shutdown_requested

    if _shutdown_requested:
        return
    _shutdown_requested = True
    _is_shutdown = True

    # Run preshutdown hooks first (with reason arg)
    for hook in _preshutdown_hooks:
        try:
            hook(reason)
        except Exception:
            pass

    # Run shutdown hooks (no args, ROS1 compatible)
    for hook in _shutdown_hooks:
        try:
            hook()
        except Exception:
            pass

    # Clean up node/executor BEFORE rclpy.shutdown()
    _shutdown_node_internal()

    # Finally shutdown rclpy
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        # Ignore errors during shutdown (Rolling can raise during cleanup)
        pass


def on_shutdown(h):
    _shutdown_hooks.append(h)


def add_preshutdown_hook(h):
    _preshutdown_hooks.append(h)


def get_time():
    node = _get_node()
    return node.get_clock().now().nanoseconds / 1e9


def get_rostime():
    return Time.now()


def get_node_uri():
    node = _get_node()
    return f'http://localhost:0/{node.get_name()}'


def has_node_remap(argv):
    # Check if --ros-args contains -r __node:= remap
    if not argv:
        return False
    in_ros_args = False
    expect_remap = False
    for arg in argv:
        if arg == '--ros-args':
            in_ros_args = True
        elif arg == '--':
            in_ros_args = False
        elif in_ros_args:
            if expect_remap:
                if arg.startswith('__node:='):
                    return True
                expect_remap = False
            elif arg in ('-r', '--remap'):
                expect_remap = True
    return False


def strip_ros2_args(argv):
    # Strip --ros-args ... [--] block from argv
    result = []
    i = 0
    in_ros_args = False
    consume_next = {
        '-r', '-p', '--remap', '--param', '--params-file',
        '--log-level', '--log-config-file', '--enclave',
    }
    while i < len(argv):
        arg = argv[i]
        if arg == '--ros-args':
            in_ros_args = True
        elif in_ros_args and arg == '--':
            in_ros_args = False
        elif in_ros_args:
            if arg in consume_next and i + 1 < len(argv):
                i += 1
        else:
            result.append(arg)
        i += 1
    return result


def myargv(argv=None):
    # Return argv with ROS arguments removed
    if argv is None:
        argv = sys.argv
    try:
        return rclpy.utilities.remove_ros_args(args=argv)
    except AttributeError:
        return strip_ros2_args(argv)
