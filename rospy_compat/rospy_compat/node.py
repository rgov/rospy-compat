"""
Node lifecycle management and global node instance for compatibility with rospy.
"""

import threading
import uuid
import signal
import sys
import time as python_time

import rclpy
import rclpy.executors

from .exceptions import ROSInitException, ROSInterruptException


# Global state for the singleton node and executor
_node = None
_node_lock = threading.RLock()
_executor = None
_executor_thread = None
_shutdown_hooks = []
_is_shutdown = False
_shutdown_requested = False


def _get_node():
    """
    Get the global node instance, creating it lazily if needed.

    Returns:
        rclpy.node.Node: The global node instance

    Raises:
        ROSInitException: If node creation fails
    """
    global _node, _executor, _executor_thread

    with _node_lock:
        if _node is None:
            # Initialize rclpy if not already done
            if not rclpy.ok():
                try:
                    rclpy.init()
                except Exception as e:
                    raise ROSInitException(f"Failed to initialize rclpy: {e}")

            # Create a generic node name if init_node hasn't been called
            node_name = f'rospy_compat_node_{uuid.uuid4().hex[:8]}'

            try:
                _node = rclpy.create_node(
                    node_name,
                    allow_undeclared_parameters=True,
                    automatically_declare_parameters_from_overrides=True
                )
            except Exception as e:
                raise ROSInitException(f"Failed to create node: {e}")

            # Start background executor thread
            _setup_executor()

        return _node


def _setup_executor():
    """
    Set up the executor in a background thread.
    This allows callbacks (timers, subscriptions, services) to be processed automatically.
    """
    global _executor, _executor_thread

    if _executor is None:
        _executor = rclpy.executors.SingleThreadedExecutor()
        _executor.add_node(_node)

        # Start executor in background daemon thread
        _executor_thread = threading.Thread(target=_spin_executor, daemon=True)
        _executor_thread.start()


def _spin_executor():
    """
    Internal function to spin the executor in the background thread.
    """
    global _is_shutdown

    try:
        while rclpy.ok() and not _is_shutdown:
            _executor.spin_once(timeout_sec=0.1)
    except Exception as e:
        # Log error but don't crash
        if _node:
            _node.get_logger().error(f"Exception in executor thread: {e}")


def _shutdown_node_internal():
    """
    Internal function to clean up the node without shutting down rclpy.
    Used when replacing a lazily-created node with a properly-named one.
    """
    global _node, _executor, _executor_thread, _is_shutdown

    # Note: This function assumes _node_lock is already held by the caller

    # Temporarily set shutdown flag to stop executor
    old_is_shutdown = _is_shutdown
    _is_shutdown = True

    if _executor is not None:
        # Remove node from executor
        if _node is not None:
            _executor.remove_node(_node)

        # Shutdown executor (this will stop the spin loop)
        _executor.shutdown()
        _executor = None

    # Wait for executor thread to finish
    if _executor_thread is not None and _executor_thread.is_alive():
        _executor_thread.join(timeout=1.0)
        _executor_thread = None

    # Destroy the node
    if _node is not None:
        try:
            _node.destroy_node()
        except Exception:
            pass  # Ignore errors during cleanup
        _node = None

    # Reset shutdown flag for the new node
    _is_shutdown = False


def init_node(name, argv=None, anonymous=False, log_level=None, disable_signals=False, **kwargs):
    """
    Initialize the rospy_compat node.

    Args:
        name (str): Name of the node
        argv (list): Command line arguments (currently ignored)
        anonymous (bool): If True, append a unique ID to the node name
        log_level (int): Logging level (currently ignored, use ROS2 logging config)
        disable_signals (bool): If True, do not register signal handlers
        **kwargs: Additional arguments passed to rclpy.create_node()

    Raises:
        ROSInitException: If initialization fails
        Exception: If node is already initialized with a non-lazy name
    """
    global _node, _executor, _executor_thread, _shutdown_requested

    with _node_lock:
        if _node is not None:
            # Check if this is a lazily-created node (generic name pattern)
            # ROS1 pattern: Publishers created before init_node()
            existing_name = _node.get_name()
            if existing_name.startswith('rospy_compat_node_'):
                # Lazily-created node already exists
                # We can't destroy and recreate it because existing Publisher/Subscriber
                # handles would become invalid. Instead, just log a warning and continue.
                _node.get_logger().warning(
                    f"init_node('{name}') called after lazy node creation. "
                    f"Using existing node '{existing_name}'. "
                    f"This is a ROS1 compatibility pattern where Publishers are created before init_node()."
                )
                # Continue with existing node (skip recreation)
                return
            else:
                # Already initialized with explicit name - this is an error
                raise Exception("Node already initialized. Cannot call init_node() twice.")

        # Initialize rclpy if not already done
        if not rclpy.ok():
            try:
                rclpy.init(args=argv)
            except Exception as e:
                raise ROSInitException(f"Failed to initialize rclpy: {e}")

        # Make node name anonymous if requested
        node_name = name
        if anonymous:
            node_name = f"{name}_{uuid.uuid4().hex[:8]}"

        # Create the node
        try:
            _node = rclpy.create_node(
                node_name,
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True,
                **kwargs
            )
        except Exception as e:
            raise ROSInitException(f"Failed to create node '{node_name}': {e}")

        # Setup signal handlers unless disabled
        if not disable_signals:
            _register_signal_handlers()

        # Start background executor thread
        _setup_executor()

        _node.get_logger().info(f"Node '{node_name}' initialized")


def _register_signal_handlers():
    """
    Register signal handlers for graceful shutdown.
    """
    def signal_handler(signum, frame):
        signal_shutdown(f"Signal {signum} received")
        sys.exit(0)

    try:
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    except Exception as e:
        # Signal handling might not work in all contexts (e.g., threads)
        if _node:
            _node.get_logger().warning(f"Could not register signal handlers: {e}")


def spin():
    """
    Block and process callbacks until shutdown is requested.
    Compatible with rospy.spin().
    """
    global _is_shutdown

    try:
        while not _is_shutdown and rclpy.ok():
            python_time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def is_shutdown():
    """
    Check if shutdown has been requested.

    Returns:
        bool: True if shutdown has been requested, False otherwise
    """
    global _is_shutdown
    return _is_shutdown or not rclpy.ok()


def signal_shutdown(reason):
    """
    Request shutdown of the node.

    Args:
        reason (str): Reason for shutdown
    """
    global _is_shutdown, _shutdown_requested, _shutdown_hooks

    if _shutdown_requested:
        return  # Already shutting down

    _shutdown_requested = True
    _is_shutdown = True

    # Call all registered shutdown hooks
    for hook in _shutdown_hooks:
        try:
            hook(reason)
        except Exception as e:
            if _node:
                _node.get_logger().error(f"Exception in shutdown hook: {e}")

    # Log shutdown
    if _node:
        _node.get_logger().info(f"Shutdown requested: {reason}")

    # Shutdown rclpy
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception as e:
            if _node:
                _node.get_logger().error(f"Error during rclpy shutdown: {e}")


def on_shutdown(h):
    """
    Register a function to be called on shutdown.
    This is an alias for add_preshutdown_hook for compatibility.

    Args:
        h (callable): Function to call on shutdown (receives reason string)
    """
    add_preshutdown_hook(h)


def add_preshutdown_hook(h):
    """
    Register a function to be called before shutdown.

    Args:
        h (callable): Function to call on shutdown (receives reason string)
    """
    global _shutdown_hooks
    _shutdown_hooks.append(h)


def get_time():
    """
    Get the current time as a float (seconds since epoch).

    Returns:
        float: Current time in seconds
    """
    node = _get_node()
    return node.get_clock().now().nanoseconds / 1e9


def get_rostime():
    """
    Get the current ROS time.

    Returns:
        Time: Current time as a Time object
    """
    from .time import Time
    return Time.now()


def get_node_uri():
    """
    Get the URI of the node.
    Note: ROS2 doesn't have the same URI concept as ROS1.

    Returns:
        str: A placeholder URI string
    """
    node = _get_node()
    return f"http://localhost:0/{node.get_name()}"


def myargv(argv=None):
    """
    Get command line arguments with ROS-specific arguments removed.
    Note: ROS2 handles this differently, so this is a simplified version.

    Args:
        argv (list): Argument list (defaults to sys.argv)

    Returns:
        list: Filtered argument list
    """
    if argv is None:
        argv = sys.argv

    # In ROS2, we don't need to filter as extensively as ROS1
    # Just return the arguments
    return argv
