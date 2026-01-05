"""
Service classes for compatibility with rospy.
"""

import time as python_time
import rclpy


def Service(name, service_type, handler, **kwargs):
    """
    Create a service server.
    Compatible with rospy.Service.

    Args:
        name (str): Service name
        service_type (type): Service type class
        handler (callable): Service handler function.
                           In ROS1, handler receives (request) and returns response.
                           This wrapper adapts to ROS2's (request, response) signature.
        **kwargs: Additional arguments (most are ROS1-specific and ignored)

    Returns:
        rclpy.service.Service: A service server object
    """
    from .node import _get_node

    node = _get_node()

    # Wrap the handler to convert ROS1 signature to ROS2 signature
    # ROS1: handler(request) -> response
    # ROS2: handler(request, response) -> response
    def wrapped_handler(request, response):
        try:
            # Call ROS1-style handler
            result = handler(request)

            # Copy fields from result to response
            if result is not None:
                for field in result.__slots__:
                    setattr(response, field, getattr(result, field))

            return response
        except Exception as e:
            node.get_logger().error(f"Exception in service handler for '{name}': {e}")
            # Return response (may be partially filled or default)
            return response

    # Create and return the service
    service = node.create_service(service_type, name, wrapped_handler)

    return service


class ServiceProxy:
    """
    Service client that provides synchronous service calls.
    Compatible with rospy.ServiceProxy.
    """

    def __init__(self, name, service_type, persistent=False, headers=None):
        """
        Create a service client.

        Args:
            name (str): Service name
            service_type (type): Service type class
            persistent (bool): Whether to use a persistent connection (not supported in ROS2)
            headers (dict): Connection headers (not supported in ROS2)
        """
        from .node import _get_node

        self.node = _get_node()
        self.service_name = name
        self.service_type = service_type

        # Create ROS2 service client
        self.client = self.node.create_client(service_type, name)

        if persistent:
            self.node.get_logger().warning(
                f"ServiceProxy for '{name}': persistent connections not supported in ROS2"
            )

    def wait_for_service(self, timeout=None):
        """
        Wait for the service to become available.

        Args:
            timeout (float): Maximum time to wait in seconds. None means wait forever.

        Returns:
            bool: True if service is available (or became available), False if timeout

        Raises:
            ROSException: If there's an error waiting for the service
        """
        from .exceptions import ROSException
        from .node import is_shutdown

        if timeout is None:
            # Wait forever
            while not self.client.service_is_ready() and not is_shutdown():
                python_time.sleep(0.1)
            return not is_shutdown()
        else:
            # Wait with timeout
            end_time = python_time.time() + timeout

            while not self.client.service_is_ready() and not is_shutdown():
                if python_time.time() >= end_time:
                    return False
                python_time.sleep(0.1)

            return self.client.service_is_ready()

    def call(self, *args, **kwargs):
        """
        Call the service synchronously.
        This is the same as __call__ for compatibility.

        Args:
            *args: Positional arguments for service request
            **kwargs: Keyword arguments for service request

        Returns:
            Service response object

        Raises:
            Exception: If service call fails
        """
        return self(*args, **kwargs)

    def __call__(self, *args, **kwargs):
        """
        Call the service synchronously.

        Args:
            *args: Either a single Request object, or arguments to construct one
            **kwargs: Keyword arguments for constructing the request

        Returns:
            Service response object

        Raises:
            Exception: If service call fails or times out
        """
        from .exceptions import ROSException
        from .node import is_shutdown

        if is_shutdown():
            raise ROSException("Cannot call service, node is shutdown")

        # Construct request
        if len(args) == 1 and isinstance(args[0], self.service_type.Request):
            request = args[0]
        else:
            # Construct request from arguments
            try:
                request = self.service_type.Request(*args, **kwargs)
            except Exception as e:
                raise ROSException(f"Failed to construct service request: {e}")

        # Call service asynchronously
        future = self.client.call_async(request)

        # Wait for result synchronously
        # Use rclpy's spin_until_future_complete to process callbacks while waiting
        timeout_sec = 10.0  # Default timeout

        try:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        except Exception as e:
            raise ROSException(f"Error while waiting for service response: {e}")

        # Check if we got a result
        if future.result() is None:
            exception = future.exception()
            if exception:
                raise ROSException(f"Service call failed: {exception}")
            else:
                raise ROSException(f"Service call timed out after {timeout_sec} seconds")

        return future.result()

    def close(self):
        """
        Close the service client.
        """
        # In ROS2, we would need to destroy the client, but there's no direct method
        # The client will be cleaned up when the node is destroyed
        pass


# Alias for compatibility
ServiceException = Exception  # ROS1 had rospy.ServiceException


def wait_for_service(service_name, timeout=None):
    """
    Wait for a service to become available.
    Module-level function for ROS1 compatibility.

    Args:
        service_name (str): Name of the service
        timeout (float): Maximum time to wait in seconds. None means wait forever.

    Raises:
        ROSException: If timeout occurs or node is shut down
    """
    from .node import _get_node, is_shutdown
    from .exceptions import ROSException

    node = _get_node()

    # We need to create a temporary client to check if service is available
    # This is a bit inefficient but matches ROS1 behavior
    # In real usage, users should create a ServiceProxy and use its wait_for_service()

    # For now, we'll just wait by polling ros2 service list
    # A proper implementation would create a temporary client
    import rclpy

    start_time = python_time.time()

    while not is_shutdown():
        # Check if service is available
        service_names_and_types = node.get_service_names_and_types()
        service_names = [name for name, _ in service_names_and_types]

        if service_name in service_names:
            return True

        # Check timeout
        if timeout is not None:
            elapsed = python_time.time() - start_time
            if elapsed >= timeout:
                raise ROSException(f"Timeout waiting for service '{service_name}'")

        python_time.sleep(0.1)

    raise ROSException(f"Node shutdown while waiting for service '{service_name}'")
