# Service classes for rospy compatibility.

import time as python_time

from .exceptions import ROSException, ROSInterruptException, ServiceException
from .impl.node import _get_node, is_shutdown
from .impl.pending import register_node_init_callback
from .logging import logerr, logwarn_once

# Default timeout for service calls (seconds). ROS2 cannot propagate handler
# failures to clients, so we use a timeout to prevent infinite hangs.
DEFAULT_SERVICE_TIMEOUT = 10.0


def _resolve_service(node, name):
    # Resolve service name using rclpy's resolver with fallback
    try:
        return node.resolve_service_name(name, only_expand=False)
    except (AttributeError, Exception):
        from .names import resolve_name
        return resolve_name(name)


def _get_response_fields(response):
    # ROS2 messages use get_fields_and_field_types(), fallback to __slots__
    if hasattr(response, 'get_fields_and_field_types'):
        return list(response.get_fields_and_field_types().keys())
    if hasattr(response, '__slots__'):
        return list(response.__slots__)
    return []


class Service:
    def __init__(self, name, service_type, handler, **kwargs):
        self._name = name
        self._service_type = service_type
        self._handler = handler
        self._service = None
        register_node_init_callback(self)

    def _after_node_init(self, node):
        resolved = _resolve_service(node, self._name)

        def wrapped(request, response):
            # Wrap handler to catch exceptions and None returns. ROS2 cannot
            # propagate handler failures to clients, so we log and return a
            # default response to prevent executor crashes.
            try:
                result = self._handler(request)
            except Exception as e:
                logerr(f"Service handler for '{self._name}' raised: {e}")
                return response  # Return default response

            if result is None:
                logerr(f"Service handler for '{self._name}' returned None")
                return response  # Return default response

            if isinstance(result, dict):
                for key, value in result.items():
                    setattr(response, key, value)
            elif isinstance(result, (tuple, list)):
                fields = _get_response_fields(response)
                if len(result) != len(fields):
                    logerr(
                        f"Service handler for '{self._name}' returned "
                        f"{len(result)} values, expected {len(fields)}"
                    )
                    return response  # Return default response
                for i, value in enumerate(result):
                    setattr(response, fields[i], value)
            else:
                for field in _get_response_fields(result):
                    setattr(response, field, getattr(result, field))

            return response

        self._service = node.create_service(self._service_type, resolved, wrapped)

    def shutdown(self, reason=''):
        if self._service is None:
            return
        _get_node().destroy_service(self._service)
        self._service = None


class ServiceProxy:
    def __init__(self, name, service_type, persistent=False, headers=None):
        if persistent:
            logwarn_once('persistent=True is not supported in ROS 2')

        self.node = _get_node()
        resolved = _resolve_service(self.node, name)
        self.service_name = resolved
        self.service_type = service_type
        self.client = self.node.create_client(service_type, resolved)

    def wait_for_service(self, timeout=None):
        if timeout is None:
            while not self.client.service_is_ready():
                if is_shutdown():
                    raise ROSInterruptException('rospy shutdown')
                python_time.sleep(0.1)
            return True

        end_time = python_time.time() + timeout
        while not self.client.service_is_ready():
            if is_shutdown():
                raise ROSInterruptException('rospy shutdown')
            if python_time.time() >= end_time:
                raise ROSException(
                    f'timeout exceeded while waiting for service {self.service_name}'
                )
            python_time.sleep(0.1)
        return True

    def call(self, *args, timeout=None, **kwargs):
        return self(*args, timeout=timeout, **kwargs)

    def __call__(self, *args, timeout=None, **kwargs):
        if is_shutdown():
            raise ROSException('Cannot call service, node is shutdown')

        # Use default timeout if not specified to prevent infinite hangs
        if timeout is None:
            timeout = DEFAULT_SERVICE_TIMEOUT

        if len(args) == 1 and isinstance(args[0], self.service_type.Request):
            request = args[0]
        else:
            request = self.service_type.Request(*args, **kwargs)

        future = self.client.call_async(request)

        # Wait for future to complete using our own loop instead of
        # rclpy.spin_until_future_complete which conflicts with our executor
        start = python_time.time()
        while not future.done():
            if is_shutdown():
                raise ROSException('Shutdown during service call')
            if python_time.time() - start >= timeout:
                raise ServiceException(
                    f'Service call to {self.service_name!r} timed out'
                )
            python_time.sleep(0.01)

        if future.result() is None:
            exc = future.exception()
            if exc:
                raise ServiceException(f'Service call failed: {exc}')
            raise ServiceException(
                f'Service call to {self.service_name!r} failed'
            )

        return future.result()

    def close(self):
        pass


def wait_for_service(service_name, timeout=None):
    node = _get_node()
    resolved = _resolve_service(node, service_name)
    start_time = python_time.time()

    while not is_shutdown():
        names = [n for n, _ in node.get_service_names_and_types()]
        if resolved in names:
            return True
        if timeout is not None and python_time.time() - start_time >= timeout:
            raise ROSException(f'Timeout waiting for service {service_name!r}')
        python_time.sleep(0.1)

    raise ROSException(f'Shutdown while waiting for service {service_name!r}')
