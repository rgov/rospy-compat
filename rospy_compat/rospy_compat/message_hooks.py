"""
Import hooks for ROS message compatibility.

This module provides an import hook system that intercepts ROS message and service
imports (*.msg and *.srv modules) and wraps message classes with ROS1-compatible
versions that support:
- Positional arguments (ROS1 style)
- Auto-populated header timestamps
- Full backward compatibility with keyword arguments
"""

import sys
import importlib
import types
from importlib.abc import MetaPathFinder, Loader


class MessageImportFinder(MetaPathFinder):
    """
    Import hook that intercepts ROS message and service module imports.

    Installed into sys.meta_path to catch imports of .msg and .srv packages
    and return a custom loader that wraps message classes for ROS1 compatibility.
    """

    def find_module(self, fullname, path=None):
        """
        Find a module (Python 2 style, kept for compatibility).

        Args:
            fullname (str): Fully qualified module name
            path: Module search path (unused)

        Returns:
            MessageImportLoader if this is a .msg or .srv module, None otherwise
        """
        # Intercept if importing from a .msg or .srv module
        if '.msg' in fullname or '.srv' in fullname:
            return MessageImportLoader(fullname)
        return None

    def find_spec(self, fullname, path, target=None):
        """
        Find a module spec (Python 3.4+ style).

        Args:
            fullname (str): Fully qualified module name
            path: Module search path (unused)
            target: Target module (unused)

        Returns:
            ModuleSpec if this is a .msg or .srv module, None otherwise
        """
        # Python 3.4+ uses find_spec instead of find_module
        if '.msg' in fullname or '.srv' in fullname:
            from importlib.machinery import ModuleSpec
            return ModuleSpec(fullname, MessageImportLoader(fullname))
        return None


class MessageImportLoader(Loader):
    """
    Custom loader that wraps ROS message classes with ROS1-compatible versions.
    """

    def __init__(self, fullname):
        """
        Initialize loader.

        Args:
            fullname (str): Fully qualified module name to load
        """
        self.fullname = fullname

    def load_module(self, fullname):
        """
        Load and wrap a message module.

        Args:
            fullname (str): Fully qualified module name

        Returns:
            Wrapped module with enhanced message classes
        """
        # Check if already loaded
        if fullname in sys.modules:
            return sys.modules[fullname]

        # Import the real module first
        # We need to temporarily remove our hook to avoid recursion
        original_meta_path = sys.meta_path[:]
        try:
            # Remove this hook temporarily
            sys.meta_path = [f for f in sys.meta_path if not isinstance(f, MessageImportFinder)]

            # Import the original module
            original_module = importlib.import_module(fullname)

        finally:
            # Restore meta_path
            sys.meta_path = original_meta_path

        # Wrap all message classes in the module
        wrapped_module = _wrap_message_module(original_module)

        # Cache the wrapped module
        sys.modules[fullname] = wrapped_module

        return wrapped_module


def _wrap_message_module(original_module):
    """
    Scan module for message/service classes and wrap them with ROS1-compatible versions.

    Also detects service types and injects standalone Request/Response classes
    for ROS1 compatibility (e.g., AddTwoIntsRequest, AddTwoIntsResponse).

    Args:
        original_module: The original ROS2 message/service module

    Returns:
        A new module object with wrapped message classes and service aliases
    """
    wrapped_attrs = {}

    for attr_name in dir(original_module):
        attr = getattr(original_module, attr_name)

        # Check for service types first (has Request/Response nested classes)
        if _is_service_type(attr):
            # Wrap service type and inject Request/Response aliases
            wrapped_attrs[attr_name] = _wrap_service_type(attr_name, attr, wrapped_attrs)
        # Then check if this is a message class (has _fields_and_field_types)
        elif (isinstance(attr, type) and
              hasattr(attr, '_fields_and_field_types') and
              hasattr(attr, '__slots__')):
            # Wrap this message class
            wrapped_attrs[attr_name] = _create_message_wrapper(attr)
        else:
            # Keep non-message/service attributes as-is
            wrapped_attrs[attr_name] = attr

    # Create a new module-like object with wrapped attributes
    wrapped_module = types.ModuleType(original_module.__name__)
    wrapped_module.__dict__.update(wrapped_attrs)

    # Preserve module metadata
    if hasattr(original_module, '__file__'):
        wrapped_module.__file__ = original_module.__file__
    if hasattr(original_module, '__package__'):
        wrapped_module.__package__ = original_module.__package__
    if hasattr(original_module, '__path__'):
        wrapped_module.__path__ = original_module.__path__
    if hasattr(original_module, '__doc__'):
        wrapped_module.__doc__ = original_module.__doc__

    return wrapped_module


def _create_message_wrapper(original_class):
    """
    Enhance the original message class with ROS1-compatible initialization.
    Modifies the class in-place rather than creating a subclass.

    Args:
        original_class: The original ROS2 message class

    Returns:
        The same class with enhanced __init__ method
    """
    # Save the original __init__ method
    original_init = original_class.__init__

    def enhanced_init(self, *args, **kwargs):
        """
        Initialize message with ROS1-compatible positional arguments.

        Args:
            *args: Positional arguments (ROS1 style)
            **kwargs: Keyword arguments (ROS2 style)
        """
        # Get field names from the class
        fields = list(self._fields_and_field_types.keys())

        # Convert positional args to kwargs
        for i, arg in enumerate(args):
            if i < len(fields):
                field_name = fields[i]

                # Handle None -> auto-create default object
                if arg is None:
                    # Get field type and create default
                    field_type = self._fields_and_field_types[field_name]

                    # Check if this is a Header field
                    if 'Header' in field_type:
                        try:
                            from std_msgs.msg import Header
                            arg = Header()
                        except ImportError:
                            # If std_msgs not available, skip
                            pass
                    # For other types, let ROS2 use its default

                # Only set if not None after conversion
                if arg is not None:
                    kwargs[field_name] = arg

        # Auto-populate header.stamp if header field exists
        # IMPORTANT: Do this BEFORE calling original __init__
        if 'header' in fields:
            header = kwargs.get('header')
            if header is not None and hasattr(header, 'stamp'):
                # Populate timestamp if it's zero
                if header.stamp.sec == 0 and header.stamp.nanosec == 0:
                    try:
                        from .time import Time
                        now = Time.now()
                        # Modify stamp fields directly
                        header.stamp.sec = now.secs
                        header.stamp.nanosec = now.nsecs
                    except Exception:
                        # If Time.now() fails (no node yet), leave as zero
                        pass

        # Call original ROS2 constructor with keyword args
        original_init(self, **kwargs)

    # Replace the __init__ method on the original class
    original_class.__init__ = enhanced_init

    # Store reference to original init for potential restoration
    original_class._original_ros2_init = original_init

    return original_class


def _is_service_type(attr):
    """
    Check if an attribute is a ROS2 service type class.

    Service types have nested Request and Response classes that are
    themselves message-like classes with _fields_and_field_types.

    Args:
        attr: Module attribute to check

    Returns:
        bool: True if attr is a service type, False otherwise
    """
    return (
        isinstance(attr, type) and
        hasattr(attr, 'Request') and
        hasattr(attr, 'Response') and
        isinstance(attr.Request, type) and
        isinstance(attr.Response, type) and
        hasattr(attr.Request, '_fields_and_field_types') and
        hasattr(attr.Response, '_fields_and_field_types')
    )


def _wrap_service_type(service_name, service_class, wrapped_attrs):
    """
    Wrap a service type's nested Request/Response classes and inject standalone aliases.

    For a service type like 'AddTwoInts', this:
    1. Wraps AddTwoInts.Request to support positional args
    2. Wraps AddTwoInts.Response to support positional args
    3. Injects 'AddTwoIntsRequest' as alias to wrapped Request
    4. Injects 'AddTwoIntsResponse' as alias to wrapped Response

    This enables ROS1-style service usage:
        from pkg.srv import AddTwoInts, AddTwoIntsResponse
        return AddTwoIntsResponse(sum_value)  # Positional arg support

    Args:
        service_name (str): Name of the service type (e.g., 'AddTwoInts')
        service_class: The service type class
        wrapped_attrs (dict): Dictionary to inject the aliases into

    Returns:
        The service class (with wrapped nested classes)
    """
    # Wrap the nested Request class
    if hasattr(service_class, 'Request'):
        wrapped_request = _create_message_wrapper(service_class.Request)
        service_class.Request = wrapped_request

        # Inject standalone Request alias: AddTwoIntsRequest
        request_alias_name = f"{service_name}Request"
        wrapped_attrs[request_alias_name] = wrapped_request

    # Wrap the nested Response class
    if hasattr(service_class, 'Response'):
        wrapped_response = _create_message_wrapper(service_class.Response)
        service_class.Response = wrapped_response

        # Inject standalone Response alias: AddTwoIntsResponse
        response_alias_name = f"{service_name}Response"
        wrapped_attrs[response_alias_name] = wrapped_response

    return service_class


def _auto_populate_header(header):
    """
    Auto-populate header.stamp with current time if it's zero.

    Args:
        header: A std_msgs/Header or similar header object
    """
    # Check if stamp is zero (not yet set)
    if not hasattr(header, 'stamp'):
        return

    if header.stamp.sec == 0 and header.stamp.nanosec == 0:
        # Import here to avoid circular dependency
        try:
            from .time import Time

            # Get current time from node clock
            now = Time.now()

            # Modify the existing stamp object's fields directly
            # Don't replace the stamp object itself
            header.stamp.sec = now.secs
            header.stamp.nanosec = now.nsecs
        except Exception:
            # If Time.now() fails (no node yet), leave timestamp as zero
            # This is expected if message is created before init_node()
            pass


def install_message_hooks():
    """
    Install the import hook into sys.meta_path.

    Should be called once when rospy_compat is imported.
    This installs the hook at the beginning of sys.meta_path so it has
    priority over other import hooks.
    """
    # Check if already installed
    for finder in sys.meta_path:
        if isinstance(finder, MessageImportFinder):
            return  # Already installed

    # Insert at beginning for priority
    sys.meta_path.insert(0, MessageImportFinder())
