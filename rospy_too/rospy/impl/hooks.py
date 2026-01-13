# Import hooks for ROS message compatibility.
# Intercepts imports of *.msg and *.srv modules and wraps message classes
# with ROS1-compatible versions supporting positional arguments.

import importlib
import sys
import types
from importlib.abc import Loader, MetaPathFinder
from importlib.machinery import ModuleSpec


def _is_message_module(fullname):
    # Match pkg.msg, pkg.srv, or pkg.action patterns (exact component match, not substring)
    parts = fullname.split('.')
    if len(parts) < 2:
        return False
    for i in range(1, len(parts)):
        if parts[i] in ('msg', 'srv', 'action'):
            return True
    return False


class MessageImportFinder(MetaPathFinder):
    def find_module(self, fullname, path=None):
        if _is_message_module(fullname):
            return MessageImportLoader(fullname)
        return None

    def find_spec(self, fullname, path, target=None):
        if _is_message_module(fullname):
            return ModuleSpec(fullname, MessageImportLoader(fullname))
        return None


class MessageImportLoader(Loader):
    def __init__(self, fullname):
        self.fullname = fullname

    def create_module(self, spec):
        # Return None to use default module creation
        return None

    def exec_module(self, module):
        fullname = module.__name__

        # Remove from sys.modules temporarily
        sys.modules.pop(fullname, None)

        # Temporarily remove hook to avoid recursion
        original_meta_path = sys.meta_path[:]
        sys.meta_path = [
            f for f in sys.meta_path if not isinstance(f, MessageImportFinder)
        ]

        try:
            original_module = importlib.import_module(fullname)

            # Wrap message classes
            for attr_name in dir(original_module):
                attr = getattr(original_module, attr_name)

                if _is_action_type(attr):
                    _wrap_action_type(attr_name, attr, module.__dict__)
                    setattr(module, attr_name, attr)
                elif _is_service_type(attr):
                    _wrap_service_type(attr_name, attr, module.__dict__)
                    setattr(module, attr_name, attr)
                elif isinstance(attr, type) and hasattr(
                    attr, '_fields_and_field_types'
                ):
                    wrapped = _create_message_wrapper(attr)
                    setattr(module, attr_name, wrapped)
                else:
                    setattr(module, attr_name, attr)

            # Copy module metadata
            for meta in ('__file__', '__package__', '__path__', '__doc__'):
                if hasattr(original_module, meta):
                    setattr(module, meta, getattr(original_module, meta))

            # Inject action aliases for .msg modules (ROS1 compatibility)
            parts = fullname.split('.')
            if len(parts) >= 2 and parts[-1] == 'msg':
                package_name = '.'.join(parts[:-1])
                _inject_action_aliases(module, package_name)
        finally:
            sys.meta_path = original_meta_path
            # Ensure our wrapped module is in sys.modules
            sys.modules[fullname] = module

    def load_module(self, fullname):
        # Fallback for older Python versions
        if fullname in sys.modules:
            return sys.modules[fullname]

        # Temporarily remove hook to avoid recursion
        original_meta_path = sys.meta_path[:]
        sys.meta_path = [
            f for f in sys.meta_path if not isinstance(f, MessageImportFinder)
        ]

        try:
            original_module = importlib.import_module(fullname)
        finally:
            sys.meta_path = original_meta_path

        wrapped_module = _wrap_message_module(original_module)
        sys.modules[fullname] = wrapped_module
        return wrapped_module


def _wrap_message_module(original_module):
    wrapped_attrs = {}

    for attr_name in dir(original_module):
        attr = getattr(original_module, attr_name)

        if _is_action_type(attr):
            wrapped_attrs[attr_name] = _wrap_action_type(
                attr_name, attr, wrapped_attrs
            )
        elif _is_service_type(attr):
            wrapped_attrs[attr_name] = _wrap_service_type(
                attr_name, attr, wrapped_attrs
            )
        elif isinstance(attr, type) and hasattr(
            attr, '_fields_and_field_types'
        ):
            # ROS 2 message class - wrap for positional arg support
            wrapped_attrs[attr_name] = _create_message_wrapper(attr)
        else:
            wrapped_attrs[attr_name] = attr

    wrapped_module = types.ModuleType(original_module.__name__)
    wrapped_module.__dict__.update(wrapped_attrs)

    for meta in ('__file__', '__package__', '__path__', '__doc__'):
        if hasattr(original_module, meta):
            setattr(wrapped_module, meta, getattr(original_module, meta))

    # Inject action aliases for .msg modules (ROS1 compatibility)
    fullname = original_module.__name__
    parts = fullname.split('.')
    if len(parts) >= 2 and parts[-1] == 'msg':
        package_name = '.'.join(parts[:-1])
        _inject_action_aliases(wrapped_module, package_name)

    return wrapped_module


def _float_to_sec_nanosec(val):
    sec = int(val)
    nanosec = int((val - sec) * 1e9)
    return sec, nanosec


def _is_time_or_duration(cls):
    return cls.__module__.startswith(
        'builtin_interfaces.msg'
    ) and cls.__name__ in ('Time', 'Duration')


def _create_message_wrapper(original_class):
    # Check if already wrapped
    if hasattr(original_class, '_rospy_wrapped'):
        return original_class

    original_init = original_class.__init__
    is_time_type = _is_time_or_duration(original_class)

    def enhanced_init(self, *args, **kwargs):
        # Handle Time/Duration with ROS1-style constructor: (secs) or (secs, nsecs)
        if is_time_type and args and not kwargs:
            if len(args) == 1:
                sec, nanosec = _float_to_sec_nanosec(args[0])
                kwargs = {'sec': sec, 'nanosec': nanosec}
                args = ()
            elif len(args) == 2:
                kwargs = {'sec': int(args[0]), 'nanosec': int(args[1])}
                args = ()

        fields = list(self._fields_and_field_types.keys())

        for i, arg in enumerate(args):
            if i < len(fields):
                field_name = fields[i]
                if arg is None and 'Header' in self._fields_and_field_types.get(
                    field_name, ''
                ):
                    from std_msgs.msg import Header

                    arg = Header()
                if arg is not None:
                    kwargs[field_name] = arg

        original_init(self, **kwargs)

    # Try to replace __init__ directly
    try:
        original_class.__init__ = enhanced_init
        original_class._rospy_wrapped = True
        original_class._original_ros2_init = original_init
    except (TypeError, AttributeError):
        # Class is immutable (e.g., Cython extension type)
        # Create a dynamic subclass instead
        class WrappedMessage(original_class):
            _rospy_wrapped = True
            _original_ros2_init = original_init

            def __init__(self, *args, **kwargs):
                fields = list(self._fields_and_field_types.keys())

                for i, arg in enumerate(args):
                    if i < len(fields):
                        field_name = fields[i]
                        if arg is not None:
                            kwargs[field_name] = arg

                original_init(self, **kwargs)

        WrappedMessage.__name__ = original_class.__name__
        WrappedMessage.__qualname__ = original_class.__qualname__
        WrappedMessage.__module__ = original_class.__module__
        return WrappedMessage

    return original_class


def _is_service_type(attr):
    return (
        isinstance(attr, type)
        and hasattr(attr, 'Request')
        and hasattr(attr, 'Response')
        and isinstance(attr.Request, type)
        and isinstance(attr.Response, type)
        and hasattr(attr.Request, '_fields_and_field_types')
        and hasattr(attr.Response, '_fields_and_field_types')
    )


def _is_action_type(attr):
    return (
        isinstance(attr, type)
        and hasattr(attr, 'Goal')
        and hasattr(attr, 'Result')
        and hasattr(attr, 'Feedback')
        and isinstance(attr.Goal, type)
        and isinstance(attr.Result, type)
        and isinstance(attr.Feedback, type)
        and hasattr(attr.Goal, '_fields_and_field_types')
        and hasattr(attr.Result, '_fields_and_field_types')
        and hasattr(attr.Feedback, '_fields_and_field_types')
    )


def _wrap_service_type(service_name, service_class, wrapped_attrs):
    if hasattr(service_class, 'Request'):
        wrapped_request = _create_message_wrapper(service_class.Request)
        service_class.Request = wrapped_request
        wrapped_attrs[f'{service_name}Request'] = wrapped_request

    if hasattr(service_class, 'Response'):
        wrapped_response = _create_message_wrapper(service_class.Response)
        service_class.Response = wrapped_response
        wrapped_attrs[f'{service_name}Response'] = wrapped_response

    return service_class


def _wrap_action_type(action_name, action_class, wrapped_attrs):
    if hasattr(action_class, 'Goal'):
        wrapped_goal = _create_message_wrapper(action_class.Goal)
        action_class.Goal = wrapped_goal
        wrapped_attrs[f'{action_name}Goal'] = wrapped_goal

    if hasattr(action_class, 'Result'):
        wrapped_result = _create_message_wrapper(action_class.Result)
        action_class.Result = wrapped_result
        wrapped_attrs[f'{action_name}Result'] = wrapped_result

    if hasattr(action_class, 'Feedback'):
        wrapped_feedback = _create_message_wrapper(action_class.Feedback)
        action_class.Feedback = wrapped_feedback
        wrapped_attrs[f'{action_name}Feedback'] = wrapped_feedback

    return action_class


def _inject_action_aliases(module, package_name):
    """Inject ROS1-style action aliases into a msg module.

    In ROS1, action files generate types in package.msg:
        from package.msg import FooAction, FooGoal, FooResult, FooFeedback

    In ROS2, action files generate types in package.action:
        from package.action import Foo  # then Foo.Goal, Foo.Result, etc.

    This function bridges the gap by adding ROS1-style aliases to msg modules.
    """
    try:
        action_module = importlib.import_module(f'{package_name}.action')
    except ImportError:
        return  # No action module, nothing to do

    for attr_name in dir(action_module):
        attr = getattr(action_module, attr_name)
        if _is_action_type(attr):
            # Wrap the action type's Goal/Result/Feedback for positional args
            _wrap_action_type(attr_name, attr, module.__dict__)

            # Add ROS1-style aliases
            module.__dict__[f'{attr_name}Action'] = attr
            module.__dict__[f'{attr_name}Goal'] = attr.Goal
            module.__dict__[f'{attr_name}Result'] = attr.Result
            module.__dict__[f'{attr_name}Feedback'] = attr.Feedback
            # ActionGoal/ActionResult/ActionFeedback are just aliases in compat mode
            module.__dict__[f'{attr_name}ActionGoal'] = attr.Goal
            module.__dict__[f'{attr_name}ActionResult'] = attr.Result
            module.__dict__[f'{attr_name}ActionFeedback'] = attr.Feedback


def install_message_hooks():
    for finder in sys.meta_path:
        if isinstance(finder, MessageImportFinder):
            return
    sys.meta_path.insert(0, MessageImportFinder())
