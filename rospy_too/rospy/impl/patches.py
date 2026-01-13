# Runtime patches for ROS1 compatibility.
# - Monkeypatches builtin_interfaces.msg.Time/Duration with ROS1 methods
# - Patches rclpy exceptions for dual-inheritance
# - Hooks rclpy.create_node() to capture externally created nodes

import warnings

import builtin_interfaces.msg as bi
import rclpy
from rclpy import exceptions as rclpy_exc
from std_msgs.msg import Header

from .. import exceptions as rospy_exc
from ..genpy.rostime import Duration as GenpyDuration
from ..genpy.rostime import Time as GenpyTime
from ..genpy.rostime import TVal, _canon
from ..logging import logwarn_once
from . import node as node_module
from .node import _get_node, _node_lock, _realize_pending, _setup_executor

# Hook rclpy.create_node() to support mixed rospy/rclpy usage
_original_create_node = rclpy.create_node


def _hooked_create_node(*args, **kwargs):
    result = _original_create_node(*args, **kwargs)
    with _node_lock:
        if node_module._node is None:
            node_module._node = result
            _setup_executor()
            _realize_pending()
            # Check for use_sim_time parameter
            try:
                if result.get_parameter('use_sim_time').value:
                    logwarn_once(
                        'use_sim_time is True but rospy_too does not support simulated time; '
                        'Time.now() will return wall clock time'
                    )
            except Exception:
                pass  # Parameter not declared or other error
        else:
            logwarn_once(
                'rclpy.create_node() called multiple times; rospy APIs will target the first node'
            )
    return result


rclpy.create_node = _hooked_create_node


# Patch builtin_interfaces.msg.Time and Duration
def _patch_builtin_interfaces():
    # Property aliases for ROS1 field names (secs/nsecs -> sec/nanosec)
    # These allow genpy methods (which use secs/nsecs) to work with builtin types
    bi.Time.secs = property(
        lambda self: self.sec, lambda self, v: setattr(self, 'sec', v)
    )
    bi.Time.nsecs = property(
        lambda self: self.nanosec, lambda self, v: setattr(self, 'nanosec', v)
    )
    bi.Duration.secs = property(
        lambda self: self.sec, lambda self, v: setattr(self, 'sec', v)
    )
    bi.Duration.nsecs = property(
        lambda self: self.nanosec, lambda self, v: setattr(self, 'nanosec', v)
    )

    # Patch constructors to accept positional args and auto-canonicalize
    def _make_patched_init(original_init, allow_negative=True):
        def patched_init(self, *args, **kwargs):
            # Handle positional args: (secs, nsecs) or (secs,) like genpy
            if args:
                secs = args[0] if len(args) > 0 else 0
                nsecs = args[1] if len(args) > 1 else 0
                # Handle float secs like genpy: Duration(0.5) -> 0 secs, 500000000 nsecs
                if not isinstance(secs, int) and nsecs == 0:
                    float_secs = float(secs)
                    secs = int(float_secs)
                    nsecs = int((float_secs - secs) * 1000000000)
                kwargs['sec'] = int(secs)
                kwargs['nanosec'] = int(nsecs)
                args = ()

            # Get values and canonicalize BEFORE calling original constructor
            # (original constructor rejects negative nanosec as it's unsigned in ROS2)
            sec = kwargs.get('sec', 0)
            nanosec = kwargs.get('nanosec', 0)
            sec, nanosec = _canon(sec, nanosec)
            kwargs['sec'] = sec
            kwargs['nanosec'] = nanosec

            # Time must be non-negative
            if not allow_negative and sec < 0:
                raise ValueError('time values must be positive')

            # Call original constructor with canonicalized values
            original_init(self, *args, **kwargs)

        return patched_init

    bi.Time.__init__ = _make_patched_init(
        bi.Time.__init__, allow_negative=False
    )
    bi.Duration.__init__ = _make_patched_init(
        bi.Duration.__init__, allow_negative=True
    )

    # Copy TVal methods to both Time and Duration
    for method in [
        'to_sec',
        'to_nsec',
        'is_zero',
        '__hash__',
        '__nonzero__',
        '__bool__',
    ]:
        if hasattr(TVal, method):
            setattr(bi.Time, method, getattr(TVal, method))
            setattr(bi.Duration, method, getattr(TVal, method))

    # Copy Time-specific methods from genpy (arithmetic, comparison)
    for method in [
        '__add__',
        '__radd__',
        '__sub__',
        '__eq__',
        '__ne__',
        '__lt__',
        '__le__',
        '__gt__',
        '__ge__',
        '__cmp__',
    ]:
        if hasattr(GenpyTime, method):
            setattr(bi.Time, method, getattr(GenpyTime, method))

    # Copy Duration-specific methods from genpy
    for method in [
        '__add__',
        '__radd__',
        '__sub__',
        '__neg__',
        '__abs__',
        '__mul__',
        '__rmul__',
        '__truediv__',
        '__floordiv__',
        '__div__',
        '__mod__',
        '__divmod__',
        '__eq__',
        '__ne__',
        '__lt__',
        '__le__',
        '__gt__',
        '__ge__',
        '__cmp__',
    ]:
        if hasattr(GenpyDuration, method):
            setattr(bi.Duration, method, getattr(GenpyDuration, method))

    # Set duration class for Time (polymorphic return type for genpy methods)
    bi.Time._duration_class = bi.Duration

    # Time.now() class method (custom - needs rclpy node)
    @classmethod
    def _time_now(cls):
        secs, nsecs = _get_node().get_clock().now().seconds_nanoseconds()
        return cls(secs, nsecs)

    bi.Time.now = _time_now

    # Rebind from_sec classmethods so cls binds to bi.Time/bi.Duration
    bi.Time.from_sec = classmethod(GenpyTime.from_sec.__func__)
    bi.Duration.from_sec = classmethod(GenpyDuration.from_sec.__func__)


_patch_builtin_interfaces()


# Patch std_msgs.msg.Header to add seq property
_seq_warned = False


def _patch_header():
    def _get_seq(self):
        return getattr(self, '_seq_value', 0)

    def _set_seq(self, value):
        global _seq_warned
        if not _seq_warned:
            warnings.warn(
                'Header.seq is deprecated in ROS 2 and has no effect',
                DeprecationWarning,
                stacklevel=2,
            )
            _seq_warned = True
        self._seq_value = value

    if not hasattr(Header, '_seq_patched'):
        Header.seq = property(_get_seq, _set_seq)
        Header._seq_patched = True


_patch_header()


# Patch rclpy exceptions for dual-inheritance
def _patch_exceptions():
    # Create dual-inheritance exception classes
    if hasattr(rclpy_exc, 'ROSInterruptException'):
        _orig = rclpy_exc.ROSInterruptException

        class ROSInterruptException(rospy_exc.ROSInterruptException, _orig):
            pass

        rclpy_exc.ROSInterruptException = ROSInterruptException


_patch_exceptions()


def _patch_type_mappings():
    try:
        import rcl_interfaces.msg
        import rosgraph_msgs.msg

        rosgraph_msgs.msg.Log = rcl_interfaces.msg.Log
    except ImportError:
        pass


_patch_type_mappings()
