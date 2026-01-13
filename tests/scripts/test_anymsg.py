#!/usr/bin/env python3
# Test AnyMsg functionality
# AnyMsg uses SerializedMessage on Rolling/Humble+ or raw=True on Foxy
# Both approaches should work for receiving raw serialized messages.

import sys
import threading
import time

import pytest


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node('test_anymsg', anonymous=True)


def is_ros2():
    """Check if we're running on ROS2 (has rospy_too)."""
    try:
        import rospy.impl.node
        return hasattr(rospy.impl.node, '_shutdown_node_internal')
    except ImportError:
        return False


def test_anymsg_class_exists():
    """Test that AnyMsg class is importable."""
    from rospy.msg import AnyMsg

    setup()

    # AnyMsg should have expected attributes
    assert hasattr(AnyMsg, '_md5sum'), 'AnyMsg should have _md5sum'
    assert hasattr(AnyMsg, '_type'), 'AnyMsg should have _type'
    assert AnyMsg._md5sum == '*', 'AnyMsg._md5sum should be *'
    assert AnyMsg._type == '*', 'AnyMsg._type should be *'

    # Instance should have _buff
    msg = AnyMsg()
    assert hasattr(msg, '_buff'), 'AnyMsg instance should have _buff'
    # _connection_header may only exist in rospy_too, not ROS1
    # ROS1's AnyMsg sets this during deserialization, not at construction

    print('OK: AnyMsg class exists')


def test_anymsg_subscribe():
    """Test subscribing with AnyMsg receives raw data."""
    import rospy
    from rospy.msg import AnyMsg
    from std_msgs.msg import String

    setup()

    if not is_ros2():
        print('SKIP: test_anymsg_subscribe (ROS1)')
        return

    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    # Create publisher FIRST so AnyMsg can discover the type (required on Foxy)
    pub = rospy.Publisher('/test/anymsg', String, queue_size=10)
    time.sleep(1.0)  # Give time for publisher to be discoverable

    # Create subscriber with AnyMsg
    # This should work on both Foxy (via raw=True) and Rolling (via SerializedMessage)
    sub = rospy.Subscriber('/test/anymsg', AnyMsg, callback)
    time.sleep(0.5)

    # Publish message
    pub.publish(String(data='test_anymsg_data'))

    if not event.wait(timeout=5.0):
        sub.unregister()
        raise AssertionError('Timeout waiting for AnyMsg message')

    sub.unregister()

    # Should have received something
    assert len(received) > 0, 'Should receive message'

    # Received message should have _buff (raw serialized data)
    msg = received[0]
    assert hasattr(msg, '_buff'), 'AnyMsg should have _buff'
    assert msg._buff is not None, '_buff should not be None'
    assert len(msg._buff) > 0, '_buff should have data'

    print('OK: AnyMsg subscribe')


def test_anymsg_connection_header():
    """Test that AnyMsg has _connection_header populated."""
    import rospy
    from rospy.msg import AnyMsg
    from std_msgs.msg import String

    setup()

    if not is_ros2():
        print('SKIP: test_anymsg_connection_header (ROS1)')
        return

    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    # Create publisher FIRST so AnyMsg can discover the type (required on Foxy)
    pub = rospy.Publisher('/test/anymsg_header', String, queue_size=10)
    time.sleep(1.0)  # Give time for publisher to be discoverable

    # Create subscriber with AnyMsg
    sub = rospy.Subscriber('/test/anymsg_header', AnyMsg, callback)
    time.sleep(0.5)

    pub.publish(String(data='header_test'))

    if not event.wait(timeout=5.0):
        sub.unregister()
        raise AssertionError('Timeout waiting for message')

    sub.unregister()

    msg = received[0]
    assert hasattr(msg, '_connection_header'), 'Should have _connection_header'
    header = msg._connection_header
    assert isinstance(header, dict), '_connection_header should be dict'
    assert 'topic' in header, '_connection_header should have topic'

    print('OK: AnyMsg connection header')


def test_anymsg_serialize():
    """Test that AnyMsg.serialize() returns raw bytes."""
    import rospy
    from rospy.msg import AnyMsg

    setup()

    # Test serialize on uninitialized AnyMsg
    msg = AnyMsg()
    with pytest.raises(rospy.ROSException):
        msg.serialize(None)

    print('OK: AnyMsg serialize raises on uninitialized')


def test_anymsg_wrapper_serialize():
    """Test _AnyMsgWrapper.serialize() with buff parameter."""
    import io

    setup()

    if not is_ros2():
        print('SKIP: test_anymsg_wrapper_serialize (ROS1)')
        return

    # Import the wrapper class
    from rospy.topics import _AnyMsgWrapper

    # Create wrapper with some test data
    test_data = b'test serialized data'
    wrapper = _AnyMsgWrapper(test_data, '/test/topic', 'std_msgs/msg/String')

    # Test serialize() with None - should return _buff directly
    result = wrapper.serialize(None)
    assert result == test_data, 'serialize(None) should return _buff'

    # Test serialize() with buff - should write to buff and return value
    buff = io.BytesIO()
    result = wrapper.serialize(buff)
    assert result == test_data, 'serialize(buff) should return buff contents'
    assert buff.getvalue() == test_data, 'buff should contain data'

    print('OK: _AnyMsgWrapper.serialize() with buff')


def test_anymsg_with_callback_args():
    """Test AnyMsg subscriber with callback_args parameter."""
    import rospy
    from rospy.msg import AnyMsg
    from std_msgs.msg import String

    setup()

    if not is_ros2():
        print('SKIP: test_anymsg_with_callback_args (ROS1)')
        return

    received = []
    event = threading.Event()

    def callback_with_args(msg, args):
        received.append((msg, args))
        event.set()

    # Create publisher first for type discovery on Foxy
    pub = rospy.Publisher('/test/anymsg_args', String, queue_size=10)
    time.sleep(1.0)

    # Create subscriber with callback_args
    sub = rospy.Subscriber('/test/anymsg_args', AnyMsg, callback_with_args, callback_args='test_arg')
    time.sleep(0.5)

    pub.publish(String(data='test'))

    if not event.wait(timeout=5.0):
        sub.unregister()
        raise AssertionError('Timeout waiting for message')

    sub.unregister()

    assert len(received) > 0, 'Should receive message'
    msg, args = received[0]
    assert hasattr(msg, '_buff'), 'Message should have _buff'
    assert args == 'test_arg', 'callback_args should be passed to callback'

    print('OK: AnyMsg with callback_args')


def test_import_message_type():
    """Test _import_message_type with different formats."""
    setup()

    if not is_ros2():
        print('SKIP: test_import_message_type (ROS1)')
        return

    from rospy.topics import _import_message_type
    from std_msgs.msg import String

    # Test 3-part format: 'pkg/msg/Type'
    msg_class = _import_message_type('std_msgs/msg/String')
    assert msg_class is String, 'Should import std_msgs/msg/String'

    # Test 2-part format: 'pkg/Type'
    msg_class = _import_message_type('std_msgs/String')
    assert msg_class is String, 'Should import std_msgs/String'

    # Test invalid format
    with pytest.raises(ValueError):
        _import_message_type('invalid')

    print('OK: _import_message_type')


def test_anymsg_works_on_foxy():
    """Test that AnyMsg works on Foxy via raw=True subscription."""
    import rospy
    from rospy.msg import AnyMsg
    from std_msgs.msg import String

    setup()

    if not is_ros2():
        print('SKIP: test_anymsg_works_on_foxy (ROS1)')
        return

    # Check if we're on Foxy (no SerializedMessage)
    try:
        from rclpy.serialization import SerializedMessage  # noqa: F401
        print('OK: AnyMsg works (using SerializedMessage on Rolling/Humble+)')
        return
    except ImportError:
        pass  # We're on Foxy, continue test

    # On Foxy, AnyMsg should work via raw=True
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    # Create publisher FIRST so AnyMsg can discover the type
    pub = rospy.Publisher('/test/anymsg_foxy', String, queue_size=10)
    time.sleep(1.0)  # Give time for publisher to be discoverable

    # This should NOT raise on Foxy anymore
    sub = rospy.Subscriber('/test/anymsg_foxy', AnyMsg, callback)
    time.sleep(0.5)

    pub.publish(String(data='foxy_test'))

    if not event.wait(timeout=5.0):
        sub.unregister()
        raise AssertionError('AnyMsg did not receive message on Foxy')

    sub.unregister()

    assert len(received) > 0, 'Should receive message on Foxy'
    assert hasattr(received[0], '_buff'), 'Should have _buff'

    print('OK: AnyMsg works on Foxy (using raw=True)')


def main():
    failed = 0

    tests = [
        test_anymsg_class_exists,
        test_anymsg_serialize,
        test_anymsg_wrapper_serialize,
        test_import_message_type,
        test_anymsg_with_callback_args,
        test_anymsg_works_on_foxy,
        test_anymsg_subscribe,
        test_anymsg_connection_header,
    ]

    for test in tests:
        try:
            test()
        except Exception as e:
            print('FAIL: %s - %s' % (test.__name__, e))
            failed += 1

    if failed:
        print('\n%d test(s) FAILED' % failed)
        sys.exit(1)
    else:
        print('\nAll tests PASSED')
        sys.exit(0)


if __name__ == '__main__':
    main()
