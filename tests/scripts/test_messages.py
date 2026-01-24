#!/usr/bin/env python3
# Test message construction and features
# These tests verify message creation works correctly.

import sys

# Import rospy first to install message hooks before importing any messages
import rospy


def setup():
    if not rospy.core.is_initialized():
        rospy.init_node("test_messages", anonymous=True)


def test_string_message():
    from std_msgs.msg import String

    setup()

    # Keyword construction
    msg = String(data="hello")
    assert msg.data == "hello", "Expected hello, got %s" % msg.data
    print("OK: String message keyword construction")

    # Positional construction (ROS 1 native, ROS 2 via import hooks)
    msg = String("world")
    assert msg.data == "world", "Expected world, got %s" % msg.data
    print("OK: String message positional construction")


def test_int32_message():
    from std_msgs.msg import Int32

    setup()

    msg = Int32(data=42)
    assert msg.data == 42, "Expected 42, got %s" % msg.data
    print("OK: Int32 message keyword construction")

    msg = Int32(42)
    assert msg.data == 42, "Expected 42, got %s" % msg.data
    print("OK: Int32 message positional construction")


def test_header_message():
    from std_msgs.msg import Header

    setup()

    # Basic construction
    msg = Header()
    # Header.seq should exist (real field in ROS 1, property returning 0 in ROS 2)
    seq = msg.seq
    assert isinstance(seq, int), "Expected seq to be int, got %s" % type(seq)
    print("OK: Header.seq exists")

    # stamp should be accessible
    stamp = msg.stamp  # noqa: F841
    print("OK: Header.stamp accessible")

    # frame_id should work
    msg.frame_id = "test_frame"
    assert msg.frame_id == "test_frame", "Expected test_frame, got %s" % msg.frame_id
    print("OK: Header.frame_id")


def test_header_stamp_fields():
    from std_msgs.msg import Header

    setup()

    msg = Header()

    # In ROS 1: stamp.secs, stamp.nsecs
    # In ROS 2: stamp.sec, stamp.nanosec
    # rospy-too provides .secs and .nsecs aliases on ROS 2

    # Test that secs/nsecs work (native in ROS 1, patched in ROS 2)
    if hasattr(msg.stamp, "secs"):
        msg.stamp.secs = 10
        msg.stamp.nsecs = 500000000
        assert msg.stamp.secs == 10, "Expected secs=10"
        assert msg.stamp.nsecs == 500000000, "Expected nsecs=500000000"
        print("OK: Header.stamp.secs/nsecs")
    elif hasattr(msg.stamp, "sec"):
        # Pure ROS 2 without patches (shouldn't happen with rospy-too)
        msg.stamp.sec = 10
        msg.stamp.nanosec = 500000000
        print("OK: Header.stamp.sec/nanosec (ROS 2 native)")


def test_complex_message():
    # Use a message type available in both ROS 1 and ROS 2
    from geometry_msgs.msg import Point

    setup()

    # Keyword construction
    p = Point(x=1.0, y=2.0, z=3.0)
    assert p.x == 1.0, "Expected x=1.0"
    assert p.y == 2.0, "Expected y=2.0"
    assert p.z == 3.0, "Expected z=3.0"
    print("OK: Point keyword construction")

    # Positional construction
    p = Point(4.0, 5.0, 6.0)
    assert p.x == 4.0, "Expected x=4.0"
    assert p.y == 5.0, "Expected y=5.0"
    assert p.z == 6.0, "Expected z=6.0"
    print("OK: Point positional construction")


def test_nested_message():
    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import Header

    setup()

    # Construct with nested message
    header = Header(frame_id="world")
    # PointStamped has header and point fields
    msg = PointStamped()
    msg.header = header
    msg.point.x = 1.0
    msg.point.y = 2.0
    msg.point.z = 3.0

    assert msg.header.frame_id == "world", "Expected frame_id=world"
    assert msg.point.x == 1.0, "Expected point.x=1.0"
    print("OK: nested message construction")


def test_header_stamp_not_auto_populated():
    """Verify headers are NOT auto-stamped (matches ROS1 genpy behavior)"""
    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import Header

    setup()

    # Create a header - stamp should be zero (not auto-populated)
    header = Header()
    # ROS1 uses secs/nsecs, ROS2 uses sec/nanosec
    sec_attr = 'secs' if hasattr(header.stamp, 'secs') else 'sec'
    nsec_attr = 'nsecs' if hasattr(header.stamp, 'nsecs') else 'nanosec'

    sec_val = getattr(header.stamp, sec_attr)
    nsec_val = getattr(header.stamp, nsec_attr)
    assert sec_val == 0, 'Expected stamp.%s=0, got %d' % (sec_attr, sec_val)
    assert nsec_val == 0, 'Expected stamp.%s=0, got %d' % (nsec_attr, nsec_val)
    print('OK: Header.stamp defaults to zero')

    # Create a message with header - stamp should still be zero
    msg = PointStamped()
    sec_val = getattr(msg.header.stamp, sec_attr)
    nsec_val = getattr(msg.header.stamp, nsec_attr)
    assert sec_val == 0, 'Expected header.stamp.%s=0' % sec_attr
    assert nsec_val == 0, 'Expected header.stamp.%s=0' % nsec_attr
    print('OK: PointStamped header.stamp defaults to zero')

    # Creating with explicit header should not auto-stamp
    explicit_header = Header(frame_id='test')
    msg2 = PointStamped()
    msg2.header = explicit_header
    sec_val = getattr(msg2.header.stamp, sec_attr)
    nsec_val = getattr(msg2.header.stamp, nsec_attr)
    assert sec_val == 0, 'Expected explicit header stamp.%s to remain zero' % sec_attr
    assert nsec_val == 0, 'Expected explicit header stamp.%s to remain zero' % nsec_attr
    print('OK: Explicit header stamp remains zero (not auto-populated)')


def is_ros2():
    try:
        import rclpy
        return True
    except ImportError:
        return False


def test_time_duration_positional_args():
    """Test Time/Duration construction with positional arguments (ROS2 hook coverage)."""
    if not is_ros2():
        print("SKIP: test_time_duration_positional_args (ROS1)")
        return

    setup()

    from builtin_interfaces.msg import Time, Duration

    # Test Time with single float arg (covers _float_to_sec_nanosec)
    t = Time(1.5)
    assert t.sec == 1, "Expected sec=1, got %s" % t.sec
    assert t.nanosec == 500000000, "Expected nanosec=500000000, got %s" % t.nanosec
    print("OK: Time(1.5) positional float")

    # Test Time with two positional args
    t2 = Time(10, 123456789)
    assert t2.sec == 10, "Expected sec=10, got %s" % t2.sec
    assert t2.nanosec == 123456789, "Expected nanosec=123456789, got %s" % t2.nanosec
    print("OK: Time(10, 123456789) positional")

    # Test Duration with single float arg
    d = Duration(2.25)
    assert d.sec == 2, "Expected sec=2, got %s" % d.sec
    assert d.nanosec == 250000000, "Expected nanosec=250000000, got %s" % d.nanosec
    print("OK: Duration(2.25) positional float")

    # Test Duration with two positional args
    d2 = Duration(5, 999999999)
    assert d2.sec == 5, "Expected sec=5, got %s" % d2.sec
    assert d2.nanosec == 999999999, "Expected nanosec=999999999, got %s" % d2.nanosec
    print("OK: Duration(5, 999999999) positional")


def test_header_none_autofill():
    """Test that None for header fields gets auto-filled (ROS2 hook coverage)."""
    if not is_ros2():
        print("SKIP: test_header_none_autofill (ROS1)")
        return

    setup()

    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import Header

    # Pass None as first positional arg (header field) - should auto-create Header
    msg = PointStamped(None)
    assert msg.header is not None, "Expected header to be auto-created"
    assert isinstance(msg.header, Header), "Expected Header type"
    print("OK: Header None autofill")


def test_message_wrapper_idempotency():
    """Test that wrapping a message twice is idempotent (ROS2 hook coverage)."""
    if not is_ros2():
        print("SKIP: test_message_wrapper_idempotency (ROS1)")
        return

    setup()

    from std_msgs.msg import String

    # String should be marked as wrapped
    assert hasattr(String, '_rospy_wrapped'), "Expected _rospy_wrapped attribute"
    assert String._rospy_wrapped is True, "Expected _rospy_wrapped=True"

    # Calling wrapper again should return same class
    from rospy.impl.hooks import _create_message_wrapper
    wrapped = _create_message_wrapper(String)
    assert wrapped is String, "Expected same class on re-wrap"
    print("OK: Message wrapper idempotency")


def test_int_to_float_coercion():
    """Test assigning int to float field is coerced to float (ROS1 genpy compatibility).

    ROS1's genpy allows int values for float fields (coerced at serialization).
    ROS2's C extension requires exact float types (crashes with PyFloat_Check assertion).
    rospy_too coerces int to float at assignment time to ensure compatibility.
    """
    if not is_ros2():
        print("SKIP: test_int_to_float_coercion (ROS1)")
        return

    from geometry_msgs.msg import Point

    setup()

    # Point.x, Point.y, Point.z are float64 fields
    p = Point()
    p.x = 1  # int assigned to float64 field
    assert p.x == 1.0, "Expected value 1.0, got %s" % p.x
    assert isinstance(p.x, float), "Expected float type after coercion, got %s" % type(p.x).__name__
    print("OK: int coerced to float")


def test_uint8_array_to_bytes():
    """Test that uint8[] fields return bytes (ROS1 genpy compatibility).

    ROS1's genpy treats uint8[] fields as bytes (b''), not lists.
    ROS2 may return array.array('B') or list depending on version.
    rospy_too coerces uint8[] fields to bytes on read for compatibility.
    """
    if not is_ros2():
        print("SKIP: test_uint8_array_to_bytes (ROS1)")
        return

    from sensor_msgs.msg import Image

    setup()

    img = Image()

    # Test assignment with list
    img.data = [0, 128, 255]
    result = img.data
    assert isinstance(result, bytes), "Expected bytes, got %s" % type(result).__name__
    assert result == bytes([0, 128, 255]), "Expected b'\\x00\\x80\\xff', got %r" % result
    print("OK: uint8[] field returns bytes from list assignment")

    # Test assignment with bytes (should pass through)
    img.data = b'\x01\x02\x03'
    result = img.data
    assert isinstance(result, bytes), "Expected bytes, got %s" % type(result).__name__
    assert result == b'\x01\x02\x03', "Expected b'\\x01\\x02\\x03', got %r" % result
    print("OK: uint8[] field returns bytes from bytes assignment")

    # Test empty array
    img.data = []
    result = img.data
    assert isinstance(result, bytes), "Expected bytes, got %s" % type(result).__name__
    assert result == b'', "Expected b'', got %r" % result
    print("OK: uint8[] field returns bytes from empty list")

    # Test assignment with bytearray
    img.data = bytearray([10, 20, 30])
    result = img.data
    assert isinstance(result, bytes), "Expected bytes, got %s" % type(result).__name__
    assert result == bytes([10, 20, 30]), "Expected b'\\x0a\\x14\\x1e', got %r" % result
    print("OK: uint8[] field returns bytes from bytearray assignment")


def main():
    failed = 0

    tests = [
        test_string_message,
        test_int32_message,
        test_header_message,
        test_header_stamp_fields,
        test_complex_message,
        test_nested_message,
        test_header_stamp_not_auto_populated,
        test_time_duration_positional_args,
        test_header_none_autofill,
        test_message_wrapper_idempotency,
        test_int_to_float_coercion,
        test_uint8_array_to_bytes,
    ]

    for test in tests:
        try:
            test()
        except Exception as e:
            print("FAIL: %s - %s" % (test.__name__, e))
            failed += 1

    if failed:
        print("\n%d test(s) FAILED" % failed)
        sys.exit(1)
    else:
        print("\nAll tests PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()
