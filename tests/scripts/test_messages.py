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
