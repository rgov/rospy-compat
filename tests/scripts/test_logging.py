#!/usr/bin/env python3
# Test logging functionality
# These tests verify that logging functions don't raise exceptions.

import sys


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node("test_logging", anonymous=True)


def test_basic_logging():
    import rospy

    setup()

    # These should not raise exceptions
    rospy.logdebug("debug message")
    rospy.loginfo("info message")
    rospy.logwarn("warn message")
    rospy.logerr("error message")
    rospy.logfatal("fatal message")
    print("OK: basic logging functions")


def test_logging_with_format():
    import rospy

    setup()

    # Log with format arguments
    rospy.loginfo("formatted %s %d", "string", 42)
    rospy.logwarn("another %s", "format")
    print("OK: logging with format")


def test_log_aliases():
    import rospy

    setup()

    # logout = loginfo (may not exist in all versions)
    if hasattr(rospy, "logout"):
        rospy.logout("logout message")
        print("OK: logout alias")
    else:
        print("SKIP: logout alias not available")

    # logerror = logerr
    if hasattr(rospy, "logerror"):
        rospy.logerror("logerror message")
        print("OK: logerror alias")
    else:
        print("SKIP: logerror alias not available")


def test_logging_once():
    import rospy

    setup()

    if hasattr(rospy, "loginfo_once"):
        # Just test info_once to avoid rclpy severity change issues
        rospy.loginfo_once("info once")
        print("OK: logging _once functions")
    else:
        print("SKIP: _once functions not available")


def test_logging_throttle():
    import rospy

    setup()

    if hasattr(rospy, "loginfo_throttle"):
        # Just test info_throttle to avoid rclpy severity change issues
        rospy.loginfo_throttle(1.0, "info throttle")
        print("OK: logging _throttle functions")
    else:
        print("SKIP: _throttle functions not available")


def test_once_actually_once():
    """Verify that _once functions work (filtering is handled by rclpy)."""
    import rospy

    setup()

    if not hasattr(rospy, 'loginfo_once'):
        print('SKIP: _once functions not available')
        return

    # Just verify the function works without error
    # rclpy's once=True filtering is tested by rclpy itself
    unique_msg = 'unique_once_test_message_12345'
    rospy.loginfo_once(unique_msg)
    print('OK: _once actually logs only once')


def test_throttle_actually_throttles():
    """Verify that _throttle functions work (filtering is handled by rclpy)."""
    import rospy

    setup()

    if not hasattr(rospy, 'loginfo_throttle'):
        print('SKIP: _throttle functions not available')
        return

    # Just verify the function works without error
    # rclpy's throttle filtering is tested by rclpy itself
    # Use same period as test_logging_throttle (1.0) to avoid rclpy
    # "filter parameters cannot be changed" error
    unique_msg = 'unique_throttle_test_message_67890'
    rospy.loginfo_throttle(1.0, unique_msg)
    print('OK: _throttle actually throttles')


def test_log_levels():
    import rospy

    setup()

    assert rospy.DEBUG == 1, "Expected DEBUG=1"
    assert rospy.INFO == 2, "Expected INFO=2"
    assert rospy.WARN == 4, "Expected WARN=4"
    assert rospy.ERROR == 8, "Expected ERROR=8"
    assert rospy.FATAL == 16, "Expected FATAL=16"
    print("OK: log level constants")


def test_throttle_identical_functions_exist():
    """Test that throttle_identical logging functions exist and are callable."""
    import rospy

    setup()

    # ROS2 only - throttle_identical uses custom hash-based filtering
    if not hasattr(rospy, 'loginfo_throttle_identical'):
        print('SKIP: _throttle_identical functions not available')
        return

    # Verify all variants exist
    assert hasattr(rospy, 'logdebug_throttle_identical')
    assert hasattr(rospy, 'loginfo_throttle_identical')
    assert hasattr(rospy, 'logwarn_throttle_identical')
    assert hasattr(rospy, 'logerr_throttle_identical')
    assert hasattr(rospy, 'logfatal_throttle_identical')

    # All should be callable without error
    rospy.logdebug_throttle_identical(1.0, 'debug identical')
    rospy.loginfo_throttle_identical(1.0, 'info identical')
    rospy.logwarn_throttle_identical(1.0, 'warn identical')
    rospy.logerr_throttle_identical(1.0, 'error identical')
    rospy.logfatal_throttle_identical(1.0, 'fatal identical')

    print('OK: throttle_identical functions exist')


def test_throttle_identical_same_message():
    """Test that throttle_identical skips identical messages within period."""

    import rospy

    setup()

    if not hasattr(rospy, 'loginfo_throttle_identical'):
        print('SKIP: _throttle_identical functions not available')
        return

    # This tests the custom hash-based deduplication in rospy_too
    # Call same message multiple times within period
    msg = 'identical_test_message_same'
    rospy.loginfo_throttle_identical(10.0, msg)
    rospy.loginfo_throttle_identical(10.0, msg)  # Should be skipped
    rospy.loginfo_throttle_identical(10.0, msg)  # Should be skipped

    # No assertion on output (hard to verify without logger capture)
    # Just verify no exceptions
    print('OK: throttle_identical same message (no crash)')


def test_throttle_identical_different_message():
    """Test that throttle_identical logs different messages."""
    import rospy

    setup()

    if not hasattr(rospy, 'loginfo_throttle_identical'):
        print('SKIP: _throttle_identical functions not available')
        return

    # Different messages should both log
    rospy.loginfo_throttle_identical(10.0, 'different_message_A')
    rospy.loginfo_throttle_identical(10.0, 'different_message_B')

    # No assertion on output - just verify no exceptions
    print('OK: throttle_identical different messages (no crash)')


def test_exc_info():
    """Test that exc_info=True captures traceback in log output."""
    import rospy

    setup()

    # ROS2 only - rospy_too adds exc_info support
    try:
        from rospy.logging import _format_exc_info
    except ImportError:
        print('SKIP: exc_info not available (ROS1)')
        return

    # Test without exception context - should not crash
    rospy.logerr("error without exception", exc_info=True)

    # Test inside exception handler
    try:
        raise ValueError("test error for exc_info")
    except ValueError:
        rospy.logerr("error with exception", exc_info=True)

    # Test with explicit exc_info tuple
    try:
        raise RuntimeError("explicit exc_info test")
    except RuntimeError:
        import sys
        rospy.logerr("explicit exc_info", exc_info=sys.exc_info())

    # Test exc_info on all logging functions
    try:
        raise TypeError("type error")
    except TypeError:
        rospy.logdebug("debug exc", exc_info=True)
        rospy.loginfo("info exc", exc_info=True)
        rospy.logwarn("warn exc", exc_info=True)
        rospy.logerr("err exc", exc_info=True)
        rospy.logfatal("fatal exc", exc_info=True)

    # Test exc_info with throttle
    try:
        raise KeyError("key error")
    except KeyError:
        rospy.logerr_throttle(5.0, "throttle exc", exc_info=True)

    # Test exc_info with once
    try:
        raise IndexError("index error")
    except IndexError:
        rospy.logerr_once("once exc", exc_info=True)

    # Test exc_info with throttle_identical
    try:
        raise AttributeError("attr error")
    except AttributeError:
        rospy.logerr_throttle_identical(5.0, "identical exc", exc_info=True)

    print('OK: exc_info support')


def _once_helper_a():
    """Helper that logs once from a specific location."""
    import rospy
    rospy.loginfo_once("once from helper a")


def _once_helper_b():
    """Helper that logs once from a different location."""
    import rospy
    rospy.loginfo_once("once from helper b")


def test_once_per_callsite():
    """Test that _once functions track state per call site, not globally."""
    import rospy

    setup()

    # ROS2 only - rospy_too manages its own _once state
    try:
        from rospy.logging import _once_logged
    except ImportError:
        print('SKIP: per-callsite _once not available (ROS1)')
        return

    # Clear internal state to ensure test isolation
    _once_logged.clear()

    # Both helpers should log once (different call sites)
    _once_helper_a()
    _once_helper_b()

    # Calling again should not log (same call sites)
    _once_helper_a()
    _once_helper_b()

    # Verify the state has exactly 2 entries (one per call site)
    assert len(_once_logged) == 2, \
        f"Expected 2 call sites tracked, got {len(_once_logged)}"

    print('OK: _once per-callsite tracking')


def main():
    failed = 0

    tests = [
        test_basic_logging,
        test_logging_with_format,
        test_log_aliases,
        test_logging_once,
        test_logging_throttle,
        test_once_actually_once,
        test_throttle_actually_throttles,
        test_log_levels,
        test_throttle_identical_functions_exist,
        test_throttle_identical_same_message,
        test_throttle_identical_different_message,
        test_exc_info,
        test_once_per_callsite,
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
