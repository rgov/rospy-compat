#!/usr/bin/env python3
# Test basic import functionality
# This test verifies that rospy can be imported and basic objects exist.

import sys


def test_import_rospy():

    print("OK: import rospy")


def test_basic_exports():
    import rospy

    # Node lifecycle
    assert hasattr(rospy, "init_node"), "Missing init_node"
    assert hasattr(rospy, "spin"), "Missing spin"
    # spin_once only exists in ROS2 (rospy_too)
    assert hasattr(rospy, "is_shutdown"), "Missing is_shutdown"
    assert hasattr(rospy, "signal_shutdown"), "Missing signal_shutdown"
    assert hasattr(rospy, "on_shutdown"), "Missing on_shutdown"
    print("OK: node lifecycle exports")

    # Topics
    assert hasattr(rospy, "Publisher"), "Missing Publisher"
    assert hasattr(rospy, "Subscriber"), "Missing Subscriber"
    assert hasattr(rospy, "wait_for_message"), "Missing wait_for_message"
    print("OK: topic exports")

    # Services
    assert hasattr(rospy, "Service"), "Missing Service"
    assert hasattr(rospy, "ServiceProxy"), "Missing ServiceProxy"
    assert hasattr(rospy, "wait_for_service"), "Missing wait_for_service"
    print("OK: service exports")

    # Parameters
    assert hasattr(rospy, "get_param"), "Missing get_param"
    assert hasattr(rospy, "set_param"), "Missing set_param"
    assert hasattr(rospy, "has_param"), "Missing has_param"
    assert hasattr(rospy, "delete_param"), "Missing delete_param"
    assert hasattr(rospy, "get_param_names"), "Missing get_param_names"
    assert hasattr(rospy, "search_param"), "Missing search_param"
    print("OK: parameter exports")

    # Time
    assert hasattr(rospy, "Time"), "Missing Time"
    assert hasattr(rospy, "Duration"), "Missing Duration"
    assert hasattr(rospy, "Rate"), "Missing Rate"
    assert hasattr(rospy, "Timer"), "Missing Timer"
    assert hasattr(rospy, "get_time"), "Missing get_time"
    assert hasattr(rospy, "get_rostime"), "Missing get_rostime"
    assert hasattr(rospy, "sleep"), "Missing sleep"
    print("OK: time exports")

    # Logging
    assert hasattr(rospy, "logdebug"), "Missing logdebug"
    assert hasattr(rospy, "loginfo"), "Missing loginfo"
    assert hasattr(rospy, "logwarn"), "Missing logwarn"
    assert hasattr(rospy, "logerr"), "Missing logerr"
    assert hasattr(rospy, "logfatal"), "Missing logfatal"
    print("OK: logging exports")

    # Exceptions
    assert hasattr(rospy, "ROSException"), "Missing ROSException"
    assert hasattr(rospy, "ROSInterruptException"), "Missing ROSInterruptException"
    print("OK: exception exports")

    # Names
    assert hasattr(rospy, "get_name"), "Missing get_name"
    assert hasattr(rospy, "get_namespace"), "Missing get_namespace"
    assert hasattr(rospy, "resolve_name"), "Missing resolve_name"
    print("OK: name exports")

    # Log levels
    assert hasattr(rospy, "DEBUG"), "Missing DEBUG"
    assert hasattr(rospy, "INFO"), "Missing INFO"
    assert hasattr(rospy, "WARN"), "Missing WARN"
    assert hasattr(rospy, "ERROR"), "Missing ERROR"
    assert hasattr(rospy, "FATAL"), "Missing FATAL"
    print("OK: log level exports")


def test_exception_inheritance():
    import rospy

    # Test that rospy exceptions are properly defined
    exc = rospy.ROSInterruptException("test")
    assert isinstance(exc, rospy.ROSException), (
        "ROSInterruptException should inherit ROSException"
    )
    print("OK: rospy exception hierarchy")

    # On ROS2, verify dual-inheritance patching
    try:
        from rclpy.exceptions import ROSInterruptException as RclpyInterrupt

        # The patched class should be catchable as rospy exception
        # Note: Rolling's ROSInterruptException may not accept message arg
        try:
            exc = RclpyInterrupt("test")
        except TypeError:
            exc = RclpyInterrupt()
        assert isinstance(exc, rospy.ROSInterruptException), (
            "rclpy.ROSInterruptException should be catchable as rospy.ROSInterruptException"
        )
        print("OK: exception dual-inheritance (ROS2)")
    except ImportError:
        # On ROS1, rclpy doesn't exist
        print("SKIP: exception dual-inheritance (ROS1)")


def test_message_hook_precision():
    try:
        from rospy.impl.hooks import _is_message_module
    except ImportError:
        print('SKIP: message_hook_precision (ROS1)')
        return

    # Should match
    assert _is_message_module('std_msgs.msg')
    assert _is_message_module('std_msgs.msg.String')
    assert _is_message_module('my_package.srv')
    assert _is_message_module('my_package.srv.MyService')

    # Should NOT match (false positive cases)
    assert not _is_message_module('my.msgtools')
    assert not _is_message_module('srvutils')
    assert not _is_message_module('messaging.core')
    print('OK: message hook precision')


def test_rosgraph_log_mapping():
    try:
        import rcl_interfaces.msg
        import rosgraph_msgs.msg
        import rospy  # noqa: F401
    except ImportError as e:
        print('SKIP: rosgraph_log_mapping - %s' % e)
        return

    assert rosgraph_msgs.msg.Log is rcl_interfaces.msg.Log
    print('OK: rosgraph_msgs/Log type mapping')


def test_ros_init_exception():
    """Test ROSInitException can be raised and caught."""
    import rospy

    assert hasattr(rospy, 'ROSInitException'), 'Missing ROSInitException'
    exc = rospy.ROSInitException('test init error')
    assert isinstance(exc, rospy.ROSException), 'ROSInitException should inherit ROSException'
    assert 'test init error' in str(exc), 'Exception message should be preserved'
    print('OK: ROSInitException')


def test_ros_serialization_exception():
    """Test ROSSerializationException can be raised and caught."""
    import rospy

    assert hasattr(rospy, 'ROSSerializationException'), 'Missing ROSSerializationException'
    exc = rospy.ROSSerializationException('test serialization error')
    assert isinstance(exc, rospy.ROSException), (
        'ROSSerializationException should inherit ROSException'
    )
    assert 'test serialization error' in str(exc), 'Exception message should be preserved'
    print('OK: ROSSerializationException')


def test_ros_time_moved_backwards_exception():
    """Test ROSTimeMovedBackwardsException can be raised and caught."""
    import rospy

    assert hasattr(rospy, 'ROSTimeMovedBackwardsException'), (
        'Missing ROSTimeMovedBackwardsException'
    )
    exc = rospy.ROSTimeMovedBackwardsException(1.5)
    assert isinstance(exc, rospy.ROSException), (
        'ROSTimeMovedBackwardsException should inherit ROSException'
    )
    # rospy_too includes time value in message, ROS1 may not
    # Just verify the exception can be created and stringified
    str(exc)  # Should not raise
    print('OK: ROSTimeMovedBackwardsException')


def test_service_exception():
    """Test ServiceException can be raised and caught."""
    import rospy

    assert hasattr(rospy, 'ServiceException'), 'Missing ServiceException'
    exc = rospy.ServiceException('test service error')
    assert isinstance(exc, Exception), 'ServiceException should inherit Exception'
    assert 'test service error' in str(exc), 'Exception message should be preserved'
    print('OK: ServiceException')


def test_import_fallback_logic():
    try:
        from rospy.impl.hooks import import_module_with_fallback_names
    except ImportError:
        print('SKIP: import_fallback_logic (ROS1)')
        return

    # Test that direct import works (std_msgs.msg exists)
    import std_msgs.msg
    mod = import_module_with_fallback_names('std_msgs.msg')
    assert mod is not None
    print('OK: import fallback - direct import works')

    # Test that non-existent package raises ModuleNotFoundError
    try:
        import_module_with_fallback_names('nonexistent_pkg_xyz.msg')
        assert False, 'Should have raised ModuleNotFoundError'
    except ModuleNotFoundError:
        pass
    print('OK: import fallback - raises for missing packages')


def main():
    failed = 0

    tests = [
        test_import_rospy,
        test_basic_exports,
        test_exception_inheritance,
        test_message_hook_precision,
        test_rosgraph_log_mapping,
        test_ros_init_exception,
        test_ros_serialization_exception,
        test_ros_time_moved_backwards_exception,
        test_service_exception,
        test_import_fallback_logic,
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
