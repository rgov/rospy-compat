#!/usr/bin/env python3
# Test mixed rospy/rclpy usage
# These tests verify that rospy_too works when rclpy is also being used.
# ROS2 only - skip on noetic.

import sys


def is_ros2():
    """Check if we're running on ROS2."""
    import importlib.util
    return importlib.util.find_spec('rclpy') is not None


def test_rclpy_create_node_hook():
    """Test that rclpy.create_node() is captured by rospy."""
    if not is_ros2():
        print('SKIP: test_rclpy_create_node_hook (ROS1)')
        return

    import rclpy
    import rospy

    # Initialize rclpy if needed
    if not rclpy.ok():
        rclpy.init()

    # Create node via rclpy (not rospy.init_node)
    node = rclpy.create_node('test_rclpy_node')

    # rospy should now be able to use this node
    assert rospy.core.is_initialized(), 'rospy should be initialized after rclpy.create_node()'

    # Node name should be available
    name = rospy.get_name()
    assert 'test_rclpy_node' in name, 'Expected test_rclpy_node in name, got %s' % name

    node.destroy_node()
    print('OK: rclpy.create_node() captured by rospy')


def test_rospy_logging_after_rclpy_node():
    """Test that rospy logging works with rclpy-created node."""
    if not is_ros2():
        print('SKIP: test_rospy_logging_after_rclpy_node (ROS1)')
        return

    import rclpy
    import rospy

    if not rclpy.ok():
        rclpy.init()

    # Use existing node or create one
    if not rospy.core.is_initialized():
        rclpy.create_node('test_logging_node')

    # Logging should work
    rospy.loginfo('Test log message from mixed usage')
    rospy.logwarn('Test warning from mixed usage')

    print('OK: rospy logging works with rclpy node')


def test_rospy_params_after_rclpy_node():
    """Test that rospy params work with rclpy-created node."""
    if not is_ros2():
        print('SKIP: test_rospy_params_after_rclpy_node (ROS1)')
        return

    import rclpy
    import rospy

    if not rclpy.ok():
        rclpy.init()

    if not rospy.core.is_initialized():
        rclpy.create_node('test_params_node')

    # Params should work
    rospy.set_param('/mixed_usage_test', 'value')
    val = rospy.get_param('/mixed_usage_test')
    assert val == 'value', 'Expected value, got %s' % val

    print('OK: rospy params work with rclpy node')


def test_multiple_create_node_warns():
    """Test that multiple rclpy.create_node() calls log a warning."""
    if not is_ros2():
        print('SKIP: test_multiple_create_node_warns (ROS1)')
        return

    import rclpy
    import rospy

    if not rclpy.ok():
        rclpy.init()

    # First node (or use existing)
    if not rospy.core.is_initialized():
        rclpy.create_node('first_node')

    # Second node - should log warning (we can't easily capture it, just verify no crash)
    node2 = rclpy.create_node('second_node')

    # rospy should still use the first node
    assert rospy.core.is_initialized(), 'rospy should still be initialized'

    node2.destroy_node()
    print('OK: multiple create_node() handled')


def main():
    failed = 0

    tests = [
        test_rclpy_create_node_hook,
        test_rospy_logging_after_rclpy_node,
        test_rospy_params_after_rclpy_node,
        test_multiple_create_node_warns,
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
