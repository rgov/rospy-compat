#!/usr/bin/env python3
# Test node initialization and lifecycle
# These tests require ROS master (roscore for ROS 1).
#
# Note: Shutdown tests invalidate the rclpy context. They are skipped when
# running as part of run_all.py (detected via RUN_ALL_TESTS env var) to
# avoid breaking subsequent tests.

import os
import sys


def test_init_node():
    import rospy

    rospy.init_node("test_node", anonymous=True)
    print("OK: init_node")

    # Should be able to get node name
    name = rospy.get_name()
    assert name.startswith("/test_node"), (
        "Expected name to start with /test_node, got %s" % name
    )
    print("OK: get_name returns %s" % name)


def test_anonymous_node_name():
    # This test checks that anonymous=True creates unique names
    # The name should contain pid and random suffix
    import rospy

    name = rospy.get_name()
    # Anonymous name format: /test_node_<pid>_<hex>
    parts = name.split("_")
    assert len(parts) >= 3, "Expected anonymous name format, got %s" % name
    print("OK: anonymous node name format")


def test_get_namespace():
    import rospy

    ns = rospy.get_namespace()
    assert ns == "/", "Expected /, got %s" % ns
    print("OK: get_namespace returns %s" % ns)


def test_resolve_name():
    import rospy

    # Absolute name
    resolved = rospy.resolve_name("/foo/bar")
    assert resolved == "/foo/bar", "Expected /foo/bar, got %s" % resolved
    print("OK: resolve_name absolute")

    # Relative name (should be resolved relative to namespace)
    resolved = rospy.resolve_name("baz")
    assert resolved == "/baz", "Expected /baz, got %s" % resolved
    print("OK: resolve_name relative")


def test_is_shutdown():
    import rospy

    assert not rospy.is_shutdown(), "Expected is_shutdown to be False"
    print("OK: is_shutdown returns False")


def test_get_time():
    import rospy

    t = rospy.get_time()
    assert t > 0, "Expected positive time, got %s" % t
    print("OK: get_time returns %s" % t)


def test_get_rostime():
    import rospy

    t = rospy.get_rostime()
    assert isinstance(t, rospy.Time), "Expected Time instance"
    assert t.secs > 0, "Expected positive secs"
    print("OK: get_rostime returns %s" % t)


def test_resolve_name_private():
    """Verify resolve_name handles private names (~) correctly."""
    import rospy

    node_name = rospy.get_name()
    resolved = rospy.resolve_name("~private_topic")
    # Should resolve to /nodename/private_topic (or /ns/nodename/private_topic if in namespace)
    expected = node_name + "/private_topic"
    assert resolved == expected, (
        "Expected %s, got %s" % (expected, resolved)
    )
    print("OK: resolve_name private (~)")


def test_resolve_name_with_caller_id():
    """Verify resolve_name uses caller_id when provided."""
    import rospy

    # Test relative name with caller_id in different namespace
    resolved = rospy.resolve_name("topic", caller_id="/other_ns/other_node")
    # Should resolve relative to /other_ns, not current node's namespace
    assert resolved == "/other_ns/topic", (
        "Expected /other_ns/topic, got %s" % resolved
    )
    print("OK: resolve_name with caller_id (relative)")

    # Test private name with caller_id
    resolved = rospy.resolve_name("~priv", caller_id="/other_ns/other_node")
    # Should resolve relative to caller_id
    assert resolved == "/other_ns/other_node/priv", (
        "Expected /other_ns/other_node/priv, got %s" % resolved
    )
    print("OK: resolve_name with caller_id (private)")


def test_resolve_name_normalization():
    """Verify resolve_name handles ROS1 name edge cases."""
    import rospy

    # Test ~/foo quirk: rospy resolves ~/foo to /foo (bug in ns_join)
    resolved = rospy.resolve_name("~/foo")
    assert resolved == "/foo", (
        "Expected /foo (rospy quirk), got %s" % resolved
    )
    print("OK: resolve_name ~/foo quirk")

    # Test double slash normalization: foo//bar -> foo/bar
    resolved = rospy.resolve_name("foo//bar")
    assert resolved == "/foo/bar", (
        "Expected /foo/bar, got %s" % resolved
    )
    print("OK: resolve_name double slash normalization")

    # Test trailing slash normalization: topic/ -> topic
    resolved = rospy.resolve_name("topic/")
    assert resolved == "/topic", (
        "Expected /topic, got %s" % resolved
    )
    print("OK: resolve_name trailing slash normalization")


def test_myargv_filters_remappings():
    # myargv strips ROS-specific arguments from argv
    import rospy

    # Check if we're on ROS2 (rospy_too has rclpy)
    try:
        import rclpy
        is_ros2 = True
    except ImportError:
        is_ros2 = False

    if is_ros2:
        # ROS2 style: --ros-args block should be stripped
        test_argv = [
            'node_name', 'arg1',
            '--ros-args', '-r', '__node:=test', '-p', 'param:=value',
            '--', 'arg2'
        ]
        result = rospy.myargv(test_argv)
        assert '--ros-args' not in result
        assert '__node:=test' not in result
        assert 'param:=value' not in result
        assert result == ['node_name', 'arg1', 'arg2'], 'got %s' % result
        print('OK: myargv strips --ros-args blocks')

        # ROS1-style := args are also stripped (rclpy treats them as deprecated remaps)
        test_argv = ['node_name', 'custom:=value', 'arg1']
        result = rospy.myargv(test_argv)
        assert 'custom:=value' not in result, 'rclpy strips deprecated := args'
        assert result == ['node_name', 'arg1'], 'got %s' % result
        print('OK: myargv strips deprecated := args')
    else:
        # ROS1: myargv strips := remapping args
        test_argv = ['node_name', '__name:=test', 'arg1']
        result = rospy.myargv(test_argv)
        assert '__name:=test' not in result, 'ROS1 strips := remaps'
        assert result == ['node_name', 'arg1'], 'got %s' % result
        print('OK: myargv strips ROS1 remapping args')


def test_get_caller_id():
    """Verify get_caller_id returns same as get_name."""
    import rospy

    name = rospy.get_name()
    caller_id = rospy.get_caller_id()
    assert caller_id == name, "Expected get_caller_id() == get_name(), got %s != %s" % (caller_id, name)
    print("OK: get_caller_id returns %s" % caller_id)


def test_get_node_uri():
    """Verify get_node_uri returns a URI string."""
    import rospy

    uri = rospy.get_node_uri()
    assert uri is not None, "Expected non-None URI"
    assert isinstance(uri, str), "Expected string URI, got %s" % type(uri)
    # ROS1 returns something like http://hostname:port/
    # rospy_too returns http://localhost:0/nodename
    assert "://" in uri, "Expected URI format with ://, got %s" % uri
    print("OK: get_node_uri returns %s" % uri)


def test_on_shutdown_hook():
    """Verify on_shutdown registers callback that fires during shutdown."""
    # Skip in full test run - shutdown invalidates context for later tests
    if os.environ.get('RUN_ALL_TESTS'):
        print('SKIP: on_shutdown hook (would break subsequent tests)')
        return

    import rospy

    # Register a hook that will be verified during test_spin_with_shutdown
    # ROS1 on_shutdown callbacks take zero arguments
    global _shutdown_hook_called
    _shutdown_hook_called = False

    def shutdown_hook():
        global _shutdown_hook_called
        _shutdown_hook_called = True

    rospy.on_shutdown(shutdown_hook)
    print("OK: on_shutdown hook registered")


def test_add_preshutdown_hook():
    """Verify add_preshutdown_hook works (alias for on_shutdown)."""
    # Skip in full test run - shutdown invalidates context for later tests
    if os.environ.get('RUN_ALL_TESTS'):
        print('SKIP: add_preshutdown_hook (would break subsequent tests)')
        return

    import rospy

    global _preshutdown_hook_called
    _preshutdown_hook_called = False

    def preshutdown_hook(reason):
        global _preshutdown_hook_called
        _preshutdown_hook_called = True

    # add_preshutdown_hook is an alias for on_shutdown
    if hasattr(rospy, 'add_preshutdown_hook'):
        rospy.add_preshutdown_hook(preshutdown_hook)
        print("OK: add_preshutdown_hook registered")
    else:
        print("SKIP: add_preshutdown_hook not available")


def test_spin_with_shutdown():
    """Verify spin exits when signal_shutdown is called and hooks fire."""
    # Skip in full test run - shutdown invalidates context for later tests
    if os.environ.get('RUN_ALL_TESTS'):
        print('SKIP: spin_with_shutdown (would break subsequent tests)')
        return

    import threading
    import time

    import rospy

    spin_exited = [False]

    def spin_thread():
        rospy.spin()
        spin_exited[0] = True

    t = threading.Thread(target=spin_thread)
    t.start()

    # Give spin a moment to start
    time.sleep(0.2)
    assert not spin_exited[0], "spin should not have exited yet"

    # Signal shutdown
    rospy.signal_shutdown("test shutdown")

    # Wait for spin to exit
    t.join(timeout=2.0)
    assert spin_exited[0], "spin should have exited after signal_shutdown"
    print("OK: spin exits on signal_shutdown")

    # Verify shutdown hooks were called (registered in earlier tests)
    global _shutdown_hook_called, _preshutdown_hook_called
    if '_shutdown_hook_called' in globals():
        assert _shutdown_hook_called, "on_shutdown hook should have been called"
        print("OK: on_shutdown hook was called")
    if '_preshutdown_hook_called' in globals() and hasattr(rospy, 'add_preshutdown_hook'):
        assert _preshutdown_hook_called, "add_preshutdown_hook should have been called"
        print("OK: add_preshutdown_hook was called")


def main():
    failed = 0

    # These tests must run in order (init_node first, spin_with_shutdown last)
    tests = [
        test_init_node,
        test_anonymous_node_name,
        test_get_namespace,
        test_resolve_name,
        test_resolve_name_private,
        test_resolve_name_with_caller_id,
        test_resolve_name_normalization,
        test_myargv_filters_remappings,
        test_is_shutdown,
        test_get_time,
        test_get_rostime,
        test_get_caller_id,
        test_get_node_uri,
        # Register shutdown hooks (tested during spin_with_shutdown)
        test_on_shutdown_hook,
        test_add_preshutdown_hook,
        # spin_with_shutdown must be last as it shuts down the node
        test_spin_with_shutdown,
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
