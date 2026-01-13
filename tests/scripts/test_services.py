#!/usr/bin/env python3
# Test service functionality
# These tests require a ROS node to be initialized and service definitions.
# NOTE: This test uses std_srvs/SetBool which is available in both ROS 1 and ROS 2.

import sys
import threading
import time


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node("test_services", anonymous=True)


def test_service_creation():
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    def handler(req):
        return SetBoolResponse(success=True, message="OK")

    srv = rospy.Service("/test/service_creation", SetBool, handler)  # noqa: F841
    assert srv is not None, "Failed to create service"
    print("OK: service creation")


def test_service_proxy():
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    # Create service
    def handler(req):
        return SetBoolResponse(success=req.data, message="echo: %s" % req.data)

    srv = rospy.Service("/test/service_proxy", SetBool, handler)  # noqa: F841
    time.sleep(0.3)  # Give time for service to register

    # Create proxy and call
    proxy = rospy.ServiceProxy("/test/service_proxy", SetBool)

    # Wait for service
    rospy.wait_for_service("/test/service_proxy", timeout=5.0)

    # Call service
    resp = proxy(True)
    assert resp.success is True, "Expected success=True, got %s" % resp.success
    assert "True" in resp.message, "Expected True in message, got %s" % resp.message
    print("OK: service proxy")


def test_service_call_args():
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    # Create service
    def handler(req):
        return SetBoolResponse(success=not req.data, message="inverted")

    srv = rospy.Service("/test/service_args", SetBool, handler)  # noqa: F841
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/service_args", SetBool)
    rospy.wait_for_service("/test/service_args", timeout=5.0)

    # Call with keyword arg
    resp = proxy(data=False)
    assert resp.success is True, (
        "Expected success=True (inverted), got %s" % resp.success
    )
    print("OK: service call with keyword args")


def test_wait_for_service():
    import uuid

    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    # Use unique service name to avoid collision with other tests
    service_name = "/test/delayed_service_%s" % uuid.uuid4().hex[:8]
    srv_holder = [None]

    # Start service in background after delay
    def delayed_start():
        time.sleep(0.5)

        def handler(req):
            return SetBoolResponse(success=True, message="delayed")

        srv_holder[0] = rospy.Service(service_name, SetBool, handler)

    srv_thread = threading.Thread(target=delayed_start)
    srv_thread.start()

    # Wait for service (should succeed after delay)
    start = time.time()
    rospy.wait_for_service(service_name, timeout=5.0)
    elapsed = time.time() - start

    assert elapsed >= 0.4, "Expected wait of at least 0.4s, got %s" % elapsed
    print("OK: wait_for_service (waited %.2fs)" % elapsed)

    srv_thread.join()

    # Clean up
    if srv_holder[0]:
        srv_holder[0].shutdown()


def test_persistent_uses_ros_logging():
    import warnings

    import rospy
    from std_srvs.srv import SetBool

    setup()

    try:
        from rospy.logging import _get_logger
        _get_logger()
    except (ImportError, AttributeError):
        print('SKIP: persistent_uses_ros_logging (ROS1)')
        return

    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter('always')
        rospy.ServiceProxy('/test/persistent_warn', SetBool, persistent=True)
        assert len(w) == 0, 'Should not use Python warnings.warn()'

    print('OK: persistent=True uses ROS logging')


def test_service_shutdown():
    """Test Service.shutdown() destroys the service"""
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    def handler(req):
        return SetBoolResponse(success=True, message="OK")

    srv = rospy.Service("/test/service_shutdown", SetBool, handler)
    time.sleep(0.3)

    # Check internal state (rospy_too uses _service, ROS1 uses different internal)
    if hasattr(srv, '_service'):
        # rospy_too
        assert srv._service is not None, "Service should be initialized"
        srv.shutdown()
        assert srv._service is None, "Service should be None after shutdown"
    else:
        # ROS1 - just verify shutdown doesn't crash
        srv.shutdown()

    print("OK: Service.shutdown()")


def test_service_proxy_close():
    """Test ServiceProxy.close() is a no-op for compatibility"""
    import rospy
    from std_srvs.srv import SetBool

    setup()

    proxy = rospy.ServiceProxy("/test/nonexistent", SetBool)

    # close() should not raise
    proxy.close()

    # Should still be usable (close is a no-op)
    assert proxy is not None, "Proxy should still exist after close()"
    print("OK: ServiceProxy.close() (no-op)")


def test_wait_for_service_timeout():
    """Test wait_for_service raises on timeout for non-existent service"""
    import rospy

    setup()

    start = time.time()
    try:
        rospy.wait_for_service("/test/nonexistent_service", timeout=0.5)
        raise AssertionError("Expected exception for timeout")
    except Exception as e:
        elapsed = time.time() - start
        # Should have waited approximately timeout duration
        assert elapsed >= 0.4, "Expected ~0.5s wait, got %.2fs" % elapsed
        assert "timeout" in str(e).lower() or "Timeout" in str(e), (
            "Expected timeout exception, got: %s" % e
        )

    print("OK: wait_for_service timeout")


def test_service_call_timeout():
    """Test ServiceProxy call timeout parameter exists and is accepted"""
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    # ROS2 only - service call timeout
    try:
        from rospy.exceptions import ServiceException  # noqa: F401
    except ImportError:
        print("SKIP: test_service_call_timeout (ROS1)")
        return

    # Note: True timeout behavior requires the service to be in a separate node
    # or use a multi-threaded executor. In single-node single-threaded mode,
    # the service handler blocks the executor so timeout can't be checked.
    # Here we just verify the timeout parameter is accepted.

    # Create a fast service to verify timeout param is accepted
    def fast_handler(req):
        return SetBoolResponse(success=True, message="fast")

    srv = rospy.Service("/test/timeout_param_service", SetBool, fast_handler)
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/timeout_param_service", SetBool)
    rospy.wait_for_service("/test/timeout_param_service", timeout=5.0)

    # Call with timeout parameter - should complete normally
    resp = proxy(True, timeout=5.0)
    assert resp.success is True, "Expected success"

    srv.shutdown()
    print("OK: ServiceProxy accepts timeout parameter")


def test_service_handler_tuple_return():
    """Test service handler can return tuple instead of Response (ROS1 pattern)"""
    import rospy
    from std_srvs.srv import SetBool

    setup()

    # ROS2 only test
    try:
        from rospy.services import _get_response_fields  # noqa: F401
    except ImportError:
        print("SKIP: test_service_handler_tuple_return (ROS1)")
        return

    # Handler returns tuple (success, message) instead of SetBoolResponse
    def tuple_handler(req):
        return (req.data, "tuple response")

    srv = rospy.Service("/test/tuple_handler", SetBool, tuple_handler)
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/tuple_handler", SetBool)
    rospy.wait_for_service("/test/tuple_handler", timeout=5.0)

    resp = proxy(True)
    assert resp.success is True, "Expected success=True from tuple handler"
    assert resp.message == "tuple response", "Expected message from tuple handler"

    srv.shutdown()
    print("OK: service handler tuple return")


def test_service_handler_dict_return():
    """Test service handler can return dict instead of Response (ROS1 pattern)"""
    import rospy
    from std_srvs.srv import SetBool

    setup()

    # ROS2 only test
    try:
        from rospy.services import _get_response_fields  # noqa: F401
    except ImportError:
        print("SKIP: test_service_handler_dict_return (ROS1)")
        return

    # Handler returns dict instead of SetBoolResponse
    def dict_handler(req):
        return {'success': not req.data, 'message': 'dict response'}

    srv = rospy.Service("/test/dict_handler", SetBool, dict_handler)
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/dict_handler", SetBool)
    rospy.wait_for_service("/test/dict_handler", timeout=5.0)

    resp = proxy(True)
    assert resp.success is False, "Expected success=False (inverted) from dict handler"
    assert resp.message == "dict response", "Expected message from dict handler"

    srv.shutdown()
    print("OK: service handler dict return")


def test_service_proxy_wait_for_service_timeout_raises():
    """Test ServiceProxy.wait_for_service raises ROSException on timeout"""
    import rospy

    setup()

    # ROS2 only test
    try:
        from rospy.exceptions import ROSException  # noqa: F401
    except ImportError:
        print("SKIP: test_service_proxy_wait_for_service_timeout_raises (ROS1)")
        return

    from std_srvs.srv import SetBool

    proxy = rospy.ServiceProxy("/test/nonexistent_proxy_wait", SetBool)

    start = time.time()
    try:
        proxy.wait_for_service(timeout=0.5)
        raise AssertionError("Expected ROSException for timeout")
    except rospy.ROSException as e:
        elapsed = time.time() - start
        assert elapsed >= 0.4, "Expected ~0.5s wait, got %.2fs" % elapsed
        assert "timeout" in str(e).lower(), "Expected timeout in message: %s" % e

    print("OK: ServiceProxy.wait_for_service timeout raises")


def test_service_handler_returns_none():
    """Test service handler returning None returns default response (ROS2 limitation)"""
    import rospy
    from std_srvs.srv import SetBool

    setup()

    # ROS2 only test
    try:
        from rospy.services import DEFAULT_SERVICE_TIMEOUT  # noqa: F401
    except ImportError:
        print("SKIP: test_service_handler_returns_none (ROS1)")
        return

    # Handler returns None - ROS2 logs error and returns default response
    # (ROS1 would raise ServiceException, but ROS2 can't propagate handler failures)
    def none_handler(req):
        return None

    srv = rospy.Service("/test/none_handler", SetBool, none_handler)
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/none_handler", SetBool)
    rospy.wait_for_service("/test/none_handler", timeout=5.0)

    # Call succeeds but returns default response (success=False, message='')
    resp = proxy(True)
    assert resp.success is False, "Expected default success=False"
    assert resp.message == '', "Expected default message=''"

    srv.shutdown()
    print("OK: service handler returns None returns default response")


def test_service_handler_tuple_wrong_length():
    """Test service handler tuple with wrong length returns default response"""
    import rospy
    from std_srvs.srv import SetBool

    setup()

    # ROS2 only test
    try:
        from rospy.services import DEFAULT_SERVICE_TIMEOUT  # noqa: F401
    except ImportError:
        print("SKIP: test_service_handler_tuple_wrong_length (ROS1)")
        return

    # Handler returns tuple with wrong length (SetBoolResponse has 2 fields)
    # ROS2 logs error and returns default response
    def short_handler(req):
        return (True,)  # Missing message field

    srv = rospy.Service("/test/short_handler", SetBool, short_handler)
    time.sleep(0.3)

    proxy = rospy.ServiceProxy("/test/short_handler", SetBool)
    rospy.wait_for_service("/test/short_handler", timeout=5.0)

    # Call succeeds but returns default response
    resp = proxy(True)
    assert resp.success is False, "Expected default success=False"
    assert resp.message == '', "Expected default message=''"

    srv.shutdown()
    print("OK: service handler tuple wrong length returns default response")


def test_service_double_shutdown():
    """Test Service.shutdown() can be called multiple times safely"""
    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    setup()

    # ROS2 only test
    try:
        from rospy.services import _get_response_fields  # noqa: F401
    except ImportError:
        print("SKIP: test_service_double_shutdown (ROS1)")
        return

    def handler(req):
        return SetBoolResponse(success=True, message="OK")

    srv = rospy.Service("/test/double_shutdown", SetBool, handler)
    time.sleep(0.3)

    # First shutdown
    srv.shutdown()

    # Second shutdown should be a no-op, not raise
    srv.shutdown()

    # Third shutdown also safe
    srv.shutdown()

    print("OK: Service double shutdown")


def main():
    failed = 0

    tests = [
        test_service_creation,
        test_service_proxy,
        test_service_call_args,
        test_wait_for_service,
        test_persistent_uses_ros_logging,
        test_service_shutdown,
        test_service_proxy_close,
        test_wait_for_service_timeout,
        test_service_call_timeout,
        test_service_handler_tuple_return,
        test_service_handler_dict_return,
        test_service_proxy_wait_for_service_timeout_raises,
        test_service_handler_returns_none,
        test_service_handler_tuple_wrong_length,
        test_service_double_shutdown,
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
