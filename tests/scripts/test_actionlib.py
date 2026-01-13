#!/usr/bin/env python3
"""Test actionlib compatibility layer.

These tests run on both ROS1 (native actionlib) and ROS2 (rospy_too actionlib).
Import paths are identical between platforms.
"""

import sys
import threading
import time


def is_ros2():
    """Check if we're running on ROS2."""
    try:
        import rclpy
        return True
    except ImportError:
        return False


def setup():
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node("test_actionlib", anonymous=True)


def get_test_action_info():
    """Get action type and helpers for the current platform.

    Returns:
        tuple: (ActionType, make_goal, get_result_value) or None if unavailable
    """
    if is_ros2():
        try:
            from example_interfaces.action import Fibonacci
            return (
                Fibonacci,
                lambda n: Fibonacci.Goal(order=n),
                lambda r: r.sequence,
            )
        except (ImportError, AttributeError) as e:
            print("Note: Could not import Fibonacci action: %s" % e)
            return None
    else:
        try:
            from actionlib.msg import TestAction, TestGoal
            return (
                TestAction,
                lambda n: TestGoal(goal=n),
                lambda r: r.result,
            )
        except ImportError as e:
            print("Note: Could not import TestAction: %s" % e)
            return None


def test_goal_status_constants():
    """Test GoalStatus constants are defined."""
    from actionlib_msgs.msg import GoalStatus

    assert GoalStatus.PENDING == 0
    assert GoalStatus.ACTIVE == 1
    assert GoalStatus.PREEMPTED == 2
    assert GoalStatus.SUCCEEDED == 3
    assert GoalStatus.ABORTED == 4
    assert GoalStatus.REJECTED == 5
    assert GoalStatus.LOST == 9

    # to_string may not exist on ROS1's actionlib_msgs.msg.GoalStatus
    if hasattr(GoalStatus, 'to_string'):
        assert GoalStatus.to_string(GoalStatus.SUCCEEDED) == "SUCCEEDED"
    print("OK: GoalStatus constants")


def test_simple_goal_state_constants():
    """Test SimpleGoalState constants are defined."""
    from actionlib import SimpleGoalState

    assert SimpleGoalState.PENDING == 0
    assert SimpleGoalState.ACTIVE == 1
    assert SimpleGoalState.DONE == 2

    if hasattr(SimpleGoalState, 'to_string'):
        assert SimpleGoalState.to_string(SimpleGoalState.DONE) == "DONE"
    print("OK: SimpleGoalState constants")


def test_status_mapping():
    """Test ROS2 to ROS1 status mapping (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_status_mapping (ROS1)")
        return

    from actionlib.goal_status import ros2_status_to_ros1, GoalStatus
    from action_msgs.msg import GoalStatus as ROS2GoalStatus

    assert ros2_status_to_ros1(ROS2GoalStatus.STATUS_ACCEPTED) == GoalStatus.PENDING
    assert ros2_status_to_ros1(ROS2GoalStatus.STATUS_EXECUTING) == GoalStatus.ACTIVE
    assert ros2_status_to_ros1(ROS2GoalStatus.STATUS_SUCCEEDED) == GoalStatus.SUCCEEDED
    assert ros2_status_to_ros1(ROS2GoalStatus.STATUS_CANCELED) == GoalStatus.PREEMPTED
    assert ros2_status_to_ros1(ROS2GoalStatus.STATUS_ABORTED) == GoalStatus.ABORTED
    print("OK: status mapping")


def test_simple_action_client_creation():
    """Test SimpleActionClient can be created."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_simple_action_client_creation (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    from actionlib import SimpleActionClient

    client = SimpleActionClient("test_action", ActionType)
    assert client is not None
    print("OK: SimpleActionClient creation")


def test_simple_action_server_creation():
    """Test SimpleActionServer can be created."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_simple_action_server_creation (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    from actionlib import SimpleActionServer

    def execute_cb(goal):
        pass

    server = SimpleActionServer("test_server", ActionType, execute_cb=execute_cb, auto_start=False)
    assert server is not None
    print("OK: SimpleActionServer creation")


def test_client_server_interaction():
    """Test basic client-server interaction."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_client_server_interaction (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    results = {"feedback_count": 0, "done": False, "result": None}

    def execute_cb(goal):
        # Publish feedback
        if is_ros2():
            server.publish_feedback(ActionType.Feedback(sequence=[0, 1]))
        else:
            from actionlib.msg import TestFeedback
            server.publish_feedback(TestFeedback(feedback=1))
        results["feedback_count"] += 1

        if server.is_preempt_requested():
            server.set_preempted()
            return

        # Compute result
        if is_ros2():
            result = ActionType.Result()
            result.sequence = [0, 1, 1, 2, 3]
        else:
            from actionlib.msg import TestResult
            result = TestResult(result=42)
        server.set_succeeded(result)

    # Create server
    server = SimpleActionServer("fibonacci", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)  # Let server start

    # Create client
    client = SimpleActionClient("fibonacci", ActionType)

    # Wait for server
    found = client.wait_for_server(timeout=rospy.Duration(5.0))
    assert found, "Server not found"
    print("OK: client found server")

    # Send goal with callbacks
    def done_cb(status, result):
        results["done"] = True
        results["result"] = result
        results["status"] = status

    def feedback_cb(feedback):
        results["feedback_count"] += 1

    goal = make_goal(5)
    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    # Wait for result
    finished = client.wait_for_result(timeout=rospy.Duration(5.0))
    assert finished, "Goal did not finish"

    state = client.get_state()
    assert state == GoalStatus.SUCCEEDED, "Expected SUCCEEDED, got %s" % state

    result = client.get_result()
    assert result is not None, "Expected result"

    print("OK: client-server interaction (state=%s)" % state)


def test_send_goal_and_wait():
    """Test send_goal_and_wait convenience method."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_send_goal_and_wait (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    def execute_cb(goal):
        if is_ros2():
            result = ActionType.Result()
            result.sequence = [0, 1, 1]
        else:
            from actionlib.msg import TestResult
            result = TestResult(result=123)
        server.set_succeeded(result)

    server = SimpleActionServer("fib_wait", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("fib_wait", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    goal = make_goal(3)
    state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5.0))

    assert state == GoalStatus.SUCCEEDED, "Expected SUCCEEDED, got %s" % state
    print("OK: send_goal_and_wait")


def test_cancel_goal():
    """Test goal cancellation."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_cancel_goal (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    executing = threading.Event()
    preempt_called = [False]

    def execute_cb(goal):
        executing.set()
        # Wait for preempt
        while not server.is_preempt_requested() and not rospy.is_shutdown():
            time.sleep(0.05)
        preempt_called[0] = True
        server.set_preempted()

    server = SimpleActionServer("fib_cancel", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("fib_cancel", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    goal = make_goal(100)
    client.send_goal(goal)

    # Wait for execution to start
    executing.wait(timeout=2.0)

    # Wait for goal to be accepted (client receives goal_handle)
    for _ in range(50):
        if client.get_state() == GoalStatus.ACTIVE:
            break
        time.sleep(0.1)

    # Cancel
    client.cancel_goal()

    # Wait for result
    finished = client.wait_for_result(timeout=rospy.Duration(5.0))
    assert finished, "Goal did not finish after cancel"

    state = client.get_state()
    assert state == GoalStatus.PREEMPTED, "Expected PREEMPTED, got %s" % state
    print("OK: cancel_goal")


def test_actionlib_msgs_classes():
    """Test GoalID and GoalStatusArray classes (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_actionlib_msgs_classes (ROS1)")
        return

    from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus

    # Test GoalID creation
    goal_id1 = GoalID()
    assert goal_id1.id == ''

    goal_id2 = GoalID(id='test_id')
    assert goal_id2.id == 'test_id'

    # Test GoalID __repr__
    repr_str = repr(goal_id2)
    assert 'GoalID' in repr_str
    assert 'test_id' in repr_str

    # Test GoalID __eq__
    goal_id3 = GoalID(id='test_id')
    goal_id3.stamp = goal_id2.stamp  # Ensure same stamp
    assert goal_id2 == goal_id3
    assert not (goal_id1 == goal_id2)
    assert not (goal_id1 == "not a GoalID")

    # Test GoalStatusArray creation
    status_array = GoalStatusArray()
    assert status_array.status_list == []

    status_array2 = GoalStatusArray(status_list=[1, 2, 3])
    assert status_array2.status_list == [1, 2, 3]

    # Test GoalStatusArray __repr__
    repr_str = repr(status_array2)
    assert 'GoalStatusArray' in repr_str

    print("OK: actionlib_msgs classes")


def test_action_utils():
    """Test action utility functions (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_action_utils (ROS1)")
        return

    from actionlib.action_utils import (
        get_action_type, get_goal_type, get_result_type, get_feedback_type,
        create_goal, create_result, create_feedback
    )
    from example_interfaces.action import Fibonacci

    # Test get_action_type with valid action
    action_type = get_action_type(Fibonacci)
    assert action_type == Fibonacci

    # Test get_goal/result/feedback_type
    assert get_goal_type(Fibonacci) == Fibonacci.Goal
    assert get_result_type(Fibonacci) == Fibonacci.Result
    assert get_feedback_type(Fibonacci) == Fibonacci.Feedback

    # Test create_goal/result/feedback
    goal = create_goal(Fibonacci, order=5)
    assert goal.order == 5

    result = create_result(Fibonacci)
    assert hasattr(result, 'sequence')

    feedback = create_feedback(Fibonacci)
    assert hasattr(feedback, 'sequence')

    # Test get_action_type error path
    try:
        get_action_type("not_an_action")
        assert False, "Expected ValueError"
    except ValueError as e:
        assert "Could not determine action type" in str(e)

    print("OK: action_utils")


def test_action_positional_args():
    """Test that action Goal/Result/Feedback support positional arguments (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_action_positional_args (ROS1)")
        return

    # Must import rospy first to install import hooks
    setup()

    # Import action AFTER rospy to get wrapped classes
    # Need to force reimport since it might have been imported earlier without hooks
    import sys
    if 'example_interfaces.action' in sys.modules:
        del sys.modules['example_interfaces.action']

    from example_interfaces.action import Fibonacci

    # Test Goal with positional arg
    goal = Fibonacci.Goal(5)
    assert goal.order == 5, "Goal positional arg failed: order=%s" % goal.order

    # Test Goal with keyword arg (should still work)
    goal2 = Fibonacci.Goal(order=10)
    assert goal2.order == 10

    # Test Result with positional arg
    result = Fibonacci.Result([1, 2, 3])
    assert list(result.sequence) == [1, 2, 3], "Result positional arg failed: sequence=%s" % list(result.sequence)

    # Test Feedback with positional arg
    feedback = Fibonacci.Feedback([0, 1, 1])
    assert list(feedback.sequence) == [0, 1, 1], "Feedback positional arg failed: sequence=%s" % list(feedback.sequence)

    print("OK: action_positional_args")


def test_server_error_paths():
    """Test server error paths when no active goal (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_server_error_paths (ROS1)")
        return

    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_server_error_paths (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    from actionlib import SimpleActionServer

    def execute_cb(goal):
        pass

    server = SimpleActionServer("test_errors", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()

    # These should log errors but not crash
    server.set_succeeded()  # No active goal
    server.set_aborted()    # No active goal
    server.set_preempted()  # No active goal
    server.publish_feedback(ActionType.Feedback())  # No active goal

    # accept_new_goal when no new goal
    result = server.accept_new_goal()
    assert result is None

    print("OK: server_error_paths")


def test_active_callback():
    """Test active_cb parameter in send_goal."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_active_callback (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer

    active_called = [False]

    def execute_cb(goal):
        if is_ros2():
            result = ActionType.Result()
            result.sequence = [0, 1]
        else:
            from actionlib.msg import TestResult
            result = TestResult(result=1)
        server.set_succeeded(result)

    server = SimpleActionServer("test_active_cb", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("test_active_cb", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    def active_cb():
        active_called[0] = True

    goal = make_goal(2)
    client.send_goal(goal, active_cb=active_cb)
    client.wait_for_result(timeout=rospy.Duration(5.0))

    assert active_called[0], "active_cb was not called"
    print("OK: active_callback")


def test_send_goal_and_wait_timeout():
    """Test send_goal_and_wait with timeout."""
    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_send_goal_and_wait_timeout (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    def execute_cb(goal):
        # Slow execution - wait for preempt or long timeout
        for _ in range(100):
            if server.is_preempt_requested():
                server.set_preempted()
                return
            time.sleep(0.1)
        if is_ros2():
            server.set_succeeded(ActionType.Result())
        else:
            from actionlib.msg import TestResult
            server.set_succeeded(TestResult(result=1))

    server = SimpleActionServer("test_timeout", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("test_timeout", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    goal = make_goal(100)
    # Short timeout should trigger cancel
    state = client.send_goal_and_wait(
        goal,
        execute_timeout=rospy.Duration(0.5),
        preempt_timeout=rospy.Duration(2.0)
    )

    assert state == GoalStatus.PREEMPTED, "Expected PREEMPTED after timeout, got %s" % state
    print("OK: send_goal_and_wait_timeout")


def test_execute_cb_exception():
    """Test exception handling in execute_cb (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_execute_cb_exception (ROS1)")
        return

    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_execute_cb_exception (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    def execute_cb(goal):
        raise RuntimeError("Test exception")

    server = SimpleActionServer("test_exception", ActionType, execute_cb=execute_cb, auto_start=False)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("test_exception", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    goal = make_goal(1)
    client.send_goal(goal)
    finished = client.wait_for_result(timeout=rospy.Duration(5.0))

    assert finished, "Goal did not finish"
    state = client.get_state()
    assert state == GoalStatus.ABORTED, "Expected ABORTED after exception, got %s" % state
    print("OK: execute_cb_exception")


def test_cancel_methods():
    """Test cancel_all_goals and cancel_goals_at_and_before_time (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_cancel_methods (ROS1)")
        return

    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_cancel_methods (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient

    client = SimpleActionClient("test_cancel_methods", ActionType)

    # These should log warnings but not crash
    client.cancel_all_goals()
    client.cancel_goals_at_and_before_time(rospy.Time.now())

    print("OK: cancel_methods")


def test_register_goal_callback_with_execute_cb():
    """Test register_goal_callback when execute_cb is set (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_register_goal_callback_with_execute_cb (ROS1)")
        return

    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_register_goal_callback_with_execute_cb (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    from actionlib import SimpleActionServer

    def execute_cb(goal):
        server.set_succeeded()

    server = SimpleActionServer("test_register_cb", ActionType, execute_cb=execute_cb, auto_start=False)

    # This should log a warning
    def goal_cb():
        pass

    server.register_goal_callback(goal_cb)

    print("OK: register_goal_callback_with_execute_cb")


def test_goal_callback_pattern():
    """Test goal_callback pattern (without execute_cb) (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_goal_callback_pattern (ROS1)")
        return

    setup()

    action_info = get_test_action_info()
    if action_info is None:
        print("SKIP: test_goal_callback_pattern (no test action available)")
        return

    ActionType, make_goal, get_result = action_info

    import rospy
    from actionlib import SimpleActionClient, SimpleActionServer
    from actionlib_msgs.msg import GoalStatus

    goal_received = threading.Event()
    preempt_received = threading.Event()

    # Create server WITHOUT execute_cb
    server = SimpleActionServer("test_goal_cb_pattern", ActionType, auto_start=False)

    def goal_callback():
        goal_received.set()
        # Accept the goal
        goal = server.accept_new_goal()
        if goal is not None:
            # Complete it
            server.set_succeeded(ActionType.Result())

    def preempt_callback():
        preempt_received.set()

    server.register_goal_callback(goal_callback)
    server.register_preempt_callback(preempt_callback)
    server.start()
    time.sleep(0.5)

    client = SimpleActionClient("test_goal_cb_pattern", ActionType)
    client.wait_for_server(timeout=rospy.Duration(5.0))

    goal = make_goal(3)
    client.send_goal(goal)

    finished = client.wait_for_result(timeout=rospy.Duration(5.0))
    assert finished, "Goal did not finish"
    assert goal_received.is_set(), "goal_callback was not called"

    state = client.get_state()
    assert state == GoalStatus.SUCCEEDED, "Expected SUCCEEDED, got %s" % state

    print("OK: goal_callback_pattern")


def test_ros1_action_import_from_msg():
    """Test ROS1-style action imports from package.msg (ROS2 only)."""
    if not is_ros2():
        print("SKIP: test_ros1_action_import_from_msg (ROS1)")
        return

    setup()

    # Clear cached module to force reimport with hooks
    import sys
    if 'example_interfaces.msg' in sys.modules:
        del sys.modules['example_interfaces.msg']

    # ROS1-style import from .msg
    from example_interfaces.msg import FibonacciAction, FibonacciGoal, FibonacciResult, FibonacciFeedback

    # Verify aliases work
    assert FibonacciAction is not None, "FibonacciAction not found"

    # Test positional arg support on Goal
    goal = FibonacciGoal(5)
    assert goal.order == 5, "FibonacciGoal positional arg failed"

    # Test keyword arg still works
    goal2 = FibonacciGoal(order=10)
    assert goal2.order == 10, "FibonacciGoal keyword arg failed"

    result = FibonacciResult()
    assert hasattr(result, 'sequence'), "FibonacciResult missing sequence"

    feedback = FibonacciFeedback()
    assert hasattr(feedback, 'sequence'), "FibonacciFeedback missing sequence"

    # Verify Action aliases map correctly
    from example_interfaces.action import Fibonacci
    assert FibonacciAction is Fibonacci, "FibonacciAction should be Fibonacci"
    assert FibonacciGoal is Fibonacci.Goal, "FibonacciGoal should be Fibonacci.Goal"
    assert FibonacciResult is Fibonacci.Result, "FibonacciResult should be Fibonacci.Result"
    assert FibonacciFeedback is Fibonacci.Feedback, "FibonacciFeedback should be Fibonacci.Feedback"

    print("OK: ROS1-style action import from msg")


def main():
    failed = 0

    tests = [
        test_goal_status_constants,
        test_simple_goal_state_constants,
        test_status_mapping,
        test_simple_action_client_creation,
        test_simple_action_server_creation,
        test_client_server_interaction,
        test_send_goal_and_wait,
        test_cancel_goal,
        # Coverage improvement tests
        test_actionlib_msgs_classes,
        test_action_utils,
        test_action_positional_args,
        test_server_error_paths,
        test_active_callback,
        test_send_goal_and_wait_timeout,
        test_execute_cb_exception,
        test_cancel_methods,
        test_register_goal_callback_with_execute_cb,
        test_goal_callback_pattern,
        test_ros1_action_import_from_msg,
    ]

    for test in tests:
        try:
            test()
        except Exception as e:
            print("FAIL: %s - %s" % (test.__name__, e))
            import traceback
            traceback.print_exc()
            failed += 1

    if failed:
        print("\n%d test(s) FAILED" % failed)
        sys.exit(1)
    else:
        print("\nAll tests PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()
