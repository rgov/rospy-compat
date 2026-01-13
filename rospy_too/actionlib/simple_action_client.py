"""SimpleActionClient implementation wrapping rclpy.action.ActionClient."""

import threading

import rospy
from rclpy.action import ActionClient as ROS2ActionClient

from actionlib.goal_status import GoalStatus, SimpleGoalState, ros2_status_to_ros1
from actionlib.action_utils import get_action_type


class SimpleActionClient:
    """
    A simple client for interacting with an action server.

    This wraps rclpy.action.ActionClient with the ROS1 actionlib API.
    """

    def __init__(self, ns, ActionSpec):
        """
        Construct a SimpleActionClient.

        Args:
            ns: The namespace for the action (e.g., 'fibonacci')
            ActionSpec: The ROS2 action type (e.g., example_interfaces.action.Fibonacci)
        """
        self._ns = ns
        self._action_type = get_action_type(ActionSpec)
        self._node = rospy.impl.node._get_node()

        self._client = ROS2ActionClient(self._node, self._action_type, ns)

        self._goal_handle = None
        self._result = None
        self._status = GoalStatus.LOST
        self._status_text = ""
        self._simple_state = SimpleGoalState.DONE

        self._done_cb = None
        self._active_cb = None
        self._feedback_cb = None

        self._done_event = threading.Event()
        self._lock = threading.Lock()

    def wait_for_server(self, timeout=rospy.Duration()):
        """
        Block until the action server is available.

        Args:
            timeout: Max time to wait. Zero means wait forever.

        Returns:
            True if server became available, False on timeout.
        """
        if timeout == rospy.Duration() or timeout.to_sec() <= 0:
            timeout_sec = None
        else:
            timeout_sec = timeout.to_sec()

        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """
        Send a goal to the action server.

        Args:
            goal: The goal message to send
            done_cb: Callback(status, result) when goal completes
            active_cb: Callback() when goal becomes active
            feedback_cb: Callback(feedback) for progress updates
        """
        self.stop_tracking_goal()

        with self._lock:
            self._done_cb = done_cb
            self._active_cb = active_cb
            self._feedback_cb = feedback_cb
            self._simple_state = SimpleGoalState.PENDING
            self._done_event.clear()
            self._result = None
            self._status = GoalStatus.PENDING
            self._status_text = ""

        # Send goal asynchronously
        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback
        )
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """Handle the response from send_goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._lock:
                self._status = GoalStatus.REJECTED
                self._simple_state = SimpleGoalState.DONE
                self._done_event.set()
                if self._done_cb:
                    self._done_cb(GoalStatus.REJECTED, None)
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._status = GoalStatus.ACTIVE
            self._simple_state = SimpleGoalState.ACTIVE
            if self._active_cb:
                self._active_cb()

        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        """Handle the result from the action."""
        result = future.result()
        with self._lock:
            self._result = result.result
            self._status = ros2_status_to_ros1(result.status)
            self._simple_state = SimpleGoalState.DONE
            self._done_event.set()
            if self._done_cb:
                self._done_cb(self._status, self._result)

    def _on_feedback(self, feedback_msg):
        """Handle feedback from the action."""
        with self._lock:
            if self._feedback_cb:
                self._feedback_cb(feedback_msg.feedback)

    def send_goal_and_wait(self, goal, execute_timeout=rospy.Duration(),
                           preempt_timeout=rospy.Duration()):
        """
        Send a goal and wait for it to complete.

        Args:
            goal: The goal to send
            execute_timeout: Max time to wait for execution
            preempt_timeout: Max time to wait for preemption after timeout

        Returns:
            The terminal goal status.
        """
        self.send_goal(goal)
        if not self.wait_for_result(execute_timeout):
            rospy.logdebug("Canceling goal")
            self.cancel_goal()
            if self.wait_for_result(preempt_timeout):
                rospy.logdebug("Preempt finished within timeout")
            else:
                rospy.logdebug("Preempt didn't finish within timeout")
        return self.get_state()

    def wait_for_result(self, timeout=rospy.Duration()):
        """
        Block until the goal finishes.

        Args:
            timeout: Max time to wait. Zero means wait forever.

        Returns:
            True if goal finished, False on timeout.
        """
        if timeout == rospy.Duration() or timeout.to_sec() <= 0:
            timeout_sec = None
        else:
            timeout_sec = timeout.to_sec()

        return self._done_event.wait(timeout=timeout_sec)

    def get_result(self):
        """Get the result of the current goal."""
        with self._lock:
            return self._result

    def get_state(self):
        """
        Get the state of the current goal.

        Returns:
            GoalStatus constant (PENDING, ACTIVE, PREEMPTED, SUCCEEDED, etc.)
        """
        with self._lock:
            if self._goal_handle is None:
                return GoalStatus.LOST
            return self._status

    def get_goal_status_text(self):
        """Get the status text for the current goal."""
        with self._lock:
            return self._status_text

    def cancel_goal(self):
        """Cancel the current goal."""
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                # Don't wait for the cancel response, the result callback
                # will handle the terminal state

    def cancel_all_goals(self):
        """Cancel all goals on the server."""
        # Note: rclpy doesn't expose a public API for cancel_all_goals
        # This requires sending a CancelGoal request with empty goal_id
        rospy.logwarn("cancel_all_goals: not fully implemented in ROS2")

    def cancel_goals_at_and_before_time(self, time):
        """Cancel all goals with timestamps at or before the given time."""
        # Note: rclpy doesn't expose a public API for timestamp-based cancel
        rospy.logwarn("cancel_goals_at_and_before_time: not fully implemented in ROS2")

    def stop_tracking_goal(self):
        """Stop tracking the current goal without canceling it."""
        with self._lock:
            self._goal_handle = None
            self._done_cb = None
            self._active_cb = None
            self._feedback_cb = None
