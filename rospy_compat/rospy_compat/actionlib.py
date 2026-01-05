"""
Actionlib compatibility module for ROS2.
Provides SimpleActionServer and SimpleActionClient classes compatible with ROS1 actionlib.
"""

import time as python_time
import threading

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus as ROS2GoalStatus


# Goal status constants compatible with ROS1 actionlib
class GoalStatus:
    """Goal status constants compatible with actionlib.GoalStatus"""
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


def _map_ros2_status_to_ros1(ros2_status):
    """Map ROS2 goal status to ROS1 actionlib status"""
    status_map = {
        ROS2GoalStatus.STATUS_ACCEPTED: GoalStatus.ACTIVE,
        ROS2GoalStatus.STATUS_EXECUTING: GoalStatus.ACTIVE,
        ROS2GoalStatus.STATUS_CANCELING: GoalStatus.PREEMPTING,
        ROS2GoalStatus.STATUS_SUCCEEDED: GoalStatus.SUCCEEDED,
        ROS2GoalStatus.STATUS_CANCELED: GoalStatus.PREEMPTED,
        ROS2GoalStatus.STATUS_ABORTED: GoalStatus.ABORTED,
    }
    return status_map.get(ros2_status, GoalStatus.LOST)


class SimpleActionServer:
    """
    Simple action server compatible with actionlib.SimpleActionServer.
    Wraps rclpy.action.ActionServer to provide ROS1-style API.
    """

    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        """
        Create a SimpleActionServer.

        Args:
            name (str): Action name
            ActionSpec: Action type class
            execute_cb (callable): Callback for executing goals. Receives goal as argument.
            auto_start (bool): Whether to start the server automatically
        """
        from .node import _get_node

        self.node = _get_node()
        self._action_spec = ActionSpec
        self._execute_cb = execute_cb
        self._current_goal = None
        self._goal_lock = threading.Lock()

        # Create ROS2 action server
        self._server = ActionServer(
            self.node,
            ActionSpec,
            name,
            execute_callback=self._execute_wrapper,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )

        if auto_start:
            self.start()

    def _goal_callback(self, goal_request):
        """Handle incoming goal requests"""
        # Accept all goals by default (ROS1 behavior)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel requests"""
        # Accept all cancel requests by default (ROS1 behavior)
        return CancelResponse.ACCEPT

    def _execute_wrapper(self, goal_handle):
        """
        Wrapper around user's execute callback.
        Maps ROS2 goal_handle to ROS1-style interface.
        """
        with self._goal_lock:
            self._current_goal = goal_handle

        try:
            # Mark goal as executing
            goal_handle.execute()

            # Call user's callback with just the goal (ROS1 style)
            if self._execute_cb:
                self._execute_cb(goal_handle.request)

            # If callback didn't set a terminal state, succeed by default
            # (In ROS1, if you don't call set_succeeded/aborted/preempted, it succeeds)
            # Note: This is a simplification; real behavior depends on callback

        except Exception as e:
            self.node.get_logger().error(f"Exception in action execute callback: {e}")
            goal_handle.abort()
            result = self._action_spec.Result()
            return result

        # Return default result
        result = self._action_spec.Result()
        return result

    def start(self):
        """Start the action server (no-op in ROS2, servers auto-start)"""
        pass

    def set_succeeded(self, result=None, text=""):
        """
        Set the current goal as succeeded.

        Args:
            result: Result object. If None, creates default result.
            text (str): Optional success message

        Returns:
            Result object
        """
        with self._goal_lock:
            if self._current_goal:
                if result is None:
                    result = self._action_spec.Result()

                self._current_goal.succeed()

                if text:
                    self.node.get_logger().info(text)

                return result

        return self._action_spec.Result() if result is None else result

    def set_preempted(self, result=None, text=""):
        """
        Set the current goal as preempted (canceled).

        Args:
            result: Result object. If None, creates default result.
            text (str): Optional preempt message

        Returns:
            Result object
        """
        with self._goal_lock:
            if self._current_goal:
                if result is None:
                    result = self._action_spec.Result()

                self._current_goal.canceled()

                if text:
                    self.node.get_logger().info(text)

                return result

        return self._action_spec.Result() if result is None else result

    def set_aborted(self, result=None, text=""):
        """
        Set the current goal as aborted.

        Args:
            result: Result object. If None, creates default result.
            text (str): Optional abort message

        Returns:
            Result object
        """
        with self._goal_lock:
            if self._current_goal:
                if result is None:
                    result = self._action_spec.Result()

                self._current_goal.abort()

                if text:
                    self.node.get_logger().error(text)

                return result

        return self._action_spec.Result() if result is None else result

    def publish_feedback(self, feedback):
        """
        Publish feedback for the current goal.

        Args:
            feedback: Feedback message
        """
        with self._goal_lock:
            if self._current_goal:
                self._current_goal.publish_feedback(feedback)

    def is_preempt_requested(self):
        """
        Check if preemption (cancellation) has been requested.

        Returns:
            bool: True if cancel requested, False otherwise
        """
        with self._goal_lock:
            if self._current_goal:
                return self._current_goal.is_cancel_requested
            return False

    def is_active(self):
        """
        Check if there is an active goal.

        Returns:
            bool: True if goal is active, False otherwise
        """
        with self._goal_lock:
            if self._current_goal:
                return self._current_goal.is_active
            return False

    def is_new_goal_available(self):
        """
        Check if a new goal is available.
        Note: This is a simplified version, may not match ROS1 exactly.

        Returns:
            bool: True if new goal available
        """
        # In ROS2, we don't have the same queuing behavior
        # This is a simplified implementation
        return self.is_active()

    def accept_new_goal(self):
        """
        Accept a new goal.
        Note: This is automatically handled in ROS2.

        Returns:
            Goal request
        """
        with self._goal_lock:
            if self._current_goal:
                return self._current_goal.request
            return None


class SimpleActionClient:
    """
    Simple action client compatible with actionlib.SimpleActionClient.
    Wraps rclpy.action.ActionClient to provide ROS1-style API.
    """

    def __init__(self, name, ActionSpec):
        """
        Create a SimpleActionClient.

        Args:
            name (str): Action name
            ActionSpec: Action type class
        """
        from .node import _get_node

        self.node = _get_node()
        self._action_spec = ActionSpec
        self._client = ActionClient(self.node, ActionSpec, name)
        self._goal_handle = None
        self._result_future = None
        self._result = None
        self._goal_status = GoalStatus.PENDING
        self._lock = threading.Lock()

    def wait_for_server(self, timeout=None):
        """
        Wait for the action server to become available.

        Args:
            timeout (float): Maximum time to wait in seconds. None means wait forever.

        Returns:
            bool: True if server is available, False if timeout
        """
        from .node import is_shutdown

        if timeout is None:
            # Wait forever
            while not self._client.server_is_ready() and not is_shutdown():
                python_time.sleep(0.1)
            return not is_shutdown()
        else:
            # Wait with timeout
            end_time = python_time.time() + timeout

            while not self._client.server_is_ready() and not is_shutdown():
                if python_time.time() >= end_time:
                    return False
                python_time.sleep(0.1)

            return self._client.server_is_ready()

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """
        Send a goal to the action server.

        Args:
            goal: Goal message
            done_cb (callable): Callback when goal completes. Receives (state, result).
            active_cb (callable): Callback when goal becomes active (currently not used).
            feedback_cb (callable): Callback for feedback messages. Receives (feedback).
        """
        # Send goal asynchronously
        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=feedback_cb if feedback_cb else lambda fb: None
        )

        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        with self._lock:
            self._goal_handle = send_goal_future.result()

            if self._goal_handle is None:
                self.node.get_logger().error("Goal was rejected by server")
                self._goal_status = GoalStatus.REJECTED
                return

            if not self._goal_handle.accepted:
                self.node.get_logger().error("Goal was rejected by server")
                self._goal_status = GoalStatus.REJECTED
                return

            self._goal_status = GoalStatus.ACTIVE

            # Get result future
            self._result_future = self._goal_handle.get_result_async()

            # Add done callback if provided
            if done_cb:
                def result_callback(future):
                    result = future.result()
                    with self._lock:
                        self._result = result.result
                        self._goal_status = _map_ros2_status_to_ros1(result.status)

                    # Call user's done callback
                    done_cb(self._goal_status, result.result)

                self._result_future.add_done_callback(result_callback)

    def wait_for_result(self, timeout=None):
        """
        Wait for the action result.

        Args:
            timeout (float): Maximum time to wait in seconds. None means wait forever.

        Returns:
            bool: True if result received, False if timeout
        """
        if self._result_future is None:
            return False

        timeout_sec = timeout if timeout is not None else None

        try:
            rclpy.spin_until_future_complete(self.node, self._result_future, timeout_sec=timeout_sec)
        except Exception as e:
            self.node.get_logger().error(f"Error waiting for result: {e}")
            return False

        # Store result
        if self._result_future.done():
            result = self._result_future.result()
            with self._lock:
                self._result = result.result
                self._goal_status = _map_ros2_status_to_ros1(result.status)
            return True

        return False

    def get_result(self):
        """
        Get the result of the action.

        Returns:
            Result message, or None if not available
        """
        with self._lock:
            return self._result

    def get_state(self):
        """
        Get the current state of the goal.

        Returns:
            int: Goal state (one of GoalStatus constants)
        """
        with self._lock:
            if self._goal_handle is None:
                return GoalStatus.PENDING

            # Update status from goal handle
            if self._goal_handle.status == ROS2GoalStatus.STATUS_EXECUTING:
                self._goal_status = GoalStatus.ACTIVE
            elif self._goal_handle.status == ROS2GoalStatus.STATUS_SUCCEEDED:
                self._goal_status = GoalStatus.SUCCEEDED
            elif self._goal_handle.status == ROS2GoalStatus.STATUS_CANCELED:
                self._goal_status = GoalStatus.PREEMPTED
            elif self._goal_handle.status == ROS2GoalStatus.STATUS_ABORTED:
                self._goal_status = GoalStatus.ABORTED

            return self._goal_status

    def cancel_goal(self):
        """
        Cancel the current goal.
        """
        with self._lock:
            if self._goal_handle:
                cancel_future = self._goal_handle.cancel_goal_async()
                # Don't wait for cancellation to complete
                # (ROS1 actionlib doesn't wait either)

    def cancel_all_goals(self):
        """
        Cancel all goals.
        Note: In simple action client, this is the same as cancel_goal.
        """
        self.cancel_goal()

    def stop_tracking_goal(self):
        """
        Stop tracking the current goal.
        """
        with self._lock:
            self._goal_handle = None
            self._result_future = None
            self._result = None
            self._goal_status = GoalStatus.PENDING
