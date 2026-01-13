"""SimpleActionServer implementation wrapping rclpy.action.ActionServer."""

import threading
import traceback

import rospy
from rclpy.action import ActionServer as ROS2ActionServer
from rclpy.action import CancelResponse, GoalResponse

from actionlib.goal_status import GoalStatus
from actionlib.action_utils import get_action_type


class SimpleActionServer:
    """
    A simple server for providing an action.

    Implements a single goal policy: only one goal can be active at a time.
    New goals preempt previous goals.
    """

    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        """
        Construct a SimpleActionServer.

        Args:
            name: The action name/namespace
            ActionSpec: The ROS2 action type
            execute_cb: Optional callback(goal) called in a separate thread
            auto_start: If True, start immediately (not recommended)
        """
        self._name = name
        self._action_type = get_action_type(ActionSpec)
        self._node = rospy.impl.node._get_node()

        self._execute_cb = execute_cb
        self._goal_callback = None
        self._preempt_callback = None

        self._current_goal_handle = None
        self._next_goal = None
        self._new_goal = False
        self._preempt_request = False
        self._new_goal_preempt_request = False

        self._lock = threading.RLock()
        self._execute_condition = threading.Condition(self._lock)
        self._terminate = False

        self._server = None
        self._execute_thread = None
        self._started = False
        self._current_done_event = None

        if auto_start:
            rospy.logwarn(
                "auto_start=True is deprecated. Set auto_start=False and call start()."
            )
            self.start()

    def start(self):
        """Start the action server."""
        if self._started:
            return

        self._server = ROS2ActionServer(
            self._node,
            self._action_type,
            self._name,
            execute_callback=self._execute_wrapper,
            goal_callback=self._goal_callback_wrapper,
            cancel_callback=self._cancel_callback_wrapper,
        )

        # Note: execute_cb is now run directly in _execute_wrapper
        # No separate thread needed

        self._started = True

    def __del__(self):
        self._terminate = True
        if getattr(self, '_execute_thread', None) and getattr(self, '_execute_condition', None):
            with self._execute_condition:
                self._execute_condition.notify_all()

    def _goal_callback_wrapper(self, goal_request):
        """Handle incoming goal requests."""
        with self._lock:
            # Accept the goal - we'll handle preemption ourselves
            return GoalResponse.ACCEPT

    def _cancel_callback_wrapper(self, goal_handle):
        """Handle cancel requests."""
        with self._lock:
            # Compare goal IDs since goal_handle objects may differ
            is_current = (self._current_goal_handle is not None and
                          goal_handle.goal_id == self._current_goal_handle.goal_id)
            is_next = (self._next_goal is not None and
                       goal_handle.goal_id == self._next_goal.goal_id)
            if is_current or is_next:
                self._preempt_request = True
                if is_next:
                    self._new_goal_preempt_request = True
                if self._preempt_callback:
                    self._preempt_callback()
            return CancelResponse.ACCEPT

    def _execute_wrapper(self, goal_handle):
        """
        Wrapper for the ROS2 execute callback.

        This is called by the ROS2 action server for each accepted goal.
        For the execute_cb pattern, we run the callback directly.
        For the goal_callback pattern, we call the user's callback.
        """
        with self._lock:
            # If there's a current goal, it's being preempted
            if self._current_goal_handle is not None:
                self._preempt_request = True

            # For execute_cb pattern, accept the goal and run execute_cb directly
            if self._execute_cb:
                # Set this goal as current
                if self._current_goal_handle is not None:
                    # Preempt old goal
                    try:
                        self._current_goal_handle.abort()
                    except Exception:
                        pass
                self._current_goal_handle = goal_handle
                self._preempt_request = False
            else:
                # For goal_callback pattern
                self._next_goal = goal_handle
                self._new_goal = True
                self._new_goal_preempt_request = False

                if self._preempt_callback and self._current_goal_handle:
                    self._preempt_callback()

                if self._goal_callback:
                    self._goal_callback()

        # For execute_cb pattern, run the callback directly
        # This blocks the executor, which is the standard ROS2 behavior
        if self._execute_cb:
            try:
                self._execute_cb(goal_handle.request)

                if self.is_active():
                    rospy.logwarn(
                        "execute_cb did not set the goal to a terminal status. "
                        "Setting to aborted."
                    )
                    self.set_aborted(text="No terminal state was set.")
            except Exception as e:
                rospy.logerr("Exception in execute callback: %s" % e)
                self.set_aborted(text="Exception in execute callback: %s" % e)

        # Return a default result - the actual result was already sent via goal_handle
        return self._action_type.Result()

    def accept_new_goal(self):
        """
        Accept the next available goal.

        Returns:
            The goal message, or None if no goal is available.
        """
        with self._lock:
            if not self._new_goal or self._next_goal is None:
                rospy.logerr("accept_new_goal called when no new goal available")
                return None

            # Preempt current goal if active
            if self._current_goal_handle is not None and self.is_active():
                self._current_goal_handle.abort()

            self._current_goal_handle = self._next_goal
            self._next_goal = None
            self._new_goal = False
            self._preempt_request = self._new_goal_preempt_request
            self._new_goal_preempt_request = False

            return self._current_goal_handle.request

    def is_new_goal_available(self):
        """Check if a new goal is available."""
        with self._lock:
            return self._new_goal

    def is_preempt_requested(self):
        """Check if preemption has been requested."""
        with self._lock:
            return self._preempt_request

    def is_active(self):
        """Check if a goal is currently active."""
        with self._lock:
            if self._current_goal_handle is None:
                return False
            return self._current_goal_handle.is_active

    def _signal_done(self):
        """Signal that the current goal is done."""
        if self._current_done_event is not None:
            self._current_done_event.set()
            self._current_done_event = None

    def set_succeeded(self, result=None, text=""):
        """Mark the current goal as succeeded."""
        with self._lock:
            if self._current_goal_handle is None:
                rospy.logerr("set_succeeded called with no active goal")
                return
            if result is None:
                result = self._action_type.Result()
            self._current_goal_handle.succeed()
            self._current_goal_handle = None
            self._signal_done()

    def set_aborted(self, result=None, text=""):
        """Mark the current goal as aborted."""
        with self._lock:
            if self._current_goal_handle is None:
                rospy.logerr("set_aborted called with no active goal")
                return
            if result is None:
                result = self._action_type.Result()
            self._current_goal_handle.abort()
            self._current_goal_handle = None
            self._signal_done()

    def set_preempted(self, result=None, text=""):
        """Mark the current goal as preempted (canceled)."""
        with self._lock:
            if self._current_goal_handle is None:
                rospy.logerr("set_preempted called with no active goal")
                return
            if result is None:
                result = self._action_type.Result()
            self._current_goal_handle.canceled()
            self._current_goal_handle = None
            self._signal_done()

    def publish_feedback(self, feedback):
        """Publish feedback for the current goal."""
        with self._lock:
            if self._current_goal_handle is None:
                rospy.logerr("publish_feedback called with no active goal")
                return
            self._current_goal_handle.publish_feedback(feedback)

    def register_goal_callback(self, cb):
        """Register a callback for when new goals arrive."""
        if self._execute_cb:
            rospy.logwarn(
                "Cannot register goal callback when execute_cb is set."
            )
            return
        self._goal_callback = cb

    def register_preempt_callback(self, cb):
        """Register a callback for when preemption is requested."""
        self._preempt_callback = cb
