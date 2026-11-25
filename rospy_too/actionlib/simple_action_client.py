import threading
from functools import partial

import rospy
from rclpy.action import ActionClient as ROS2ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from actionlib.action_utils import get_action_type
from actionlib.goal_status import (
    GoalStatus,
    SimpleGoalState,
    ros2_status_to_ros1,
)

__all__ = ['SimpleActionClient', 'SimpleGoalState']


class _Goal:
    def __init__(self, done_cb, active_cb, feedback_cb):
        self.done_cb = done_cb
        self.active_cb = active_cb
        self.feedback_cb = feedback_cb
        self.handle = None
        self.result = None
        self.status = GoalStatus.PENDING
        self.cancel_requested = False
        self.done = threading.Event()


def _timeout(duration):
    return duration.to_sec() if duration.to_sec() > 0 else None


class SimpleActionClient:
    def __init__(self, ns, ActionSpec):
        self._client = ROS2ActionClient(
            rospy.impl.node._get_node(),
            get_action_type(ActionSpec),
            rospy.resolve_name(ns),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._goal = None
        self._lock = threading.Lock()

    def wait_for_server(self, timeout=rospy.Duration()):
        return self._client.wait_for_server(timeout_sec=_timeout(timeout))

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        tracked = _Goal(done_cb, active_cb, feedback_cb)
        with self._lock:
            self._goal = tracked
        future = self._client.send_goal_async(
            goal, feedback_callback=partial(self._feedback, tracked)
        )
        future.add_done_callback(partial(self._accepted, tracked))

    def _accepted(self, tracked, future):
        handle = future.result()
        if not handle.accepted:
            self._finish(tracked, GoalStatus.REJECTED, None)
            return
        with self._lock:
            if self._goal is not tracked:
                return
            tracked.handle = handle
            tracked.status = GoalStatus.ACTIVE
            if tracked.cancel_requested:
                handle.cancel_goal_async()
        result = handle.get_result_async()
        try:
            if tracked.active_cb is not None:
                tracked.active_cb()
        finally:
            result.add_done_callback(partial(self._received_result, tracked))

    def _received_result(self, tracked, future):
        response = future.result()
        self._finish(
            tracked, ros2_status_to_ros1(response.status), response.result
        )

    def _finish(self, tracked, status, result):
        with self._lock:
            if self._goal is not tracked:
                return
            tracked.status = status
            tracked.result = result
        if tracked.done_cb is None:
            tracked.done.set()
        else:

            def notify():
                try:
                    tracked.done_cb(status, result)
                finally:
                    tracked.done.set()

            # A ROS 1 done callback may block on another action or service.
            threading.Thread(target=notify, daemon=True).start()

    def _feedback(self, tracked, message):
        with self._lock:
            if self._goal is not tracked:
                return
        if tracked.feedback_cb is not None:
            tracked.feedback_cb(message.feedback)

    def send_goal_and_wait(
        self,
        goal,
        execute_timeout=rospy.Duration(),
        preempt_timeout=rospy.Duration(),
    ):
        self.send_goal(goal)
        if not self.wait_for_result(execute_timeout):
            self.cancel_goal()
            self.wait_for_result(preempt_timeout)
        return self.get_state()

    def wait_for_result(self, timeout=rospy.Duration()):
        with self._lock:
            goal = self._goal
        return goal is not None and goal.done.wait(_timeout(timeout))

    def get_result(self):
        with self._lock:
            return self._goal.result if self._goal is not None else None

    def get_state(self):
        with self._lock:
            return (
                self._goal.status if self._goal is not None else GoalStatus.LOST
            )

    def get_goal_status_text(self):
        # A native ROS 2 GetResult response contains no status text.
        return ''

    def cancel_goal(self):
        with self._lock:
            if self._goal is not None:
                self._goal.cancel_requested = True
                if self._goal.handle is not None:
                    self._goal.handle.cancel_goal_async()

    def cancel_all_goals(self):
        rospy.logwarn('cancel_all_goals: not fully implemented in ROS2')

    def cancel_goals_at_and_before_time(self, time):
        rospy.logwarn(
            'cancel_goals_at_and_before_time: not fully implemented in ROS2'
        )

    def stop_tracking_goal(self):
        with self._lock:
            self._goal = None
