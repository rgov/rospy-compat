import threading
from uuid import UUID

import rospy
from rclpy.action import ActionServer as ROS2ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import GoalEvent
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from actionlib.action_utils import get_action_type
from actionlib.envelopes import ActionEvents
from actionlib.goal_status import GoalStatus


class _Goal:
    def __init__(self, handle):
        self.handle = handle
        self.id = str(UUID(bytes=bytes(handle.goal_id.uuid)))
        # rclpy exposes only the UUID publicly. This is the acceptance
        # timestamp returned to the native client by both Foxy and Rolling.
        self.stamp = handle._goal_info.stamp
        self.result = Future()
        self.preempt = False


class SimpleActionServer:
    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        self._name = rospy.resolve_name(name)
        self._node = rospy.impl.node._get_node()
        self._action_type = get_action_type(ActionSpec)
        self._execute_cb = execute_cb
        self._goal_callback = None
        self._preempt_callback = None
        self._current = None
        self._next = None
        self._results = {}
        self._executing = False
        self._server = None
        self._lock = threading.RLock()

        if auto_start:
            self.start()

    def start(self):
        if self._server is not None:
            return
        self._events = ActionEvents(self._node, self._name)
        self._server = ROS2ActionServer(
            self._node,
            self._action_type,
            self._name,
            execute_callback=self._execute_result,
            callback_group=MutuallyExclusiveCallbackGroup(),
            goal_callback=lambda request: GoalResponse.ACCEPT,
            handle_accepted_callback=self._accepted,
            cancel_callback=self._cancel,
        )

    def _accepted(self, handle):
        goal = _Goal(handle)
        with self._lock:
            self._events.publish(
                self._events.goals, goal, goal.stamp, handle.request
            )
            # Each native execute coroutine returns only its own goal's result.
            self._results[bytes(handle.goal_id.uuid)] = goal.result
            handle.execute()
            if self._next is not None:
                self._finish(self._next, GoalStatus.RECALLED)
            self._next = goal
            preempt = self._preempt_callback if self._current else None
            if self._current is not None:
                self._current.preempt = True
            if self._execute_cb is not None:
                if not self._executing:
                    self._executing = True
                    threading.Thread(
                        target=self._run_goals, daemon=True
                    ).start()
        if preempt is not None:
            preempt()
        if self._goal_callback is not None:
            self._goal_callback()

    async def _execute_result(self, handle):
        with self._lock:
            result = self._results.pop(bytes(handle.goal_id.uuid))
        return await result

    def _cancel(self, handle):
        with self._lock:
            for goal in (self._current, self._next):
                if goal is not None and goal.handle == handle:
                    goal.preempt = True
                    cb = (
                        self._preempt_callback
                        if goal is self._current
                        else None
                    )
                    break
            else:
                return CancelResponse.REJECT
        if cb is not None:
            cb()
        return CancelResponse.ACCEPT

    def _run_goals(self):
        while True:
            with self._lock:
                if self._next is None:
                    self._executing = False
                    return
                request = self.accept_new_goal()
                goal = self._current
            try:
                self._execute_cb(request)
            except Exception as exc:
                rospy.logerr('Exception in action execute callback: %s', exc)
            with self._lock:
                if self._current is goal:
                    self.set_aborted(
                        text='Execute callback returned without a result'
                    )

    def accept_new_goal(self):
        with self._lock:
            if self._next is None:
                rospy.logerr(
                    'accept_new_goal called with no new goal available'
                )
                return None
            if self._current is not None:
                self._finish(self._current, GoalStatus.PREEMPTED)
            self._current, self._next = self._next, None
            return self._current.handle.request

    def is_new_goal_available(self):
        with self._lock:
            return self._next is not None

    def is_preempt_requested(self):
        with self._lock:
            return self._current is not None and self._current.preempt

    def is_active(self):
        with self._lock:
            return self._current is not None and self._current.handle.is_active

    def _finish(self, goal, status, result=None, text=''):
        if goal is None:
            rospy.logerr('No active goal')
            return
        if result is None:
            result = self._action_type.Result()
        if status == GoalStatus.SUCCEEDED:
            goal.handle.succeed()
        elif status == GoalStatus.ABORTED:
            goal.handle.abort()
        else:
            # ROS 1 also permits preemption by the server or a newer goal.
            if not goal.handle.is_cancel_requested:
                goal.handle._update_state(GoalEvent.CANCEL_GOAL)
            goal.handle.canceled()
        if goal is self._current:
            self._current = None
        self._events.publish(
            self._events.results, goal, rospy.Time.now(), result, status, text
        )
        goal.result.set_result(result)

    def set_succeeded(self, result=None, text=''):
        with self._lock:
            self._finish(self._current, GoalStatus.SUCCEEDED, result, text)

    def set_aborted(self, result=None, text=''):
        with self._lock:
            self._finish(self._current, GoalStatus.ABORTED, result, text)

    def set_preempted(self, result=None, text=''):
        with self._lock:
            self._finish(self._current, GoalStatus.PREEMPTED, result, text)

    def publish_feedback(self, feedback):
        with self._lock:
            if self._current is not None:
                self._current.handle.publish_feedback(feedback)

    def register_goal_callback(self, cb):
        if self._execute_cb is not None:
            rospy.logwarn('Cannot register goal callback with an execute_cb')
        else:
            self._goal_callback = cb

    def register_preempt_callback(self, cb):
        self._preempt_callback = cb
