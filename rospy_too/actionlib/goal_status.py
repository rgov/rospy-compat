"""
GoalStatus constants compatible with ROS1 actionlib_msgs/GoalStatus.

Maps ROS2 action_msgs GoalStatus values to ROS1 equivalents.
"""

from action_msgs.msg import GoalStatus as ROS2GoalStatus


def _get_name_of_constant(cls, n):
    """Get the name of a constant by its value."""
    for k, v in cls.__dict__.items():
        if isinstance(v, int) and v == n:
            return k
    return "UNKNOWN_%d" % n


class GoalStatus:
    """
    ROS1-compatible goal status constants.

    These match the values from actionlib_msgs/GoalStatus in ROS1.
    """
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

    @classmethod
    def to_string(cls, status):
        return _get_name_of_constant(cls, status)


class SimpleGoalState:
    """Simplified goal states used by SimpleActionClient."""
    PENDING = 0
    ACTIVE = 1
    DONE = 2

    @classmethod
    def to_string(cls, state):
        return _get_name_of_constant(cls, state)


class CommState:
    """Communication state machine states (for API compatibility)."""
    WAITING_FOR_GOAL_ACK = 0
    PENDING = 1
    ACTIVE = 2
    WAITING_FOR_RESULT = 3
    WAITING_FOR_CANCEL_ACK = 4
    RECALLING = 5
    PREEMPTING = 6
    DONE = 7
    LOST = 8

    @classmethod
    def to_string(cls, state):
        return _get_name_of_constant(cls, state)


class TerminalState:
    """Terminal goal states."""
    RECALLED = GoalStatus.RECALLED
    REJECTED = GoalStatus.REJECTED
    PREEMPTED = GoalStatus.PREEMPTED
    ABORTED = GoalStatus.ABORTED
    SUCCEEDED = GoalStatus.SUCCEEDED
    LOST = GoalStatus.LOST

    @classmethod
    def to_string(cls, state):
        return _get_name_of_constant(cls, state)


def ros2_status_to_ros1(ros2_status):
    """Convert ROS2 GoalStatus to ROS1 GoalStatus."""
    mapping = {
        ROS2GoalStatus.STATUS_UNKNOWN: GoalStatus.PENDING,
        ROS2GoalStatus.STATUS_ACCEPTED: GoalStatus.PENDING,
        ROS2GoalStatus.STATUS_EXECUTING: GoalStatus.ACTIVE,
        ROS2GoalStatus.STATUS_CANCELING: GoalStatus.PREEMPTING,
        ROS2GoalStatus.STATUS_SUCCEEDED: GoalStatus.SUCCEEDED,
        ROS2GoalStatus.STATUS_CANCELED: GoalStatus.PREEMPTED,
        ROS2GoalStatus.STATUS_ABORTED: GoalStatus.ABORTED,
    }
    return mapping.get(ros2_status, GoalStatus.LOST)


def is_terminal_state(status):
    """Check if a GoalStatus value represents a terminal state."""
    return status in (
        GoalStatus.PREEMPTED,
        GoalStatus.SUCCEEDED,
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    )
