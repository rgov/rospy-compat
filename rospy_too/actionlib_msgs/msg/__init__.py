"""actionlib_msgs.msg compatibility module for ROS2.

Provides ROS1-compatible GoalStatus and GoalID message types.
"""

from actionlib.goal_status import GoalStatus


class GoalID:
    """ROS1-compatible GoalID wrapper for ROS2.

    In ROS1, GoalID has:
        - stamp: time when goal was created
        - id: unique string identifier

    In ROS2, action goals use UUIDs instead, but this wrapper
    provides the ROS1 interface for compatibility.
    """

    __slots__ = ['stamp', 'id']

    def __init__(self, stamp=None, id=''):
        from builtin_interfaces.msg import Time
        self.stamp = stamp if stamp is not None else Time()
        self.id = id

    def __repr__(self):
        return f'GoalID(stamp={self.stamp}, id={self.id!r})'

    def __eq__(self, other):
        if not isinstance(other, GoalID):
            return False
        return self.stamp == other.stamp and self.id == other.id


class GoalStatusArray:
    """ROS1-compatible GoalStatusArray wrapper for ROS2.

    Contains an array of GoalStatus messages with a header.
    """

    __slots__ = ['header', 'status_list']

    def __init__(self, header=None, status_list=None):
        from std_msgs.msg import Header
        self.header = header if header is not None else Header()
        self.status_list = status_list if status_list is not None else []

    def __repr__(self):
        return f'GoalStatusArray(header={self.header}, status_list={self.status_list})'


__all__ = ['GoalStatus', 'GoalID', 'GoalStatusArray']
