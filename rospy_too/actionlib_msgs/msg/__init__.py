"""actionlib_msgs.msg compatibility module for ROS2.

Provides ROS1-compatible GoalStatus and GoalID message types.
"""

from actionlib.goal_status import GoalID, GoalStatus


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
