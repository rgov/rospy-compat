from functools import lru_cache

from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import Header

from actionlib.goal_status import GoalID, GoalStatus


def _wire_type():
    # Plain rospy imports can also run directly from an unbuilt source tree.
    from rospy_too_interfaces.msg import ActionEvent

    return ActionEvent


def _type_name(payload_type):
    package = payload_type.__module__.split('.', 1)[0]
    return f'{package}/action/{payload_type.__name__}'


class _Envelope:
    _rospy_wire_type = staticmethod(_wire_type)

    @classmethod
    def _rospy_from_wire(cls, event):
        if event.payload_type != _type_name(cls._payload_type):
            return None
        goal_id = GoalID(stamp=event.goal_stamp, id=event.goal_id)
        payload = deserialize_message(bytes(event.payload), cls._payload_type)
        if cls._kind == 'goal':
            return cls(event.header, goal_id, payload)
        status = GoalStatus(goal_id, event.status, event.text)
        return cls(event.header, status, payload)


class _ActionGoal(_Envelope):
    def __init__(self, header=None, goal_id=None, goal=None):
        self.header = Header() if header is None else header
        self.goal_id = GoalID() if goal_id is None else goal_id
        self.goal = self._payload_type() if goal is None else goal


class _ActionResult(_Envelope):
    def __init__(self, header=None, status=None, result=None):
        self.header = Header() if header is None else header
        self.status = GoalStatus() if status is None else status
        self.result = self._payload_type() if result is None else result


@lru_cache(maxsize=None)
def envelope_type(payload_type, kind):
    action = payload_type.__name__.rsplit('_', 1)[0]
    return type(
        f'{action}Action{kind.title()}',
        (_ActionGoal if kind == 'goal' else _ActionResult,),
        {'_payload_type': payload_type, '_kind': kind},
    )


class ActionEvents:
    def __init__(self, node, name):
        self.wire_type = _wire_type()
        self.goals = node.create_publisher(self.wire_type, name + '/goal', 50)
        self.results = node.create_publisher(
            self.wire_type, name + '/result', 50
        )

    def publish(self, pub, goal, stamp, payload, status=0, text=''):
        event = self.wire_type()
        event.header.stamp = stamp
        event.goal_stamp = goal.stamp
        event.goal_id = goal.id
        event.status = status
        event.text = text
        event.payload_type = _type_name(type(payload))
        event.payload = serialize_message(payload)
        pub.publish(event)
