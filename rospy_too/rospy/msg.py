# Message utilities for rospy compatibility.

from .exceptions import ROSException


class AnyMsg:
    # Subscribe to any topic regardless of type. Incoming messages are not
    # deserialized. Instead, raw serialized data is accessed via _buff.
    _md5sum = '*'
    _type = '*'
    _has_header = False
    _full_text = ''
    __slots__ = ['_buff', '_connection_header']

    def __init__(self):
        self._buff = None
        self._connection_header = {}

    def serialize(self, buff):
        if self._buff is None:
            raise ROSException('AnyMsg is not initialized')
        buff.write(self._buff)

    def deserialize(self, data):
        self._buff = data
        return self
