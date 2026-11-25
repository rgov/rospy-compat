from threading import Condition

from .impl.node import _get_node, _is_node_initialized, get_rostime, get_time
from .time import Duration, Time

__all__ = [
    'Duration',
    'Time',
    'get_rostime',
    'get_time',
    'get_rostime_cond',
    'is_wallclock',
]

_rostime_cond = Condition()


def is_wallclock():
    return (
        not _is_node_initialized()
        or not _get_node().get_clock().ros_time_is_active
    )


def get_rostime_cond():
    if not is_wallclock():
        raise NotImplementedError('ROS clock update notifications')
    return _rostime_cond
