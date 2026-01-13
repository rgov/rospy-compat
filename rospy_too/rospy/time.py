# Time and Duration classes for rospy compatibility.
# Re-exports builtin_interfaces.msg.Time/Duration which have been monkeypatched
# in impl/patches.py to provide ROS1 rospy.Time/Duration compatibility.

from builtin_interfaces.msg import Duration, Time

__all__ = ['Time', 'Duration']
