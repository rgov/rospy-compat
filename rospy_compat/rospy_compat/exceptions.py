"""
ROS exceptions for compatibility with rospy.
"""


class ROSException(Exception):
    """
    Base exception class for ROS-related errors.
    """
    pass


class ROSInterruptException(ROSException, KeyboardInterrupt):
    """
    Exception raised when ROS is interrupted (e.g., via Ctrl+C).
    Compatible with rospy.ROSInterruptException.
    """
    pass


class ROSInitException(ROSException):
    """
    Exception raised when there is an error initializing ROS.
    """
    pass


class ROSSerializationException(ROSException):
    """
    Exception raised when there is an error serializing or deserializing messages.
    """
    pass
