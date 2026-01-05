"""
Quality of Service (QoS) profile utilities for converting ROS1 parameters to ROS2 QoS.
"""

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


def create_qos_profile(queue_size=10, latch=False, reliable=True):
    """
    Create a QoS profile based on ROS1-style parameters.

    Args:
        queue_size (int): The depth of the message queue (ROS1's queue_size parameter).
        latch (bool): Whether to latch messages (ROS1's latch parameter).
                     If True, uses TRANSIENT_LOCAL durability for "latching" behavior.
        reliable (bool): Whether to use reliable communication. Default is True.

    Returns:
        QoSProfile: A configured QoS profile for ROS2.
    """
    qos = QoSProfile(depth=queue_size)

    # Set reliability
    if reliable:
        qos.reliability = ReliabilityPolicy.RELIABLE
    else:
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

    # Set durability (latching behavior)
    if latch:
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        qos.durability = DurabilityPolicy.VOLATILE

    # Use KEEP_LAST history policy (standard for ROS1 compatibility)
    qos.history = HistoryPolicy.KEEP_LAST

    return qos
