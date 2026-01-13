# QoS profile utilities for ROS1 to ROS2 conversion.

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def create_qos_profile(queue_size=10, latch=False, reliable=True):
    qos = QoSProfile(depth=queue_size)
    qos.reliability = (
        ReliabilityPolicy.RELIABLE
        if reliable
        else ReliabilityPolicy.BEST_EFFORT
    )
    qos.durability = (
        DurabilityPolicy.TRANSIENT_LOCAL if latch else DurabilityPolicy.VOLATILE
    )
    qos.history = HistoryPolicy.KEEP_LAST
    return qos
