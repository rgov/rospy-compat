"""
Publisher classes for compatibility with rospy.
"""

from .qos import create_qos_profile


class PublisherWrapper:
    """
    Wrapper around rclpy Publisher to provide ROS1-compatible publish() behavior.
    In ROS1, you could publish primitive types directly (e.g., pub.publish("hello")).
    This wrapper automatically converts primitives to message objects.
    """

    def __init__(self, publisher, msg_type):
        self._publisher = publisher
        self._msg_type = msg_type

    def publish(self, msg):
        """
        Publish a message. Auto-converts primitive types to message objects.

        Args:
            msg: Either a message object or a primitive value
        """
        # Check if msg is already a message object of the right type
        if isinstance(msg, self._msg_type):
            self._publisher.publish(msg)
        else:
            # Try to construct message from primitive value
            # This handles common ROS1 pattern: pub.publish("hello") for String messages
            try:
                msg_obj = self._msg_type()
                # Check for common single-field message types
                if hasattr(msg_obj, 'data'):
                    msg_obj.data = msg
                    self._publisher.publish(msg_obj)
                else:
                    # Can't auto-convert, raise helpful error
                    raise TypeError(
                        f"Cannot auto-convert {type(msg)} to {self._msg_type}. "
                        f"Please pass a {self._msg_type} object."
                    )
            except Exception as e:
                raise TypeError(
                    f"Error publishing message: {e}. "
                    f"Expected {self._msg_type} object or convertible value."
                )

    def __getattr__(self, name):
        """Forward other attributes to the underlying publisher."""
        return getattr(self._publisher, name)


def Publisher(topic, msg_type, queue_size=10, latch=False, **kwargs):
    """
    Create a publisher for the specified topic.
    Compatible with rospy.Publisher.

    Args:
        topic (str): Topic name to publish to
        msg_type (type): Message type class
        queue_size (int): Size of the outgoing message queue (maps to QoS depth)
        latch (bool): If True, last message published is saved and sent to new subscribers
        **kwargs: Additional arguments (most are ROS1-specific and ignored)

    Returns:
        PublisherWrapper: A wrapped publisher with ROS1-compatible .publish() method
    """
    from .node import _get_node

    node = _get_node()

    # Create QoS profile from ROS1 parameters
    qos = create_qos_profile(queue_size=queue_size, latch=latch)

    # Create the publisher
    publisher = node.create_publisher(msg_type, topic, qos)

    # Wrap it to provide ROS1-compatible publish() behavior
    return PublisherWrapper(publisher, msg_type)
