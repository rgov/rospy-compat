"""
Subscriber classes for compatibility with rospy.
"""

import io
from rclpy.serialization import deserialize_message, serialize_message

from .qos import create_qos_profile


class AnyMsg:
    """
    Marker class for subscribing to any message type.
    Compatible with rospy.AnyMsg.

    When used with Subscriber, the callback will receive a message object
    with a _connection_header attribute and serialize() method for compatibility.
    """
    pass


class _AnyMsgWrapper:
    """
    Internal wrapper for SerializedMessage to provide rospy.AnyMsg compatibility.
    Provides _connection_header attribute and serialize() method.
    """

    def __init__(self, serialized_msg, topic_name, msg_type_str):
        """
        Create an AnyMsg wrapper.

        Args:
            serialized_msg: The rclpy SerializedMessage
            topic_name (str): Name of the topic
            msg_type_str (str): Message type string (e.g., 'std_msgs/msg/String')
        """
        self._serialized_msg = serialized_msg
        self._connection_header = {
            'type': msg_type_str,
            'topic': topic_name,
            'message_definition': '',  # Not available in ROS2
            'callerid': '',  # Not directly available in ROS2
            'latching': '0',
        }

    def serialize(self, buff=None):
        """
        Serialize the message to a buffer.

        Args:
            buff: Optional buffer to write to (io.BytesIO)

        Returns:
            bytes: Serialized message data
        """
        if buff is None:
            return bytes(self._serialized_msg)
        else:
            buff.write(bytes(self._serialized_msg))
            return buff.getvalue()

    def deserialize(self, str_data):
        """
        Deserialize is not implemented for the wrapper.
        Users should manually deserialize using the message type.
        """
        raise NotImplementedError(
            "AnyMsg wrapper does not support deserialize(). "
            "Please deserialize manually using the message class."
        )


def Subscriber(topic, msg_type, callback=None, callback_args=None, queue_size=10, **kwargs):
    """
    Create a subscriber for the specified topic.
    Compatible with rospy.Subscriber.

    Args:
        topic (str): Topic name to subscribe to
        msg_type (type or AnyMsg): Message type class, or AnyMsg for dynamic types
        callback (callable): Function to call when a message is received
        callback_args (tuple): Additional arguments to pass to callback
        queue_size (int): Size of the incoming message queue (maps to QoS depth)
        **kwargs: Additional arguments (most are ROS1-specific and ignored)

    Returns:
        rclpy.subscription.Subscription: A subscription object
    """
    from .node import _get_node

    node = _get_node()

    # Handle AnyMsg subscriptions
    if msg_type is AnyMsg:
        return _create_anymsg_subscription(node, topic, callback, callback_args, queue_size)

    # Create QoS profile from ROS1 parameters
    qos = create_qos_profile(queue_size=queue_size)

    # Wrap callback to handle callback_args if provided
    if callback is not None:
        if callback_args is not None:
            original_callback = callback

            def wrapped_callback(msg):
                original_callback(msg, callback_args)

            callback = wrapped_callback

    # Create and return the subscription
    subscription = node.create_subscription(msg_type, topic, callback, qos)

    return subscription


def _create_anymsg_subscription(node, topic, callback, callback_args, queue_size):
    """
    Internal helper to create an AnyMsg subscription.

    Args:
        node: ROS2 node
        topic (str): Topic name
        callback (callable): Callback function
        callback_args (tuple): Additional callback arguments
        queue_size (int): Queue size

    Returns:
        rclpy.subscription.Subscription: A subscription
    """
    # For AnyMsg, we need to:
    # 1. Get topic type information
    # 2. Subscribe to raw serialized messages
    # 3. Wrap them to provide _connection_header and serialize() method

    # Try to get topic type from existing publishers
    topic_info = node.get_publishers_info_by_topic(topic)
    msg_type_str = None

    if topic_info:
        # Get the message type from the first publisher
        # Format in ROS2 is like 'std_msgs/msg/String'
        msg_type_str = topic_info[0].topic_type
    else:
        # Topic doesn't exist yet, we'll update this in the callback
        # when we receive the first message
        msg_type_str = 'unknown'

    # Create a wrapper callback that provides rospy.AnyMsg compatibility
    def anymsg_callback(serialized_msg):
        # Update msg_type_str if we didn't have it before
        nonlocal msg_type_str
        if msg_type_str == 'unknown':
            topic_info = node.get_publishers_info_by_topic(topic)
            if topic_info:
                msg_type_str = topic_info[0].topic_type

        # Wrap the serialized message to provide rospy.AnyMsg interface
        wrapped_msg = _AnyMsgWrapper(serialized_msg, topic, msg_type_str)

        # Call user callback
        if callback_args is not None:
            callback(wrapped_msg, callback_args)
        else:
            callback(wrapped_msg)

    # Subscribe to raw serialized messages
    from rclpy.serialization import SerializedMessage
    from rclpy.qos import QoSProfile

    qos = QoSProfile(depth=queue_size)

    # In ROS2, to subscribe to raw messages, we need to use a special subscription
    # that receives SerializedMessage objects
    try:
        # This requires rclpy to support serialized message subscriptions
        subscription = node.create_subscription(
            SerializedMessage,
            topic,
            anymsg_callback,
            qos
        )
    except Exception as e:
        node.get_logger().error(
            f"Failed to create AnyMsg subscription for topic '{topic}': {e}. "
            "AnyMsg support may require specific rclpy version features."
        )
        raise

    return subscription
