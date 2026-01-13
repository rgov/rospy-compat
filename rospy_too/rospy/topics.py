# Publisher and Subscriber classes for rospy compatibility.
import importlib
import threading
import time as python_time

from rclpy.qos import QoSProfile

from .exceptions import ROSException, ROSInitException
from .impl.node import _get_node
from .impl.pending import register_node_init_callback
from .impl.qos import create_qos_profile
from .logging import logwarn
from .msg import AnyMsg

# Try to import SerializedMessage (rolling/humble+), fall back to raw=True (foxy+)
try:
    from rclpy.serialization import SerializedMessage
except ImportError:
    SerializedMessage = None


def _resolve_topic(node, name):
    # Resolve topic name using rclpy's resolver with fallback
    try:
        return node.resolve_topic_name(name, only_expand=False)
    except (AttributeError, Exception):
        from .names import resolve_name
        return resolve_name(name)


class SubscribeListener:
    def __init__(self):
        logwarn(
            'SubscribeListener is not fully supported in ROS2. '
            'peer_subscribe/peer_unsubscribe callbacks will not be invoked.'
        )

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        pass

    def peer_unsubscribe(self, topic_name, num_peers):
        pass


class Publisher:
    def __init__(
        self,
        topic,
        msg_type,
        subscriber_listener=None,
        tcp_nodelay=False,
        latch=False,
        headers=None,
        queue_size=10,
    ):
        self._topic = topic
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._latch = latch
        self._subscriber_listener = subscriber_listener
        self._publisher = None
        register_node_init_callback(self)

    def _after_node_init(self, node):
        resolved = _resolve_topic(node, self._topic)
        qos = create_qos_profile(queue_size=self._queue_size, latch=self._latch)
        self._publisher = node.create_publisher(self._msg_type, resolved, qos)

    def publish(self, msg):
        if self._publisher is None:
            raise ROSInitException(
                'rospy.init_node() must be called before publishing'
            )

        if isinstance(msg, self._msg_type):
            self._publisher.publish(msg)
        else:
            # Auto-convert single-field messages (like std_msgs/String)
            # Cache the 'data' field check to avoid repeated instantiation
            if not hasattr(self, '_has_data_field'):
                self._has_data_field = 'data' in getattr(
                    self._msg_type, '__slots__', []
                ) or (
                    hasattr(self._msg_type, 'get_fields_and_field_types')
                    and 'data' in self._msg_type.get_fields_and_field_types()
                )

            if self._has_data_field:
                msg_obj = self._msg_type()
                msg_obj.data = msg
                self._publisher.publish(msg_obj)
            else:
                raise TypeError(
                    f'Cannot auto-convert {type(msg)} to {self._msg_type}'
                )

    def get_num_connections(self):
        if self._publisher is None:
            return 0
        return self._publisher.get_subscription_count()

    def unregister(self):
        if self._publisher is None:
            return
        _get_node().destroy_publisher(self._publisher)
        self._publisher = None

    def __getattr__(self, name):
        if self._publisher is None:
            raise AttributeError(
                f'{type(self).__name__} has no attribute {name!r}'
            )
        return getattr(self._publisher, name)


class Subscriber:
    def __init__(
        self,
        topic,
        msg_type,
        callback=None,
        callback_args=None,
        queue_size=10,
        buff_size=65536,
        tcp_nodelay=False,
        latch=False,
    ):
        self._topic = topic
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._latch = latch
        self._callbacks = (
            [(callback, callback_args)] if callback is not None else []
        )
        self._subscription = None
        register_node_init_callback(self)

    def _after_node_init(self, node):
        resolved = _resolve_topic(node, self._topic)
        if self._msg_type is AnyMsg:
            self._subscription = _create_anymsg_subscription(
                node, resolved, self._invoke_callbacks, None, self._queue_size
            )
        else:
            qos = create_qos_profile(
                queue_size=self._queue_size, latch=self._latch
            )
            self._subscription = node.create_subscription(
                self._msg_type, resolved, self._invoke_callbacks, qos
            )

    def _invoke_callbacks(self, msg):
        for callback, callback_args in self._callbacks:
            if callback_args is not None:
                callback(msg, callback_args)
            else:
                callback(msg)

    def add_callback(self, callback, callback_args=None):
        self._callbacks.append((callback, callback_args))

    def get_num_connections(self):
        if self._subscription is None:
            return 0
        return self._subscription.get_publisher_count()

    def unregister(self):
        if self._subscription is None:
            return
        _get_node().destroy_subscription(self._subscription)
        self._subscription = None


class _AnyMsgWrapper:
    def __init__(self, serialized_msg, topic_name, msg_type_str):
        self._buff = bytes(serialized_msg)
        self._connection_header = {
            'type': msg_type_str,
            'topic': topic_name,
            'message_definition': '',
            'callerid': '',
            'latching': '0',
        }

    def serialize(self, buff=None):
        if buff is None:
            return self._buff
        buff.write(self._buff)
        return buff.getvalue()


def _import_message_type(type_str):
    # Handle both 'pkg/msg/Type' and 'pkg/Type' formats
    parts = type_str.split('/')
    if len(parts) == 3:
        pkg, _, msg_name = parts
    elif len(parts) == 2:
        pkg, msg_name = parts
    else:
        raise ValueError(f'Invalid message type format: {type_str}')

    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, msg_name)


def _create_anymsg_subscription(
    node, topic, callback, callback_args, queue_size
):
    # Uses SerializedMessage on Rolling/Humble+ or raw=True on Foxy
    qos = QoSProfile(depth=queue_size)

    if SerializedMessage is not None:
        # Rolling/Humble+: use SerializedMessage type
        topic_info = node.get_publishers_info_by_topic(topic)
        msg_type_str = topic_info[0].topic_type if topic_info else 'unknown'

        def anymsg_callback(serialized_msg):
            nonlocal msg_type_str
            if msg_type_str == 'unknown':
                info = node.get_publishers_info_by_topic(topic)
                if info:
                    msg_type_str = info[0].topic_type

            wrapped = _AnyMsgWrapper(serialized_msg, topic, msg_type_str)
            if callback_args is not None:
                callback(wrapped, callback_args)
            else:
                callback(wrapped)

        return node.create_subscription(
            SerializedMessage, topic, anymsg_callback, qos
        )

    # Foxy: use raw=True to get bytes directly
    # We must discover the actual message type from publishers
    msg_type = None
    msg_type_str = 'unknown'

    # Poll for up to 2 seconds to discover publisher type
    for _ in range(40):
        topic_info = node.get_publishers_info_by_topic(topic)
        if topic_info:
            # Warn if multiple publishers have different types
            types = set(info.topic_type for info in topic_info)
            if len(types) > 1:
                logwarn(
                    f'AnyMsg: Multiple publishers with different types on '
                    f'{topic}: {types}'
                )
            msg_type_str = topic_info[0].topic_type
            try:
                msg_type = _import_message_type(msg_type_str)
                break
            except (ImportError, AttributeError, ValueError):
                pass
        python_time.sleep(0.05)

    if msg_type is None:
        raise ROSException(
            f'AnyMsg: Could not discover message type for {topic}. '
            f'Ensure a publisher exists before creating AnyMsg subscriber.'
        )

    def anymsg_callback_foxy(raw_bytes):
        wrapped = _AnyMsgWrapper(raw_bytes, topic, msg_type_str)
        if callback_args is not None:
            callback(wrapped, callback_args)
        else:
            callback(wrapped)

    return node.create_subscription(
        msg_type, topic, anymsg_callback_foxy, qos, raw=True
    )


def wait_for_message(topic, topic_type, timeout=None):
    result = [None]
    event = threading.Event()

    def callback(msg):
        result[0] = msg
        event.set()

    sub = Subscriber(topic, topic_type, callback)

    if timeout is not None:
        success = event.wait(timeout=timeout)
    else:
        # Use loop with short timeouts to allow signal handling
        # (event.wait() without timeout can't be interrupted on macOS)
        while not event.is_set():
            event.wait(timeout=1.0)
        success = True

    sub.unregister()

    if not success:
        raise ROSException('Timeout waiting for message on %s' % topic)

    return result[0]
