# Publisher and Subscriber classes for rospy compatibility.
import importlib
import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from .exceptions import ROSException, ROSInitException
from .impl.hooks import prepare_received_message
from .impl.node import _get_node
from .impl.pending import cancel_node_init_callback, register_node_init_callback
from .impl.qos import create_qos_profile
from .logging import logwarn
from .msg import AnyMsg
from .names import resolve_name

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
        resolved = resolve_name(self._topic)
        qos = create_qos_profile(1 if self._latch else self._queue_size, self._latch)
        self.name = self.resolved_name = resolved
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
        cancel_node_init_callback(self)
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
        self._discovery = None
        self._lock = threading.RLock()
        self._group = MutuallyExclusiveCallbackGroup()
        self._generation = 0
        self._received = False
        register_node_init_callback(self)

    def _after_node_init(self, node):
        self.name = self.resolved_name = resolve_name(self._topic)
        with self._lock:
            publishers = node.get_publishers_info_by_topic(self.name)
            if publishers or self._msg_type is not AnyMsg:
                self._subscribe(node, publishers)
            if not publishers and (not self._latch or self._msg_type is AnyMsg):
                self._discovery = node.create_timer(
                    0.1, lambda: self._discover(node), callback_group=self._group
                )

    def _discover(self, node):
        with self._lock:
            if self._discovery is None:
                return
            publishers = node.get_publishers_info_by_topic(self.name)
            if not publishers:
                return
            if self._subscription is None or (
                not self._received and self._qos(publishers) != self._initial_qos
            ):
                self._subscribe(node, publishers)
            node.destroy_timer(self._discovery)
            self._discovery = None

    def _qos(self, publishers):
        durable = self._latch or bool(publishers) and all(
            endpoint.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            for endpoint in publishers
        )
        reliable = all(
            endpoint.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT
            for endpoint in publishers
        )
        return durable, reliable

    def _subscribe(self, node, publishers):
        msg_type = self._msg_type
        raw = msg_type is AnyMsg
        if raw:
            type_name = publishers[0].topic_type
            msg_type = _import_message_type(type_name)
        elif hasattr(msg_type, '_rospy_wire_type'):
            msg_type = msg_type._rospy_wire_type()

        self._initial_qos = self._qos(publishers)
        durable, reliable = self._initial_qos
        qos = create_qos_profile(self._queue_size, durable, reliable)
        self._generation += 1
        generation = self._generation

        def receive(message):
            with self._lock:
                if generation != self._generation:
                    return
                self._received = True
            if raw:
                message = _AnyMsgWrapper(message, self.name, type_name)
            else:
                message = prepare_received_message(message)
                if hasattr(self._msg_type, '_rospy_from_wire'):
                    message = self._msg_type._rospy_from_wire(message)
                    if message is None:
                        return
            self._invoke_callbacks(message)

        if self._subscription is not None:
            node.destroy_subscription(self._subscription)
        self._subscription = node.create_subscription(
            msg_type, self.name, receive, qos,
            callback_group=self._group, raw=raw,
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
        if hasattr(self._subscription, 'get_publisher_count'):
            return self._subscription.get_publisher_count()
        return _get_node().count_publishers(self.name)

    def unregister(self):
        cancel_node_init_callback(self)
        with self._lock:
            self._generation += 1
            if self._discovery is not None:
                _get_node().destroy_timer(self._discovery)
                self._discovery = None
            if self._subscription is not None:
                _get_node().destroy_subscription(self._subscription)
                self._subscription = None


class _AnyMsgWrapper(AnyMsg):
    def __init__(self, serialized_msg, topic_name, msg_type_str):
        self._buff = bytes(serialized_msg)
        self._connection_header = {
            'type': msg_type_str.replace('/msg/', '/'),
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
