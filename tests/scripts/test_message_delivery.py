#!/usr/bin/env python3
import importlib
import io
import threading
import time

import numpy
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Header, String
from stereo_msgs.msg import DisparityImage


def wait_until(check):
    deadline = time.monotonic() + 3
    while not check():
        if time.monotonic() >= deadline:
            raise AssertionError('Timed out waiting for discovery or delivery')
        time.sleep(0.02)


def test_late_latched_and_early_ordinary_subscribers():
    pub = rospy.Publisher('late', String, latch=True, queue_size=1)
    pub.publish('first')
    result = []
    sub = rospy.Subscriber('late', String, lambda msg: result.append(msg.data))
    wait_until(lambda: result == ['first'])
    pub.publish('second')
    wait_until(lambda: result == ['first', 'second'])
    assert sub.name == rospy.resolve_name('late')
    sub.unregister()
    pub.unregister()

    result.clear()
    sub = rospy.Subscriber('early', String, lambda msg: result.append(msg.data))
    pub = rospy.Publisher('early', String, queue_size=1)
    wait_until(pub.get_num_connections)
    pub.publish('one')
    wait_until(lambda: result == ['one'])
    sub.unregister()
    pub.unregister()


def test_anymsg_round_trip_and_typed_binary_receive():
    typed, raw = [], []
    rospy.Subscriber('binary_raw', rospy.AnyMsg, raw.append)
    rospy.Subscriber('binary', CompressedImage, typed.append)
    pub = rospy.Publisher('binary', CompressedImage, queue_size=1)
    raw_pub = rospy.Publisher('binary_raw', CompressedImage, queue_size=1)

    wait_until(lambda: pub.get_num_connections() and raw_pub.get_num_connections())
    msg = CompressedImage()
    assert isinstance(msg.data, bytes) and msg.data == b''
    msg = CompressedImage(data=b'\x00robot\xff')
    assert msg.data.decode('latin-1') == '\x00robot\xff'
    pub.publish(msg)
    raw_pub.publish(msg)
    wait_until(lambda: typed and raw)

    assert typed[0].data == msg.data
    assert typed[0].data.decode('latin-1') == '\x00robot\xff'
    type_name = raw[0]._connection_header['type']
    assert type_name == 'sensor_msgs/CompressedImage'
    package, name = type_name.split('/')
    cls = getattr(importlib.import_module(package + '.msg'), name)
    buf = io.BytesIO()
    raw[0].serialize(buf)
    restored = cls()
    assert restored.deserialize(buf.getvalue()) is restored
    assert restored.data.decode('latin-1') == '\x00robot\xff'


def test_nested_binary_round_trip():
    msg = DisparityImage()
    msg.image.data = b'nested'
    serialized = io.BytesIO()
    msg.serialize(serialized)
    restored = DisparityImage().deserialize(serialized.getvalue())
    assert restored.image.data.decode() == 'nested'
    restored.image.data = b''
    serialized = io.BytesIO()
    restored.serialize(serialized)
    assert DisparityImage().deserialize(serialized.getvalue()).image.data == b''


def test_profile_arrays_and_sequence_number():
    header = Header()
    header.seq = 42
    msg = Float32MultiArray()
    msg.data = numpy.array([1.5, 2.5])
    buf = io.BytesIO()
    msg.serialize(buf)
    assert list(Float32MultiArray().deserialize(buf.getvalue()).data) == [1.5, 2.5]


def main():
    # Module-scope registration, with no retained Python handle.
    arrived = threading.Event()
    rospy.Subscriber('/pre_init_delivery', String, lambda _: arrived.set())
    rospy.init_node('test_message_delivery', anonymous=True)
    pub = rospy.Publisher('/pre_init_delivery', String, queue_size=1)
    wait_until(pub.get_num_connections)
    pub.publish('pre-init')
    assert arrived.wait(3)
    test_late_latched_and_early_ordinary_subscribers()
    test_anymsg_round_trip_and_typed_binary_receive()
    test_nested_binary_round_trip()
    test_profile_arrays_and_sequence_number()
    print('message delivery passed', flush=True)


if __name__ == '__main__':
    main()
