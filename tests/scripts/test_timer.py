#!/usr/bin/env python3
# Test Timer and Rate functionality
# These tests require a ROS node to be initialized.

import sys
import threading
import time


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node('test_timer', anonymous=True)


def test_timer_basic():
    """Test that Timer fires callback at expected interval."""
    import rospy

    setup()

    called = []
    event = threading.Event()

    def callback(timer_event):
        called.append(timer_event)
        if len(called) >= 3:
            event.set()

    timer = rospy.Timer(rospy.Duration(0.1), callback)

    # Wait for callbacks
    if not event.wait(timeout=5.0):
        timer.shutdown()
        raise AssertionError('Timeout waiting for timer callbacks')

    timer.shutdown()
    assert len(called) >= 3, 'Expected at least 3 callbacks, got %d' % len(called)
    print('OK: Timer basic')


def test_timer_event_fields():
    """Test that TimerEvent has expected attributes."""
    import rospy

    setup()

    events = []
    event = threading.Event()

    def callback(timer_event):
        events.append(timer_event)
        event.set()

    timer = rospy.Timer(rospy.Duration(0.1), callback)

    if not event.wait(timeout=5.0):
        timer.shutdown()
        raise AssertionError('Timeout waiting for timer callback')

    timer.shutdown()

    te = events[0]
    # TimerEvent should have these attributes
    assert hasattr(te, 'last_expected'), 'TimerEvent should have last_expected'
    assert hasattr(te, 'last_real'), 'TimerEvent should have last_real'
    assert hasattr(te, 'current_expected'), 'TimerEvent should have current_expected'
    assert hasattr(te, 'current_real'), 'TimerEvent should have current_real'
    assert hasattr(te, 'last_duration'), 'TimerEvent should have last_duration'

    # current_expected and current_real should be Time objects
    assert hasattr(te.current_expected, 'secs'), 'current_expected should be Time-like'
    assert hasattr(te.current_real, 'secs'), 'current_real should be Time-like'

    print('OK: TimerEvent fields')


def test_timer_oneshot():
    """Test that oneshot=True fires only once."""
    import rospy

    setup()

    called = []

    def callback(timer_event):
        called.append(timer_event)

    timer = rospy.Timer(rospy.Duration(0.1), callback, oneshot=True)

    # Wait enough time for multiple callbacks if it wasn't oneshot
    time.sleep(0.5)

    timer.shutdown()

    assert len(called) == 1, 'Oneshot timer should fire exactly once, got %d' % len(called)
    print('OK: Timer oneshot')


def test_timer_shutdown():
    """Test that Timer.shutdown() stops the timer."""
    import rospy

    setup()

    called = []

    def callback(timer_event):
        called.append(timer_event)

    timer = rospy.Timer(rospy.Duration(0.05), callback)

    # Wait for some callbacks
    time.sleep(0.2)
    count_before_shutdown = len(called)

    # Shutdown
    timer.shutdown()

    # Wait more
    time.sleep(0.2)
    count_after_shutdown = len(called)

    # Should have stopped after shutdown
    assert count_before_shutdown > 0, 'Should have callbacks before shutdown'
    assert count_after_shutdown == count_before_shutdown, (
        'Should not have new callbacks after shutdown: %d vs %d' %
        (count_before_shutdown, count_after_shutdown)
    )
    print('OK: Timer.shutdown()')


def test_timer_with_float_period():
    """Test Timer accepts float period (rospy_too extension, not ROS1)."""
    import rospy

    setup()

    # On ROS1, Timer only accepts Duration, not float.
    # This is a rospy_too extension for convenience.
    # Detect ROS1 by checking for rclpy
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print('SKIP: Timer with float period (ROS1 requires Duration)')
        return

    called = []
    event = threading.Event()

    def callback(timer_event):
        called.append(timer_event)
        event.set()

    # Pass float instead of Duration
    timer = rospy.Timer(0.1, callback)

    if not event.wait(timeout=5.0):
        timer.shutdown()
        raise AssertionError('Timeout waiting for timer callback')

    timer.shutdown()
    assert len(called) >= 1, 'Timer with float period should work'
    print('OK: Timer with float period')


def test_rate_remaining():
    """Test Rate.remaining() returns Duration."""
    import rospy

    setup()

    rate = rospy.Rate(10)  # 10 Hz = 0.1s period

    # Before first sleep, remaining should be full period
    remaining = rate.remaining()
    assert hasattr(remaining, 'to_sec'), 'remaining() should return Duration-like'

    # After sleep, check remaining
    rate.sleep()
    remaining = rate.remaining()

    # Remaining should be positive Duration
    assert remaining.to_sec() >= 0, 'remaining should be non-negative'
    assert remaining.to_sec() <= 0.1, 'remaining should be <= period (0.1s)'

    print('OK: Rate.remaining()')


def test_sleep_with_duration():
    """Test rospy.sleep() with Duration argument."""
    import rospy

    setup()

    start = time.time()
    rospy.sleep(rospy.Duration(0.2))
    elapsed = time.time() - start

    assert 0.15 < elapsed < 0.5, 'Expected ~0.2s sleep, got %.2fs' % elapsed
    print('OK: sleep with Duration')


def test_sleep_with_float():
    """Test rospy.sleep() with float argument."""
    import rospy

    setup()

    start = time.time()
    rospy.sleep(0.2)
    elapsed = time.time() - start

    assert 0.15 < elapsed < 0.5, 'Expected ~0.2s sleep, got %.2fs' % elapsed
    print('OK: sleep with float')


def main():
    failed = 0

    tests = [
        test_timer_basic,
        test_timer_event_fields,
        test_timer_oneshot,
        test_timer_shutdown,
        test_timer_with_float_period,
        test_rate_remaining,
        test_sleep_with_duration,
        test_sleep_with_float,
    ]

    for test in tests:
        try:
            test()
        except Exception as e:
            print('FAIL: %s - %s' % (test.__name__, e))
            failed += 1

    if failed:
        print('\n%d test(s) FAILED' % failed)
        sys.exit(1)
    else:
        print('\nAll tests PASSED')
        sys.exit(0)


if __name__ == '__main__':
    main()
