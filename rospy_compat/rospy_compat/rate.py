"""
Rate control and timers for compatibility with rospy.
"""

import time as python_time
from .time import Duration


class Rate:
    """
    Convenience class for sleeping in a loop at a specified rate.
    Compatible with rospy.Rate.
    """

    def __init__(self, hz):
        """
        Create a Rate instance.

        Args:
            hz (float): Frequency in Hz
        """
        self.sleep_duration = 1.0 / hz
        self.last_time = None

    def sleep(self):
        """
        Sleep for the amount of time remaining in the cycle.
        """
        from .node import is_shutdown

        if self.last_time is None:
            self.last_time = python_time.time()
            return

        current_time = python_time.time()
        elapsed = current_time - self.last_time
        remaining = self.sleep_duration - elapsed

        if remaining > 0 and not is_shutdown():
            python_time.sleep(remaining)

        self.last_time = python_time.time()

    def remaining(self):
        """
        Get the time remaining until the next expected sleep time.

        Returns:
            Duration: Time remaining
        """
        if self.last_time is None:
            return Duration.from_sec(self.sleep_duration)

        current_time = python_time.time()
        elapsed = current_time - self.last_time
        remaining = self.sleep_duration - elapsed

        return Duration.from_sec(max(0, remaining))


class TimerEvent:
    """
    Event object passed to Timer callbacks.
    Compatible with rospy.TimerEvent.
    """

    def __init__(self):
        from .time import Time
        # current_real: time the callback actually occurred
        self.current_real = Time.now()
        # current_expected: time the callback was expected to occur
        self.current_expected = Time.now()
        # last_real: last time callback occurred
        self.last_real = None
        # last_expected: last expected time
        self.last_expected = None


class Timer:
    """
    Convenience class for calling a callback at a specified rate.
    Compatible with rospy.Timer.
    """

    def __init__(self, period, callback, oneshot=False):
        """
        Create a Timer instance.

        Args:
            period (Duration or float): Period between callbacks. If float, interpreted as seconds.
            callback (callable): Function to call. Receives TimerEvent as argument.
            oneshot (bool): If True, timer fires only once and then stops.
        """
        from .node import _get_node

        self.oneshot = oneshot
        self.callback = callback
        self._cancelled = False

        # Convert period to seconds
        if hasattr(period, 'to_sec'):
            period_sec = period.to_sec()
        else:
            period_sec = float(period)

        node = _get_node()
        self._timer = node.create_timer(period_sec, self._wrapped_callback)

    def _wrapped_callback(self):
        """Internal callback wrapper"""
        if self._cancelled:
            return

        # Create TimerEvent object (ROS1 compatibility)
        event = TimerEvent()

        # Call user callback with TimerEvent
        try:
            self.callback(event)
        except Exception as e:
            from .node import _get_node
            _get_node().get_logger().error(f"Exception in timer callback: {e}")

        # If oneshot, cancel after first call
        if self.oneshot:
            self.shutdown()

    def shutdown(self):
        """
        Stop the timer.
        """
        if not self._cancelled:
            self._cancelled = True
            if self._timer is not None:
                self._timer.cancel()


def sleep(duration):
    """
    Sleep for the specified duration.

    Args:
        duration (Duration, float, or int): Time to sleep. If Duration, uses to_sec().
                                            If float/int, interpreted as seconds.
    """
    from .node import is_shutdown

    # Convert to seconds
    if hasattr(duration, 'to_sec'):
        sleep_time = duration.to_sec()
    else:
        sleep_time = float(duration)

    # Sleep, checking for shutdown periodically
    if sleep_time <= 0:
        return

    end_time = python_time.time() + sleep_time

    while python_time.time() < end_time and not is_shutdown():
        remaining = end_time - python_time.time()
        # Sleep in small increments to check shutdown more frequently
        sleep_increment = min(remaining, 0.1)
        if sleep_increment > 0:
            python_time.sleep(sleep_increment)
