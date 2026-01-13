# Rate control and timers for rospy compatibility.

import time as python_time

from .exceptions import ROSInterruptException
from .impl.node import is_shutdown
from .impl.pending import register_node_init_callback
from .time import Duration, Time


class Rate:
    def __init__(self, hz, reset=False):
        self.sleep_dur = 1.0 / hz
        self.last_time = None
        self._reset = reset

    def sleep(self):
        if self.last_time is None:
            self.last_time = python_time.time()
            return

        elapsed = python_time.time() - self.last_time
        remaining = self.sleep_dur - elapsed

        if remaining > 0:
            end_time = python_time.time() + remaining
            while python_time.time() < end_time:
                if is_shutdown():
                    raise ROSInterruptException('rospy shutdown')
                python_time.sleep(min(end_time - python_time.time(), 0.1))

        if is_shutdown():
            raise ROSInterruptException('rospy shutdown')

        self.last_time = python_time.time()

    def remaining(self):
        if self.last_time is None:
            return Duration.from_sec(self.sleep_dur)
        remaining = self.sleep_dur - (python_time.time() - self.last_time)
        return Duration.from_sec(max(0, remaining))


class TimerEvent:
    def __init__(
        self,
        last_expected=None,
        last_real=None,
        current_expected=None,
        current_real=None,
        last_duration=None,
    ):
        self.last_expected = last_expected
        self.last_real = last_real
        self.current_expected = current_expected or Time.now()
        self.current_real = current_real or Time.now()
        self.last_duration = last_duration


class Timer:
    def __init__(self, period, callback, oneshot=False, reset=False):
        self._period = (
            period.to_sec() if hasattr(period, 'to_sec') else float(period)
        )
        self._callback = callback
        self._oneshot = oneshot
        self._reset = reset
        self._timer = None
        self._cancelled = False
        self._last_expected = None
        self._last_real = None
        self._last_duration = None
        self._current_expected = None
        register_node_init_callback(self)

    def _after_node_init(self, node):
        start = Time.now()
        self._current_expected = start + Duration.from_sec(self._period)
        self._timer = node.create_timer(self._period, self._wrapped_callback)

    def _wrapped_callback(self):
        if self._cancelled:
            return

        current_real = Time.now()
        start_wall = python_time.time()

        event = TimerEvent(
            last_expected=self._last_expected,
            last_real=self._last_real,
            current_expected=self._current_expected,
            current_real=current_real,
            last_duration=self._last_duration,
        )

        self._callback(event)

        self._last_duration = python_time.time() - start_wall
        self._last_expected = self._current_expected
        self._last_real = current_real

        # Calculate next expected time by incrementing (not from now)
        next_expected = self._current_expected + Duration.from_sec(self._period)

        # Handle reset flag: if next_expected is in the past, reset to now + period
        if self._reset and next_expected < current_real:
            next_expected = current_real + Duration.from_sec(self._period)

        self._current_expected = next_expected

        if self._oneshot:
            self.shutdown()

    def shutdown(self):
        if not self._cancelled:
            self._cancelled = True
            if self._timer is not None:
                self._timer.cancel()


def sleep(duration):
    if hasattr(duration, 'to_sec'):
        sleep_time = duration.to_sec()
    else:
        sleep_time = float(duration)

    if sleep_time <= 0:
        if is_shutdown():
            raise ROSInterruptException('rospy shutdown')
        return

    end_time = python_time.time() + sleep_time
    while python_time.time() < end_time:
        if is_shutdown():
            raise ROSInterruptException('rospy shutdown')
        remaining = min(end_time - python_time.time(), 0.1)
        if remaining > 0:
            python_time.sleep(remaining)
