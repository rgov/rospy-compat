"""
Time and Duration classes for compatibility with rospy.
"""

import rclpy.time
import rclpy.duration


class Time:
    """
    Wrapper around rclpy.time.Time to provide rospy-compatible API.
    """

    def __init__(self, secs=0, nsecs=0):
        """
        Create a Time instance.

        Args:
            secs (int): Seconds component
            nsecs (int): Nanoseconds component
        """
        total_nsecs = int(secs) * 1000000000 + int(nsecs)
        self._time = rclpy.time.Time(nanoseconds=total_nsecs)

    @staticmethod
    def now():
        """
        Get the current time.

        Returns:
            Time: Current time from the node's clock
        """
        from .node import _get_node
        node = _get_node()
        return Time._from_rclpy_time(node.get_clock().now())

    @staticmethod
    def from_sec(float_secs):
        """
        Create a Time from a float representing seconds.

        Args:
            float_secs (float): Time in seconds

        Returns:
            Time: Time instance
        """
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1e9)
        return Time(secs, nsecs)

    @staticmethod
    def _from_rclpy_time(rclpy_time):
        """Internal helper to create Time from rclpy.time.Time"""
        t = Time()
        t._time = rclpy_time
        return t

    @property
    def secs(self):
        """Get the seconds component"""
        return self._time.nanoseconds // 1000000000

    @property
    def nsecs(self):
        """Get the nanoseconds component"""
        return self._time.nanoseconds % 1000000000

    def to_sec(self):
        """
        Convert to float seconds.

        Returns:
            float: Time in seconds
        """
        return self._time.nanoseconds / 1e9

    def to_nsec(self):
        """
        Convert to integer nanoseconds.

        Returns:
            int: Time in nanoseconds
        """
        return self._time.nanoseconds

    def __sub__(self, other):
        """
        Subtract two Times to get a Duration.

        Args:
            other (Time): Time to subtract

        Returns:
            Duration: Time difference
        """
        diff_ns = self._time.nanoseconds - other._time.nanoseconds
        return Duration._from_nanoseconds(diff_ns)

    def __add__(self, other):
        """
        Add a Duration to a Time.

        Args:
            other (Duration): Duration to add

        Returns:
            Time: New time
        """
        if isinstance(other, Duration):
            new_ns = self._time.nanoseconds + other._duration.nanoseconds
            return Time._from_rclpy_time(rclpy.time.Time(nanoseconds=new_ns))
        return NotImplemented

    def __eq__(self, other):
        """Check equality"""
        if not isinstance(other, Time):
            return False
        return self._time.nanoseconds == other._time.nanoseconds

    def __ne__(self, other):
        """Check inequality"""
        return not self.__eq__(other)

    def __lt__(self, other):
        """Less than comparison"""
        return self._time.nanoseconds < other._time.nanoseconds

    def __le__(self, other):
        """Less than or equal comparison"""
        return self._time.nanoseconds <= other._time.nanoseconds

    def __gt__(self, other):
        """Greater than comparison"""
        return self._time.nanoseconds > other._time.nanoseconds

    def __ge__(self, other):
        """Greater than or equal comparison"""
        return self._time.nanoseconds >= other._time.nanoseconds

    def __repr__(self):
        return f"Time(secs={self.secs}, nsecs={self.nsecs})"


class Duration:
    """
    Wrapper around rclpy.duration.Duration to provide rospy-compatible API.
    """

    def __init__(self, secs=0, nsecs=0):
        """
        Create a Duration instance.

        Args:
            secs (int/float): Seconds component
            nsecs (int): Nanoseconds component
        """
        total_nsecs = int(secs) * 1000000000 + int(nsecs)
        self._duration = rclpy.duration.Duration(nanoseconds=total_nsecs)

    @staticmethod
    def from_sec(float_secs):
        """
        Create a Duration from a float representing seconds.

        Args:
            float_secs (float): Duration in seconds

        Returns:
            Duration: Duration instance
        """
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1e9)
        return Duration(secs, nsecs)

    @staticmethod
    def _from_nanoseconds(nsecs):
        """Internal helper to create Duration from nanoseconds"""
        d = Duration()
        d._duration = rclpy.duration.Duration(nanoseconds=nsecs)
        return d

    @property
    def secs(self):
        """Get the seconds component"""
        return self._duration.nanoseconds // 1000000000

    @property
    def nsecs(self):
        """Get the nanoseconds component"""
        return self._duration.nanoseconds % 1000000000

    def to_sec(self):
        """
        Convert to float seconds.

        Returns:
            float: Duration in seconds
        """
        return self._duration.nanoseconds / 1e9

    def to_nsec(self):
        """
        Convert to integer nanoseconds.

        Returns:
            int: Duration in nanoseconds
        """
        return self._duration.nanoseconds

    def __add__(self, other):
        """Add two Durations"""
        if isinstance(other, Duration):
            new_ns = self._duration.nanoseconds + other._duration.nanoseconds
            return Duration._from_nanoseconds(new_ns)
        return NotImplemented

    def __sub__(self, other):
        """Subtract two Durations"""
        if isinstance(other, Duration):
            new_ns = self._duration.nanoseconds - other._duration.nanoseconds
            return Duration._from_nanoseconds(new_ns)
        return NotImplemented

    def __mul__(self, other):
        """Multiply Duration by a scalar"""
        if isinstance(other, (int, float)):
            new_ns = int(self._duration.nanoseconds * other)
            return Duration._from_nanoseconds(new_ns)
        return NotImplemented

    def __div__(self, other):
        """Divide Duration by a scalar"""
        if isinstance(other, (int, float)):
            new_ns = int(self._duration.nanoseconds / other)
            return Duration._from_nanoseconds(new_ns)
        return NotImplemented

    __truediv__ = __div__  # Python 3 compatibility

    def __eq__(self, other):
        """Check equality"""
        if not isinstance(other, Duration):
            return False
        return self._duration.nanoseconds == other._duration.nanoseconds

    def __ne__(self, other):
        """Check inequality"""
        return not self.__eq__(other)

    def __lt__(self, other):
        """Less than comparison"""
        return self._duration.nanoseconds < other._duration.nanoseconds

    def __le__(self, other):
        """Less than or equal comparison"""
        return self._duration.nanoseconds <= other._duration.nanoseconds

    def __gt__(self, other):
        """Greater than comparison"""
        return self._duration.nanoseconds > other._duration.nanoseconds

    def __ge__(self, other):
        """Greater than or equal comparison"""
        return self._duration.nanoseconds >= other._duration.nanoseconds

    def __repr__(self):
        return f"Duration(secs={self.secs}, nsecs={self.nsecs})"
