#!/usr/bin/env python3
# Test Time and Duration functionality
# Some tests require a node to be initialized.

import sys


def test_time_construction():
    import rospy

    # Basic construction
    t = rospy.Time(10, 500000000)
    assert t.secs == 10, "Expected secs=10, got %s" % t.secs
    assert t.nsecs == 500000000, "Expected nsecs=500000000, got %s" % t.nsecs
    print("OK: Time construction")

    # Float seconds
    t = rospy.Time.from_sec(10.5)
    assert t.secs == 10, "Expected secs=10, got %s" % t.secs
    assert abs(t.nsecs - 500000000) < 1000, (
        "Expected nsecs~=500000000, got %s" % t.nsecs
    )
    print("OK: Time.from_sec")


def test_time_methods():
    import rospy

    t = rospy.Time(10, 500000000)

    # to_sec
    sec = t.to_sec()
    assert abs(sec - 10.5) < 0.0001, "Expected 10.5, got %s" % sec
    print("OK: Time.to_sec")

    # to_nsec
    nsec = t.to_nsec()
    assert nsec == 10500000000, "Expected 10500000000, got %s" % nsec
    print("OK: Time.to_nsec")

    # is_zero
    assert not t.is_zero(), "Expected not is_zero"
    assert rospy.Time(0, 0).is_zero(), "Expected is_zero for Time(0,0)"
    print("OK: Time.is_zero")


def test_time_arithmetic():
    import rospy

    t1 = rospy.Time(10, 0)
    t2 = rospy.Time(5, 0)
    d = rospy.Duration(3, 0)

    # Time - Time = Duration
    diff = t1 - t2
    # Check duck typing: has secs/nsecs and to_sec (Duration-like)
    assert hasattr(diff, "secs"), "Expected Duration-like with secs"
    assert hasattr(diff, "to_sec"), "Expected Duration-like with to_sec"
    assert diff.secs == 5, "Expected 5, got %s" % diff.secs
    print("OK: Time - Time = Duration")

    # Time + Duration = Time
    result = t1 + d
    # Check duck typing: has secs/nsecs (Time-like)
    assert hasattr(result, "secs"), "Expected Time-like with secs"
    assert result.secs == 13, "Expected 13, got %s" % result.secs
    print("OK: Time + Duration = Time")


def test_time_comparison():
    import rospy

    t1 = rospy.Time(10, 0)
    t2 = rospy.Time(5, 0)
    t3 = rospy.Time(10, 0)

    assert t1 > t2, "Expected t1 > t2"
    assert t2 < t1, "Expected t2 < t1"
    assert t1 >= t3, "Expected t1 >= t3"
    assert t1 <= t3, "Expected t1 <= t3"
    assert t1 == t3, "Expected t1 == t3"
    assert t1 != t2, "Expected t1 != t2"
    print("OK: Time comparison")


def test_duration_construction():
    import rospy

    # Basic construction
    d = rospy.Duration(10, 500000000)
    assert d.secs == 10, "Expected secs=10, got %s" % d.secs
    assert d.nsecs == 500000000, "Expected nsecs=500000000, got %s" % d.nsecs
    print("OK: Duration construction")

    # Float seconds
    d = rospy.Duration.from_sec(10.5)
    assert d.secs == 10, "Expected secs=10, got %s" % d.secs
    assert abs(d.nsecs - 500000000) < 1000, (
        "Expected nsecs~=500000000, got %s" % d.nsecs
    )
    print("OK: Duration.from_sec")


def test_duration_methods():
    import rospy

    d = rospy.Duration(10, 500000000)

    # to_sec
    sec = d.to_sec()
    assert abs(sec - 10.5) < 0.0001, "Expected 10.5, got %s" % sec
    print("OK: Duration.to_sec")

    # to_nsec
    nsec = d.to_nsec()
    assert nsec == 10500000000, "Expected 10500000000, got %s" % nsec
    print("OK: Duration.to_nsec")

    # is_zero
    assert not d.is_zero(), "Expected not is_zero"
    assert rospy.Duration(0, 0).is_zero(), "Expected is_zero for Duration(0,0)"
    print("OK: Duration.is_zero")


def test_duration_arithmetic():
    import rospy

    d1 = rospy.Duration(10, 0)
    d2 = rospy.Duration(3, 0)

    # Duration + Duration
    result = d1 + d2
    assert hasattr(result, "secs"), "Expected Duration-like with secs"
    assert result.secs == 13, "Expected 13, got %s" % result.secs
    print("OK: Duration + Duration")

    # Duration - Duration
    result = d1 - d2
    assert hasattr(result, "secs"), "Expected Duration-like with secs"
    assert result.secs == 7, "Expected 7, got %s" % result.secs
    print("OK: Duration - Duration")

    # Duration * scalar
    result = d1 * 2
    assert hasattr(result, "secs"), "Expected Duration-like with secs"
    assert result.secs == 20, "Expected 20, got %s" % result.secs
    print("OK: Duration * scalar")

    # Duration / scalar
    result = d1 / 2
    assert hasattr(result, "secs"), "Expected Duration-like with secs"
    assert result.secs == 5, "Expected 5, got %s" % result.secs
    print("OK: Duration / scalar")

    # -Duration
    result = -d1
    assert hasattr(result, "secs"), "Expected Duration-like with secs"
    assert result.to_sec() == -10, "Expected -10, got %s" % result.to_sec()
    print("OK: -Duration")


def test_duration_comparison():
    import rospy

    d1 = rospy.Duration(10, 0)
    d2 = rospy.Duration(5, 0)
    d3 = rospy.Duration(10, 0)

    assert d1 > d2, "Expected d1 > d2"
    assert d2 < d1, "Expected d2 < d1"
    assert d1 >= d3, "Expected d1 >= d3"
    assert d1 <= d3, "Expected d1 <= d3"
    assert d1 == d3, "Expected d1 == d3"
    assert d1 != d2, "Expected d1 != d2"
    print("OK: Duration comparison")


def test_canonicalization():
    """Verify that Time/Duration auto-canonicalize (normalize sec/nsec)."""
    import rospy

    # Test 1: nsecs overflow (>= 1e9 should carry to secs)
    d = rospy.Duration(1, 1500000000)  # 1.5 billion nsecs
    assert d.secs == 2, "Expected secs=2 after canonicalization, got %s" % d.secs
    assert d.nsecs == 500000000, "Expected nsecs=500000000, got %s" % d.nsecs
    print("OK: Duration canonicalization (overflow)")

    # Test 2: negative nsecs should borrow from secs
    d = rospy.Duration(2, -500000000)
    assert d.secs == 1, "Expected secs=1 after canonicalization, got %s" % d.secs
    assert d.nsecs == 500000000, "Expected nsecs=500000000, got %s" % d.nsecs
    print("OK: Duration canonicalization (negative nsecs)")

    # Test 3: Time overflow
    t = rospy.Time(1, 2000000000)  # 2 billion nsecs
    assert t.secs == 3, "Expected secs=3 after canonicalization, got %s" % t.secs
    assert t.nsecs == 0, "Expected nsecs=0, got %s" % t.nsecs
    print("OK: Time canonicalization")


def test_arithmetic_canonicalization():
    """Verify arithmetic results are canonicalized (the main bug fix)."""
    import rospy

    # Test 1: Addition with nsec overflow
    d1 = rospy.Duration(1, 500000000)  # 1.5 sec
    d2 = rospy.Duration(1, 600000000)  # 1.6 sec
    result = d1 + d2  # Should be 3.1 sec, not 2 sec + 1.1 billion nsec
    assert result.secs == 3, "Expected secs=3, got %s" % result.secs
    assert result.nsecs == 100000000, "Expected nsecs=100000000, got %s" % result.nsecs
    print("OK: Duration + Duration canonicalization")

    # Test 2: Subtraction with negative nsec result
    d1 = rospy.Duration(2, 0)
    d2 = rospy.Duration(1, 100000000)
    result = d1 - d2  # Should be 0.9 sec
    assert result.secs == 0, "Expected secs=0, got %s" % result.secs
    assert result.nsecs == 900000000, "Expected nsecs=900000000, got %s" % result.nsecs
    print("OK: Duration - Duration canonicalization")

    # Test 3: Time + Duration with overflow
    t = rospy.Time(1, 800000000)
    d = rospy.Duration(0, 500000000)
    result = t + d  # Should be 2.3 sec
    assert result.secs == 2, "Expected secs=2, got %s" % result.secs
    assert result.nsecs == 300000000, "Expected nsecs=300000000, got %s" % result.nsecs
    print("OK: Time + Duration canonicalization")

    # Test 4: Time - Time with negative nsec
    t1 = rospy.Time(2, 0)
    t2 = rospy.Time(1, 100000000)
    result = t1 - t2  # Should be Duration(0, 900000000)
    assert result.secs == 0, "Expected secs=0, got %s" % result.secs
    assert result.nsecs == 900000000, "Expected nsecs=900000000, got %s" % result.nsecs
    print("OK: Time - Time canonicalization")


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node("test_time", anonymous=True)


def test_time_now():
    import rospy

    setup()

    t = rospy.Time.now()
    assert hasattr(t, "secs"), "Time.now() should have secs"
    assert hasattr(t, "nsecs"), "Time.now() should have nsecs"
    assert t.secs >= 0, "Time.now() secs should be non-negative"
    assert t.to_sec() > 0, "Time.now() should return non-zero time"
    print("OK: Time.now()")


def test_get_rostime():
    import rospy

    setup()

    t = rospy.get_rostime()
    assert hasattr(t, "secs"), "get_rostime() should have secs"
    assert hasattr(t, "nsecs"), "get_rostime() should have nsecs"
    assert t.to_sec() > 0, "get_rostime() should return non-zero time"
    print("OK: get_rostime()")


def test_get_time():
    import rospy

    setup()

    t = rospy.get_time()
    assert isinstance(t, float), "get_time() should return float"
    assert t > 0, "get_time() should return positive value"
    print("OK: get_time()")


def test_time_is_builtin_type():
    """Verify rospy.Time/Duration are builtin_interfaces types (ROS2 only)"""
    import rospy

    setup()

    # Skip on ROS1 - builtin_interfaces is ROS2 only
    import importlib.util
    if importlib.util.find_spec('builtin_interfaces') is None:
        print('SKIP: test_time_is_builtin_type (ROS1)')
        return

    t = rospy.Time.now()
    d = rospy.Duration(1, 0)

    # Should be builtin types, not wrapper types
    assert type(t).__module__ == "builtin_interfaces.msg._time", (
        "Time should be builtin type, got %s" % type(t).__module__
    )
    assert type(d).__module__ == "builtin_interfaces.msg._duration", (
        "Duration should be builtin type, got %s" % type(d).__module__
    )

    # Arithmetic should return builtin types
    t2 = t + d
    assert type(t2).__module__ == "builtin_interfaces.msg._time", (
        "Time + Duration should return builtin Time, got %s" % type(t2).__module__
    )

    d2 = t - t
    assert type(d2).__module__ == "builtin_interfaces.msg._duration", (
        "Time - Time should return builtin Duration, got %s" % type(d2).__module__
    )

    print("OK: Time/Duration are builtin types")


def main():
    failed = 0

    tests = [
        test_time_construction,
        test_time_methods,
        test_time_arithmetic,
        test_time_comparison,
        test_duration_construction,
        test_duration_methods,
        test_duration_arithmetic,
        test_duration_comparison,
        test_canonicalization,
        test_arithmetic_canonicalization,
        test_time_now,
        test_get_rostime,
        test_get_time,
        test_time_is_builtin_type,
    ]

    for test in tests:
        try:
            test()
        except Exception as e:
            print("FAIL: %s - %s" % (test.__name__, e))
            failed += 1

    if failed:
        print("\n%d test(s) FAILED" % failed)
        sys.exit(1)
    else:
        print("\nAll tests PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()
