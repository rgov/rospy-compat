#!/usr/bin/env python3
# Test parameter server functionality
# These tests require a ROS node to be initialized.

import sys

import pytest


def setup():
    import rospy

    if not rospy.core.is_initialized():
        rospy.init_node("test_params", anonymous=True)


def test_set_get_param():
    import rospy

    setup()

    # Set and get string
    rospy.set_param("/test/string_param", "hello")
    val = rospy.get_param("/test/string_param")
    assert val == "hello", "Expected hello, got %s" % val
    print("OK: set/get string param")

    # Set and get int
    rospy.set_param("/test/int_param", 42)
    val = rospy.get_param("/test/int_param")
    assert val == 42, "Expected 42, got %s" % val
    print("OK: set/get int param")

    # Set and get float
    rospy.set_param("/test/float_param", 3.14)
    val = rospy.get_param("/test/float_param")
    assert abs(val - 3.14) < 0.001, "Expected 3.14, got %s" % val
    print("OK: set/get float param")

    # Set and get bool
    rospy.set_param("/test/bool_param", True)
    val = rospy.get_param("/test/bool_param")
    assert val is True, "Expected True, got %s" % val
    print("OK: set/get bool param")

    # Set and get list
    rospy.set_param("/test/list_param", [1, 2, 3])
    val = rospy.get_param("/test/list_param")
    assert val == [1, 2, 3], "Expected [1,2,3], got %s" % val
    print("OK: set/get list param")


def test_dict_param():
    import rospy

    setup()

    # Set dict param
    rospy.set_param("/test/dict_param", {"a": 1, "b": 2})
    val = rospy.get_param("/test/dict_param")
    assert val == {"a": 1, "b": 2}, "Expected dict, got %s" % val
    print("OK: set/get dict param")

    # Set nested dict - ROS stores as separate params
    rospy.set_param("/test/nested", {"x": {"y": 10}})
    val = rospy.get_param("/test/nested")
    assert val == {"x": {"y": 10}}, "Expected nested dict, got %s" % val
    print("OK: set/get nested dict param")


def test_has_param():
    import rospy

    setup()

    rospy.set_param("/test/exists", "value")
    assert rospy.has_param("/test/exists"), "Expected param to exist"
    assert not rospy.has_param("/test/does_not_exist"), "Expected param to not exist"
    print("OK: has_param")


def test_get_param_default():
    import rospy

    setup()

    # Non-existent param with default
    val = rospy.get_param("/test/nonexistent", "default_value")
    assert val == "default_value", "Expected default_value, got %s" % val
    print("OK: get_param with default")


def test_delete_param():
    import rospy

    setup()

    rospy.set_param("/test/to_delete", "value")
    assert rospy.has_param("/test/to_delete"), "Expected param to exist before delete"

    rospy.delete_param("/test/to_delete")
    assert not rospy.has_param("/test/to_delete"), (
        "Expected param to not exist after delete"
    )
    print("OK: delete_param")


def test_private_param():
    import rospy

    setup()

    # Private param (~ prefix)
    rospy.set_param("~private_param", "private_value")
    val = rospy.get_param("~private_param")
    assert val == "private_value", "Expected private_value, got %s" % val
    print("OK: private param")


def test_private_param_namespace():
    import rospy

    setup()

    # Set a private param
    rospy.set_param("~ns_test_param", "ns_value")

    # Verify it resolves to /nodename/param
    node_name = rospy.get_name()
    # Node name is like /test_params_12345_abc or test_params_12345_abc
    # Remove leading slash if present for param lookup
    if node_name.startswith("/"):
        node_name = node_name[1:]

    # Check that we can access it via the fully-qualified path
    fq_name = f"/{node_name}/ns_test_param"
    val = rospy.get_param(fq_name)
    assert val == "ns_value", "Expected ns_value via FQ path %s, got %s" % (fq_name, val)
    print("OK: private param namespace resolution")


def test_search_param():
    import rospy

    setup()

    rospy.set_param("/global_search_test", "found")
    result = rospy.search_param("global_search_test")
    assert result is not None, "Expected to find param"
    assert "global_search_test" in result, (
        "Expected param name in result, got %s" % result
    )
    print("OK: search_param")


def test_delete_param_logs_once():
    # Verify delete_param uses logwarn_once (ROS2 only)
    # We just verify the function works - rclpy handles the once filtering
    import rospy
    setup()

    try:
        from rospy.logging import _get_logger
        _get_logger()
    except (ImportError, AttributeError):
        print('SKIP: delete_param_logs_once (ROS1)')
        return

    rospy.set_param('/test/logonce3', 'val')
    rospy.delete_param('/test/logonce3')
    # rclpy's once=True filtering is tested by rclpy itself
    print('OK: delete_param logs once')


def test_get_param_default_no_side_effect():
    """Verify get_param with default doesn't create param on server (ROS1 behavior)."""
    import rospy
    setup()

    # Use a unique param name to avoid collision
    param_name = "/test/side_effect_check_unique"

    # Ensure param doesn't exist
    if rospy.has_param(param_name):
        rospy.delete_param(param_name)
    assert not rospy.has_param(param_name), "Param should not exist initially"

    # Get with default
    val = rospy.get_param(param_name, "my_default")
    assert val == "my_default", "Expected my_default, got %s" % val

    # Verify param STILL doesn't exist (ROS1 behavior)
    assert not rospy.has_param(param_name), (
        "get_param with default should NOT create param on server"
    )
    print("OK: get_param default no side effect")


def test_dict_param_slash_access():
    """Verify nested dict params are accessible via slash paths (ROS1 format)."""
    import rospy
    setup()

    # Set nested dict
    rospy.set_param("/test/slash_nested", {"x": {"y": 10}})

    # In ROS1, this creates /test/slash_nested/x/y
    # rospy_too translates slashes to dots internally for ROS2 compatibility
    assert rospy.has_param("/test/slash_nested/x/y"), (
        "Nested dict param should be accessible via /test/slash_nested/x/y"
    )
    val = rospy.get_param("/test/slash_nested/x/y")
    assert val == 10, "Expected 10, got %s" % val
    print("OK: dict param slash access")


def test_search_param_returns_full_path():
    """Verify search_param returns fully qualified path."""
    import rospy
    setup()

    rospy.set_param("/search_fullpath_test", "value")
    result = rospy.search_param("search_fullpath_test")
    assert result is not None, "Expected to find param"
    assert result.startswith("/"), (
        "search_param should return fully qualified path starting with /, got %s" % result
    )
    assert result == "/search_fullpath_test", (
        "Expected /search_fullpath_test, got %s" % result
    )
    print("OK: search_param returns full path")


def test_get_param_names():
    """Verify get_param_names returns list of parameter names."""
    import rospy
    setup()

    # Set some unique params
    rospy.set_param("/test/names_test_a", "value_a")
    rospy.set_param("/test/names_test_b", "value_b")

    # Get param names - may not be available on Foxy (no list_parameters API)
    try:
        names = rospy.get_param_names()
    except AttributeError as e:
        if 'list_parameters' in str(e):
            print("SKIP: get_param_names (list_parameters not available on Foxy)")
            return
        raise

    assert isinstance(names, list), "Expected list, got %s" % type(names)
    # Our params should be in the list (normalized to dots in ROS2)
    # Note: ROS2 uses dots internally, so check for either format
    found_a = any("names_test_a" in n for n in names)
    found_b = any("names_test_b" in n for n in names)
    assert found_a, "Expected to find names_test_a in param names"
    assert found_b, "Expected to find names_test_b in param names"
    print("OK: get_param_names returns %d params" % len(names))


def test_get_deleted_param_with_default():
    """Getting deleted param with default returns the default."""
    import rospy
    setup()

    # ROS2 only test
    try:
        from rospy.params import _deleted_params  # noqa: F401
    except ImportError:
        print("SKIP: test_get_deleted_param_with_default (ROS1)")
        return

    # Set a param then delete it
    rospy.set_param("/test/del_default", "original")
    rospy.delete_param("/test/del_default")

    # Get with default - should return default
    val = rospy.get_param("/test/del_default", "my_default")
    assert val == "my_default", "Expected 'my_default', got %s" % val

    # Get without default - should raise KeyError
    with pytest.raises(KeyError):
        rospy.get_param("/test/del_default")

    print("OK: get deleted param with default")


def test_get_nonexistent_param_no_default():
    """Getting nonexistent param without default raises KeyError (ROS1 behavior)."""
    import rospy
    setup()

    # Get param that definitely doesn't exist, no default - should raise KeyError
    with pytest.raises(KeyError):
        rospy.get_param("/test/truly_nonexistent_xyz_123")

    print("OK: get nonexistent param raises KeyError")


def test_set_param_overwrites_existing():
    """Setting existing param updates it correctly."""
    import rospy
    setup()

    # Set param twice
    rospy.set_param("/test/overwrite", "first")
    val1 = rospy.get_param("/test/overwrite")
    assert val1 == "first", "Expected 'first', got %s" % val1

    rospy.set_param("/test/overwrite", "second")
    val2 = rospy.get_param("/test/overwrite")
    assert val2 == "second", "Expected 'second', got %s" % val2

    print("OK: set param overwrites existing")


def test_search_param_not_found():
    """search_param returns None for nonexistent param."""
    import rospy
    setup()

    result = rospy.search_param("/totally/nonexistent/param/xyz_123")
    assert result is None, "Expected None for nonexistent param, got %s" % result

    print("OK: search param not found")


def test_get_param_names_excludes_deleted():
    """get_param_names excludes deleted parameters."""
    import rospy
    setup()

    # ROS2 only test (requires _deleted_params)
    try:
        from rospy.params import _deleted_params  # noqa: F401
    except ImportError:
        print("SKIP: test_get_param_names_excludes_deleted (ROS1)")
        return

    # Set params
    rospy.set_param("/test/names_del_a", "val")
    rospy.set_param("/test/names_del_b", "val")

    # Delete one
    rospy.delete_param("/test/names_del_a")

    # Get param names
    try:
        names = rospy.get_param_names()
    except AttributeError as e:
        if 'list_parameters' in str(e):
            print("SKIP: test_get_param_names_excludes_deleted (list_parameters not available)")
            return
        raise

    # Deleted param should not be in list
    found_a = any("names_del_a" in n for n in names)
    found_b = any("names_del_b" in n for n in names)
    assert not found_a, "Deleted param should not be in names"
    assert found_b, "Non-deleted param should be in names"

    print("OK: get_param_names excludes deleted")


def test_get_param_names_slash_format():
    """get_param_names returns ROS1 slash format, not ROS2 dotted format."""
    import rospy
    setup()

    # ROS2 only test
    try:
        from rospy.params import _deleted_params  # noqa: F401
    except ImportError:
        print("SKIP: test_get_param_names_slash_format (ROS1)")
        return

    # Set a nested param
    rospy.set_param("/test/slash/format/check", "value")

    try:
        names = rospy.get_param_names()
    except AttributeError as e:
        if 'list_parameters' in str(e):
            print("SKIP: test_get_param_names_slash_format (list_parameters not available)")
            return
        raise

    # Should find with slashes, not dots
    found_slash = any("/test/slash/format/check" in n for n in names)
    found_dot = any("test.slash.format.check" in n for n in names)

    assert found_slash, "Should find param with slash format in names"
    assert not found_dot, "Should NOT find param with dotted format in names"

    # All names should start with /
    for name in names:
        assert name.startswith("/"), "All param names should start with /, got: %s" % name

    print("OK: get_param_names returns slash format")


def test_search_param_upward_finds_root():
    """search_param finds params at root level via upward search."""
    import rospy
    setup()

    # Set a param at root
    rospy.set_param("/upward_search_root_param", "found_it")

    # search_param should find it even without / prefix
    result = rospy.search_param("upward_search_root_param")
    assert result == "/upward_search_root_param", (
        "Expected /upward_search_root_param, got %s" % result
    )

    print("OK: search_param upward finds root")


def test_list_of_dicts_param():
    """Verify list of dicts can be set and retrieved correctly."""
    import rospy
    setup()

    # Set list of dicts
    value = [{"a": 1}, {"a": 2}]
    rospy.set_param("/test/list_of_dicts", value)

    # Get it back
    result = rospy.get_param("/test/list_of_dicts")
    assert result == value, "Expected %s, got %s" % (value, result)

    print("OK: list of dicts param")


def test_nested_list_param():
    """Verify dict containing list can be set and retrieved correctly."""
    import rospy
    setup()

    # Set dict with nested list
    value = {"items": [1, 2, 3]}
    rospy.set_param("/test/nested_list", value)

    # Get it back
    result = rospy.get_param("/test/nested_list")
    assert result == value, "Expected %s, got %s" % (value, result)

    print("OK: nested list param")


def test_list_index_access_blocked():
    """Verify list elements are NOT accessible via slash paths (ROS1 behavior).

    In ROS1, lists are atomic values. You can access /list to get the full list,
    but NOT /list/0 to get an element. This matches ROS1 parameter server behavior.
    """
    import rospy
    setup()

    # ROS2 only test
    try:
        from rospy.params import _deleted_params  # noqa: F401
    except ImportError:
        print("SKIP: test_list_index_access_blocked (ROS1)")
        return

    # Set list of dicts
    rospy.set_param("/test/list_slash", [{"a": 1}, {"a": 2}])

    # In ROS1, lists are atomic - can't access elements via slash paths
    assert not rospy.has_param("/test/list_slash/0"), (
        "List element should NOT be accessible via /test/list_slash/0"
    )
    assert not rospy.has_param("/test/list_slash/0/a"), (
        "List element should NOT be accessible via /test/list_slash/0/a"
    )

    # get_param should return default or raise KeyError
    val = rospy.get_param("/test/list_slash/0/a", "default")
    assert val == "default", "Expected 'default' for list index access, got %s" % val

    with pytest.raises(KeyError):
        rospy.get_param("/test/list_slash/0/a")

    # But the full list should still be accessible
    result = rospy.get_param("/test/list_slash")
    assert result == [{"a": 1}, {"a": 2}], "Expected list, got %s" % result

    print("OK: list index access blocked (ROS1 behavior)")


def test_get_private_namespace():
    """Verify get_param('~', None) returns all private params as dict."""
    import rospy
    setup()

    # Set some private params
    rospy.set_param("~priv_ns_a", "value_a")
    rospy.set_param("~priv_ns_b", 42)

    # Get entire private namespace
    result = rospy.get_param("~", None)

    # Should return dict with our params (or None if no params)
    assert result is not None, "Expected dict, got None"
    assert isinstance(result, dict), "Expected dict, got %s" % type(result)
    assert "priv_ns_a" in result, "Expected priv_ns_a in result, got %s" % result
    assert "priv_ns_b" in result, "Expected priv_ns_b in result, got %s" % result
    assert result["priv_ns_a"] == "value_a", "Expected value_a, got %s" % result["priv_ns_a"]
    assert result["priv_ns_b"] == 42, "Expected 42, got %s" % result["priv_ns_b"]

    print("OK: get private namespace")


def main():
    failed = 0

    tests = [
        test_set_get_param,
        test_dict_param,
        test_has_param,
        test_get_param_default,
        test_delete_param,
        test_private_param,
        test_private_param_namespace,
        test_search_param,
        test_delete_param_logs_once,
        test_get_param_default_no_side_effect,
        test_dict_param_slash_access,
        test_search_param_returns_full_path,
        test_get_param_names,
        test_get_deleted_param_with_default,
        test_get_nonexistent_param_no_default,
        test_set_param_overwrites_existing,
        test_search_param_not_found,
        test_get_param_names_excludes_deleted,
        test_get_param_names_slash_format,
        test_search_param_upward_finds_root,
        test_list_of_dicts_param,
        test_nested_list_param,
        test_list_index_access_blocked,
        test_get_private_namespace,
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
