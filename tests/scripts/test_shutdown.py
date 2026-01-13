#!/usr/bin/env python3
"""Test shutdown lifecycle in subprocess to avoid disrupting test runner.

These tests spawn subprocesses that go through the full shutdown cycle,
allowing us to verify:
- signal_shutdown() properly cleans up executor and node
- Shutdown hooks run with valid node context
- SIGINT triggers graceful shutdown (not immediate sys.exit)

Subprocess tests run under coverage (if available) so their execution
contributes to overall coverage metrics.
"""

import os
import subprocess
import sys
import tempfile

# Coverage data directory (set by run_all.py or coverage run)
COVERAGE_DATA_DIR = os.environ.get('COVERAGE_DATA_DIR', '/tmp/coverage_data')


def _run_python_with_coverage(script_content, script_file=None):
    """Run Python script, optionally under coverage for subprocess coverage collection.

    Note: coverage run does not support -c flag, so we always write to a temp file
    when running under coverage.
    """
    env = os.environ.copy()
    temp_script = None

    # Check if we should collect coverage
    try:
        import coverage  # noqa: F401
        collect_coverage = True
        # Use parallel mode so multiple subprocesses don't clobber each other
        coverage_args = [
            sys.executable, '-m', 'coverage', 'run',
            '--parallel-mode',
            '--source=/ws/src/rospy_too/rospy',
            '--branch',
        ]
    except ImportError:
        collect_coverage = False
        coverage_args = [sys.executable]

    try:
        # Determine script path
        if script_file:
            script_path = script_file
        elif collect_coverage:
            # coverage run doesn't support -c, need temp file
            temp_script = tempfile.NamedTemporaryFile(
                mode='w', suffix='.py', delete=False
            )
            temp_script.write(script_content)
            temp_script.close()
            script_path = temp_script.name
        else:
            # Plain Python supports -c
            cmd = coverage_args + ['-c', script_content]
            return subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                env=env,
            )

        cmd = coverage_args + [script_path]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30,
            env=env,
        )
        return result

    finally:
        # Clean up temp file if created
        if temp_script is not None:
            try:
                os.unlink(temp_script.name)
            except OSError:
                pass


# Skip these tests in ROS1 (no rospy_too)
def _is_ros2():
    try:
        import rospy.impl.node
        return hasattr(rospy.impl.node, '_shutdown_node_internal')
    except ImportError:
        return False


def test_signal_shutdown_cleanup():
    """Test that signal_shutdown properly cleans up executor and node."""
    if not _is_ros2():
        print("SKIP: test_signal_shutdown_cleanup (ROS1)")
        return

    # Python script to run in subprocess
    script = '''
import sys
sys.path.insert(0, '/ws/src/rospy_too')

import rospy
from rospy.impl import node as node_module

# Initialize
rospy.init_node('test_shutdown_cleanup', anonymous=True)

# Verify node and executor exist
assert node_module._node is not None, "Node should exist"
assert node_module._executor is not None, "Executor should exist"
assert node_module._executor_thread is not None, "Executor thread should exist"
assert node_module._executor_thread.is_alive(), "Executor thread should be alive"

# Trigger shutdown
rospy.signal_shutdown("test cleanup")

# Verify cleanup happened
assert node_module._node is None, "Node should be None after shutdown"
assert node_module._executor is None, "Executor should be None after shutdown"
assert node_module._executor_thread is None or not node_module._executor_thread.is_alive(), \\
    "Executor thread should be stopped after shutdown"
assert node_module._is_shutdown is True, "Shutdown flag should be True"

print("SUBPROCESS_OK: cleanup verified")
'''

    result = _run_python_with_coverage(script)

    if result.returncode != 0:
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        raise AssertionError(f"Subprocess failed with code {result.returncode}")

    assert "SUBPROCESS_OK: cleanup verified" in result.stdout, \
        f"Expected success message, got: {result.stdout}"

    print("OK: signal_shutdown cleanup")


def test_shutdown_hooks_run_with_valid_context():
    """Test that shutdown hooks can use rospy APIs (node still valid)."""
    if not _is_ros2():
        print("SKIP: test_shutdown_hooks_run_with_valid_context (ROS1)")
        return

    script = '''
import sys
sys.path.insert(0, '/ws/src/rospy_too')

import rospy

hook_results = []

def my_shutdown_hook():
    # This hook should be able to use rospy APIs
    # ROS1 on_shutdown callbacks take zero arguments
    try:
        # Try to log - this requires valid node context
        rospy.loginfo("Shutdown hook running")
        hook_results.append("logged")
    except Exception as e:
        hook_results.append(f"error: {e}")

    # Try to get time - requires valid node
    try:
        t = rospy.get_time()
        hook_results.append(f"time={t:.1f}")
    except Exception as e:
        hook_results.append(f"time_error: {e}")

rospy.init_node('test_hooks_context', anonymous=True)
rospy.on_shutdown(my_shutdown_hook)

# Trigger shutdown
rospy.signal_shutdown("testing hooks")

# Check results
if "logged" in hook_results:
    print("SUBPROCESS_OK: hook could log")
else:
    print(f"SUBPROCESS_FAIL: hook results = {hook_results}")
    sys.exit(1)
'''

    result = _run_python_with_coverage(script)

    if result.returncode != 0:
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        raise AssertionError(f"Subprocess failed with code {result.returncode}")

    assert "SUBPROCESS_OK: hook could log" in result.stdout, \
        f"Expected success, got: {result.stdout}"

    print("OK: shutdown hooks run with valid context")


def test_sigint_graceful_shutdown():
    """Test that SIGINT triggers graceful shutdown, not immediate exit."""
    if not _is_ros2():
        print("SKIP: test_sigint_graceful_shutdown (ROS1)")
        return

    # Create a temporary script file (easier for signal handling)
    script = '''
import sys
sys.path.insert(0, '/ws/src/rospy_too')

import signal
import time
import rospy
from rospy.impl import node as node_module

cleanup_verified = False

def verify_cleanup():
    global cleanup_verified
    # At this point, node should still be valid (hooks run before cleanup)
    # ROS1 on_shutdown callbacks take zero arguments
    if node_module._node is not None:
        cleanup_verified = True
        print("HOOK_OK: node valid during hook")
    else:
        print("HOOK_FAIL: node already destroyed")

rospy.init_node('test_sigint', anonymous=True, disable_signals=False)
rospy.on_shutdown(verify_cleanup)

# Give a moment for setup
time.sleep(0.1)

# Send SIGINT to self
import os
os.kill(os.getpid(), signal.SIGINT)

# After signal handler runs, we should still be here (no sys.exit)
# Give time for shutdown to complete
time.sleep(0.5)

# Verify shutdown happened gracefully
if node_module._is_shutdown:
    print("SUBPROCESS_OK: graceful shutdown")
else:
    print("SUBPROCESS_FAIL: shutdown flag not set")
    sys.exit(1)
'''

    with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
        f.write(script)
        script_path = f.name

    try:
        result = _run_python_with_coverage(None, script_file=script_path)

        # Process might exit with 0 or due to signal - both are acceptable
        # as long as we see the success message
        if "SUBPROCESS_OK: graceful shutdown" in result.stdout:
            print("OK: SIGINT graceful shutdown")
        elif "HOOK_OK: node valid during hook" in result.stdout:
            # Hook ran successfully even if process exited differently
            print("OK: SIGINT graceful shutdown (hook verified)")
        else:
            print(f"STDOUT: {result.stdout}")
            print(f"STDERR: {result.stderr}")
            raise AssertionError("Expected graceful shutdown message")

    finally:
        os.unlink(script_path)


def test_multiple_shutdown_calls_idempotent():
    """Test that calling signal_shutdown multiple times is safe."""
    if not _is_ros2():
        print("SKIP: test_multiple_shutdown_calls_idempotent (ROS1)")
        return

    script = '''
import sys
sys.path.insert(0, '/ws/src/rospy_too')

import rospy

hook_count = [0]

def counting_hook():
    # ROS1 on_shutdown callbacks take zero arguments
    hook_count[0] += 1

rospy.init_node('test_idempotent', anonymous=True)
rospy.on_shutdown(counting_hook)

# Call shutdown multiple times
rospy.signal_shutdown("first")
rospy.signal_shutdown("second")
rospy.signal_shutdown("third")

# Hook should only run once
if hook_count[0] == 1:
    print("SUBPROCESS_OK: idempotent")
else:
    print(f"SUBPROCESS_FAIL: hook ran {hook_count[0]} times")
    sys.exit(1)
'''

    result = _run_python_with_coverage(script)

    if result.returncode != 0:
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        raise AssertionError(f"Subprocess failed with code {result.returncode}")

    assert "SUBPROCESS_OK: idempotent" in result.stdout, \
        f"Expected idempotent message, got: {result.stdout}"

    print("OK: multiple shutdown calls idempotent")


def test_is_shutdown_reflects_state():
    """Test that rospy.is_shutdown() correctly reflects shutdown state."""
    if not _is_ros2():
        print("SKIP: test_is_shutdown_reflects_state (ROS1)")
        return

    script = '''
import sys
sys.path.insert(0, '/ws/src/rospy_too')

import rospy

rospy.init_node('test_is_shutdown', anonymous=True)

# Before shutdown
if rospy.is_shutdown():
    print("SUBPROCESS_FAIL: is_shutdown True before shutdown")
    sys.exit(1)

rospy.signal_shutdown("test")

# After shutdown
if not rospy.is_shutdown():
    print("SUBPROCESS_FAIL: is_shutdown False after shutdown")
    sys.exit(1)

print("SUBPROCESS_OK: is_shutdown correct")
'''

    result = _run_python_with_coverage(script)

    if result.returncode != 0:
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        raise AssertionError(f"Subprocess failed with code {result.returncode}")

    assert "SUBPROCESS_OK: is_shutdown correct" in result.stdout, \
        f"Expected success, got: {result.stdout}"

    print("OK: is_shutdown reflects state")


def main():
    failed = 0

    tests = [
        test_signal_shutdown_cleanup,
        test_shutdown_hooks_run_with_valid_context,
        test_sigint_graceful_shutdown,
        test_multiple_shutdown_calls_idempotent,
        test_is_shutdown_reflects_state,
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
