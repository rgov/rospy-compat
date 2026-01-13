## Purpose

This project implements a transitional package for running completely unmodified ROS 1 nodes, written in Python and based on rospy, under ROS 2. The compatibility layer translates rospy calls to rclpy while keeping all of the original rospy semantics, insofar as they are possible.

(The project and package are called 'rospy-too', and the exported Python module is called 'rospy'; so some clarity is required. When just 'rospy' is referenced, it means the original ROS 1 rospy package.)

## Targeted Versions

The API being reimplemented is from ROS Noetic.

The ROS 2 distributions targeted are Foxy and Rolling: The code must run unmodified and correctly on both releases. Note that Foxy uses Python 3.8, and Rolling currently uses 3.12.3.

## Audience

The point of this compatibility layer is to assist in porting, so the end consumer is presumably a knowledgeable ROS programmer. The library needs to serve their needs for porting, it is NOT a production-quality runtime library and should not be written in an 'enterprise' style.

## Coding Style

Program like an expert Python hacker with deep knowledge of the language.

Use `ruff` to format code with 80 column lines and single quotes for strings. Exception: Do not reformat code copied from genpy or the tutorial code. Fix linter issues that arise.

Do not use docstrings. Do not write verbose comments that are obvious; the desired behavior is already documented by rospy, so we only need to document any potential compatibility deviations or more complicated parts of the implementation.

Do not use type hints.

Keep code concise and minimal. In all cases try to reduce cyclomatic complexity.Follow conventions like PEP 8. Do not use temporary variables, or intermediate variables only written and read once.

Do not put import statements inside functions unless strictly necessary to avoid cylical dependencies. DO NOT!

Keep the code organized. The top-level modules exported by rospy-too should follow those of the original rospy. The impl subpackage can have whatever you want. Please don't prefix stuff with underscores in the impl subpackage.

## Available Code References

Do not modify these files.

refs/ros_comm/clients/rospy/src/rospy/: Original ROS Noetic rospy sources
refs/genpy/src/genpy: Sources of genpy for Noetic release

refs/rclpy-rolling/rclpy/src/rclpy/: Sources of rclpy for Rolling release
refs/rclpy-foxy/rclpy/src/rclpy/: Sources of rclpy for Foxy release

refs/rospy2: Sources to a similar project implemented in just 500 lines

## Testing

Tests must be run in ros containers (ros:noetic, ros:foxy, ros:rolling), not on the host. You can use `podman run --rm` to make temporary containers.

Each test case must finish in 10 seconds, plenty of time to send and receive a message between nodes.

Our test cases should prove we can run the same code unmodified under ROS 1 and both ROS 2 releases. (To this end, we might do something like have shell ros1 and ros2 package directories and then volume mount the sources.)

The container should always build the catkin/colcon from scratch and not persist anything to the host.

Example workflow:
```bash
./tests/run.sh noetic   # Verify ROS1 baseline
./tests/run.sh foxy     # Verify ROS2 Foxy compatibility
./tests/run.sh rolling  # Verify ROS2 Rolling compatibility
./tests/run.sh all      # Run all three
```

## Testing Invariants

**ROS1 is the behavioral reference.** When ROS2 behavior differs from ROS1, match ROS1. Run tests on Noetic first to establish expected behavior, then verify Foxy and Rolling match.

**Test on all three platforms.** Always run:
```bash
./tests/run.sh noetic   # ROS1 reference behavior
./tests/run.sh foxy     # ROS2 Foxy
./tests/run.sh rolling  # ROS2 Rolling
```

**Use pytest for assertions.** The test runner installs pytest in containers. For exception testing, use `pytest.raises()`:
```python
import pytest

def test_missing_param_raises():
    with pytest.raises(KeyError):
        rospy.get_param('/nonexistent')
```

**Test structure.** Tests are standalone functions (not unittest.TestCase classes). Each test file has a `main()` that runs all `test_*` functions. The custom runner (`run_all.py`) imports and executes these.