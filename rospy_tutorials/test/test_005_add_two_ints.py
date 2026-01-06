#!/usr/bin/env python3

"""
Test for Tutorial 005: Service Client/Server

Validates:
- Service server advertises /add_two_ints service
- Service calls return correct results
- Both calling styles work (async call with Request object)
- Edge cases handled correctly (zero, negative, large numbers)
- Server shuts down cleanly

ROS Foxy Compatibility Notes:
- Uses client.wait_for_service() method (not node.wait_for_service())
- No significant API changes needed for Humble migration
"""

import unittest
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
import launch
import launch.actions
from launch.substitutions import FindExecutable
import launch_testing.actions
import launch_testing.markers

from rospy_tutorials.srv import AddTwoInts


@pytest.mark.launch_testing
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch add_two_ints service server for testing."""

    return launch.LaunchDescription([
        # Launch service server in separate process
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'add_two_ints_server'
            ],
            output='screen',
            name='add_two_ints_server'
        ),

        # Signal that setup is complete and tests can start
        launch_testing.actions.ReadyToTest()
    ])


class TestAddTwoIntsService(unittest.TestCase):
    """Active tests that run while service server is running."""

    def setUp(self):
        """Create test node and service client before each test."""
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('test_add_two_ints_client')

        # Create service client
        self.client = self.node.create_client(AddTwoInts, '/add_two_ints')

        # Wait for service to become available (10-second timeout)
        # Foxy: Use client.wait_for_service(), not node.wait_for_service()
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.fail('Service /add_two_ints not available after 10 seconds')

    def tearDown(self):
        """Clean up test node after each test."""
        self.node.destroy_client(self.client)
        self.node.destroy_node()

    def _call_service(self, a, b, timeout=10.0):
        """
        Helper method to call the service and return the response.

        Args:
            a: First integer
            b: Second integer
            timeout: Timeout in seconds for the service call

        Returns:
            The service response

        Raises:
            AssertionError if the service call times out or fails
        """
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service asynchronously
        future = self.client.call_async(request)

        # Create executor and spin until complete or timeout
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        # Spin until future completes or timeout
        executor.spin_until_future_complete(future, timeout_sec=timeout)

        if future.done():
            return future.result()
        else:
            self.fail(f'Service call timed out after {timeout} seconds')

    def test_basic_addition(self):
        """Test basic addition: 10 + 20 = 30."""
        response = self._call_service(10, 20)
        self.assertEqual(response.sum, 30, f"Expected 10 + 20 = 30, got {response.sum}")

    def test_edge_cases(self):
        """Test edge cases: zero, negative, and large numbers."""
        test_cases = [
            (0, 0, 0, "zero + zero"),
            (-5, 5, 0, "negative + positive = zero"),
            (100, -50, 50, "positive + negative"),
            (-10, -20, -30, "negative + negative"),
            (999999, 1, 1000000, "large numbers"),
            (2147483647, 0, 2147483647, "max 32-bit int"),
        ]

        for a, b, expected, description in test_cases:
            with self.subTest(case=description):
                response = self._call_service(a, b)
                self.assertEqual(
                    response.sum, expected,
                    f"{description}: Expected {a} + {b} = {expected}, got {response.sum}"
                )

    def test_calling_style_with_request_object(self):
        """Test calling service with explicit Request object."""
        # This tests the formal calling style: client.call_async(Request(...))
        request = AddTwoInts.Request()
        request.a = 15
        request.b = 25

        future = self.client.call_async(request)

        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.spin_until_future_complete(future, timeout_sec=10.0)

        if future.done():
            response = future.result()
            self.assertEqual(response.sum, 40, f"Expected 15 + 25 = 40, got {response.sum}")
        else:
            self.fail('Service call with Request object timed out')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Tests that run after service server has shut down."""

    def test_exit_code(self, proc_info):
        """Verify service server exited cleanly."""
        # Foxy: Accept SIGINT/SIGTERM exit codes
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2, -15],
                f"Process {info.process_name} exited with unexpected code {info.returncode}"
            )
