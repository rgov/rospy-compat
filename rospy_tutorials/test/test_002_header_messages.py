#!/usr/bin/env python3

"""
Test for Tutorial 002: Messages with Headers

Validates:
- HeaderString custom messages can be published
- Header timestamps are auto-populated (non-zero)
- ROS1-style positional arguments work (no errors)
- Both nodes shut down cleanly

ROS Foxy Compatibility Notes:
- Uses manual subscription + spinning instead of WaitForTopics
- WaitForTopics added in Humble - when migrating to Humble+, replace
  manual subscription code with:
  from launch_testing_ros import WaitForTopics
  with WaitForTopics([('/chatter_header', HeaderString)], timeout=10.0): pass
"""

import unittest
import pytest
import time

import rclpy
import launch
import launch.actions
from launch.substitutions import FindExecutable
import launch_testing.actions
import launch_testing.markers

from rospy_tutorials.msg import HeaderString


@pytest.mark.launch_testing
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch header talker and listener nodes for testing."""

    return launch.LaunchDescription([
        # Launch header talker node in separate process
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'talker_header.py'
            ],
            output='screen',
            name='talker_header'
        ),

        # Launch header listener node in separate process
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'listener_header.py'
            ],
            output='screen',
            name='listener_header'
        ),

        # Signal that setup is complete and tests can start
        launch_testing.actions.ReadyToTest()
    ])


class TestHeaderMessages(unittest.TestCase):
    """Active tests that run while nodes are running."""

    def test_header_messages_published(self, proc_output):
        """
        Test that talker_header publishes HeaderString messages within 10 seconds.

        Also validates that header timestamps are auto-populated (non-zero).
        This confirms the import hook system is working correctly.

        Foxy: Uses manual subscription + spinning.
        Humble+: Replace with WaitForTopics utility.
        """
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create test node to subscribe
        node = rclpy.create_node('test_header_messages')

        # Variable to store received message
        received_msg = []

        def callback(msg):
            received_msg.append(msg)

        # Subscribe to the header topic
        subscription = node.create_subscription(
            HeaderString,
            '/chatter_header',
            callback,
            10
        )

        # Spin for up to 10 seconds waiting for a message
        start_time = time.time()
        while len(received_msg) == 0 and (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Clean up
        node.destroy_subscription(subscription)
        node.destroy_node()

        # Verify we got a message
        self.assertGreater(len(received_msg), 0, "No HeaderString messages received within 10 seconds")

        # Verify header timestamp is non-zero (auto-populated by import hooks)
        msg = received_msg[0]
        timestamp_nonzero = (msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0)
        self.assertTrue(
            timestamp_nonzero,
            f"Header timestamp was not auto-populated (sec={msg.header.stamp.sec}, nanosec={msg.header.stamp.nanosec})"
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Tests that run after nodes have shut down."""

    def test_exit_code(self, proc_info):
        """Verify both nodes exited cleanly."""
        # Foxy: Accept SIGINT/SIGTERM exit codes
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2, -15],
                f"Process {info.process_name} exited with unexpected code {info.returncode}"
            )
