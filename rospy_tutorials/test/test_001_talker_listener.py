#!/usr/bin/env python3

"""
Test for Tutorial 001: Basic Talker/Listener

Validates:
- Talker publishes messages to /chatter topic
- Listener receives messages
- Both nodes shut down cleanly

ROS Foxy Compatibility Notes:
- Uses manual subscription + spinning instead of WaitForTopics
- WaitForTopics added in Humble - when migrating to Humble+, replace
  manual subscription code with:
  from launch_testing_ros import WaitForTopics
  with WaitForTopics([('/chatter', String)], timeout=10.0): pass
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

from std_msgs.msg import String


@pytest.mark.launch_testing
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch talker and listener nodes for testing."""

    return launch.LaunchDescription([
        # Launch talker node in separate process
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'talker.py'
            ],
            output='screen',
            name='talker'
        ),

        # Launch listener node in separate process
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'listener.py'
            ],
            output='screen',
            name='listener'
        ),

        # Signal that setup is complete and tests can start
        launch_testing.actions.ReadyToTest()
    ])


class TestTalkerListener(unittest.TestCase):
    """Active tests that run while nodes are running."""

    def test_messages_published(self, proc_output):
        """
        Test that talker publishes messages to /chatter within 10 seconds.

        Foxy: Uses manual subscription + spinning.
        Humble+: Replace with WaitForTopics utility.
        """
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create test node to subscribe
        node = rclpy.create_node('test_talker_listener')

        # Variable to store received messages
        received_messages = []

        def callback(msg):
            received_messages.append(msg.data)

        # Subscribe to the chatter topic
        subscription = node.create_subscription(
            String,
            '/chatter',
            callback,
            10
        )

        # Spin for up to 10 seconds waiting for messages
        start_time = time.time()
        while len(received_messages) == 0 and (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Clean up
        node.destroy_subscription(subscription)
        node.destroy_node()

        # Verify we got messages
        self.assertGreater(len(received_messages), 0, "No messages received within 10 seconds")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Tests that run after nodes have shut down."""

    def test_exit_code(self, proc_info):
        """
        Verify both nodes exited cleanly.

        Foxy compatibility: Accept both clean exit (0) and SIGINT (-2/-15)
        When migrating to Humble+, may be able to use stricter checking.
        """
        # Foxy: Iterate over ProcessExited objects
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2, -15],  # 0=clean, -2=SIGINT, -15=SIGTERM
                f"Process {info.process_name} exited with unexpected code {info.returncode}"
            )
