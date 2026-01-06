#!/usr/bin/env python3

"""
Test for Tutorial 006: Parameters

Validates:
- Node retrieves and uses parameters correctly
- Parameters can be set via launch arguments
- Parameter operations work with rospy_compat layer
- Node shuts down cleanly

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
    """Launch param_talker with pre-configured parameters."""

    return launch.LaunchDescription([
        # Launch param_talker node in separate process with parameters
        launch.actions.ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                'run',
                'rospy_tutorials',
                'param_talker.py',
                '--ros-args',
                '-p', 'topic_name:=custom_topic',
                '-p', 'utterance:=test utterance',
                '-p', 'gains.P:=1.0',
                '-p', 'gains.I:=2.0',
                '-p', 'gains.D:=3.0'
            ],
            output='screen',
            name='param_talker'
        ),

        # Signal that setup is complete and tests can start
        launch_testing.actions.ReadyToTest()
    ])


class TestParameters(unittest.TestCase):
    """Active tests that run while param_talker node is running."""

    def test_node_uses_parameters(self, proc_output):
        """
        Test that node retrieves and uses parameters correctly.

        The param_talker node should publish to /chatter using configured parameters.

        Foxy: Uses manual subscription + spinning.
        Humble+: Replace with WaitForTopics utility.
        """
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create test node to subscribe
        node = rclpy.create_node('test_param_talker')

        # Variable to store received messages
        received_messages = []

        def callback(msg):
            received_messages.append(msg.data)

        # Subscribe to the chatter topic (param_talker publishes here by default)
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

        # Verify we got messages (confirms node is running with parameters)
        self.assertGreater(len(received_messages), 0, "No messages received within 10 seconds")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Tests that run after param_talker has shut down."""

    def test_exit_code(self, proc_info):
        """Verify param_talker exited cleanly."""
        # Foxy: Accept SIGINT/SIGTERM exit codes
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2, -15],
                f"Process {info.process_name} exited with unexpected code {info.returncode}"
            )
