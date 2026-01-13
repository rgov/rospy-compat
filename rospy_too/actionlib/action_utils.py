"""Utilities for action type introspection and conversion."""


def get_action_type(action_spec):
    """
    Extract the ROS2 action type from an action spec.

    In ROS1, users pass the *Action class (e.g., FibonacciAction).
    In ROS2, the action type is the class itself (e.g., Fibonacci).

    This function handles both cases by checking for the ROS2 action
    attributes (Goal, Result, Feedback, Impl).
    """
    # Check if it's already a ROS2 action type
    if hasattr(action_spec, 'Goal') and hasattr(action_spec, 'Result'):
        return action_spec

    # ROS1-style: ActionSpec has action_goal, action_result, action_feedback
    # We need to find the parent action type
    raise ValueError(
        "Could not determine action type from %s. "
        "Pass the ROS2 action type directly (e.g., Fibonacci, not FibonacciAction)."
        % action_spec
    )


def get_goal_type(action_type):
    """Get the Goal message type from an action type."""
    return action_type.Goal


def get_result_type(action_type):
    """Get the Result message type from an action type."""
    return action_type.Result


def get_feedback_type(action_type):
    """Get the Feedback message type from an action type."""
    return action_type.Feedback


def create_goal(action_type, *args, **kwargs):
    """Create a Goal message for the given action type."""
    goal_type = get_goal_type(action_type)
    return goal_type(*args, **kwargs)


def create_result(action_type, *args, **kwargs):
    """Create a Result message for the given action type."""
    result_type = get_result_type(action_type)
    return result_type(*args, **kwargs)


def create_feedback(action_type, *args, **kwargs):
    """Create a Feedback message for the given action type."""
    feedback_type = get_feedback_type(action_type)
    return feedback_type(*args, **kwargs)
