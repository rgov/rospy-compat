# rospy_compat

ROS1 rospy compatibility layer for ROS2.

## Overview

`rospy_compat` is a compatibility shim that allows ROS1 Python nodes to run in ROS2 with minimal code changes. It provides a rospy-compatible API by wrapping rclpy, enabling smooth migration from ROS1 to ROS2.

**Key benefit:** Migrate your Python nodes by simply changing one import line!

## Installation

This package is designed to be built with colcon in a ROS2 workspace:

```bash
cd /path/to/ros2_ws
colcon build --packages-select rospy_compat
source install/setup.bash
```

## Usage

### Basic Migration

To migrate a ROS1 node to ROS2, simply change your import statement:

**Before (ROS1):**
```python
import rospy
from std_msgs.msg import String

rospy.init_node('my_node')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(String(data="hello"))
    rate.sleep()
```

**After (ROS2 with rospy_compat):**
```python
import rospy_compat as rospy  # <-- ONLY CHANGE
from std_msgs.msg import String

rospy.init_node('my_node')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(String(data="hello"))
    rate.sleep()
```

## Supported Features

### Core Functionality
- ✅ **Node lifecycle:** `init_node()`, `spin()`, `is_shutdown()`, `signal_shutdown()`
- ✅ **Publishers:** `Publisher()` with queue_size and latch support
- ✅ **Subscribers:** `Subscriber()` with callback support
- ✅ **Services:** `Service()` server and `ServiceProxy()` client (synchronous)
- ✅ **Parameters:** `get_param()`, `set_param()`, `has_param()`, etc.
- ✅ **Logging:** All logging levels including throttled variants
- ✅ **Time:** `Time`, `Duration` classes with full arithmetic
- ✅ **Rate control:** `Rate`, `Timer`, `sleep()`
- ✅ **Exceptions:** `ROSInterruptException`, `ROSException`
- ✅ **Shutdown hooks:** `add_preshutdown_hook()`, `on_shutdown()`

### Advanced Features
- ✅ **Actionlib:** Full `SimpleActionServer` and `SimpleActionClient` support
- ✅ **AnyMsg:** Subscribe to topics with unknown message types
- ✅ **Throttled logging:** `logwarn_throttle()`, `loginfo_throttle()`, etc.
- ✅ **Anonymous nodes:** Support for `anonymous=True` parameter
- ✅ **Latched publishers:** Automatic QoS conversion for `latch=True`

## API Reference

### Node Management

```python
# Initialize node
rospy.init_node('node_name', anonymous=False, disable_signals=False)

# Spin (block until shutdown)
rospy.spin()

# Check shutdown status
if rospy.is_shutdown():
    # cleanup

# Request shutdown
rospy.signal_shutdown("reason")

# Register shutdown hook
def cleanup(reason):
    print(f"Shutting down: {reason}")

rospy.on_shutdown(cleanup)
# or
rospy.core.add_preshutdown_hook(cleanup)
```

### Publishing & Subscribing

```python
# Create publisher
pub = rospy.Publisher('topic_name', MsgType, queue_size=10, latch=False)
pub.publish(msg)

# Create subscriber
def callback(msg):
    rospy.loginfo(f"Received: {msg}")

sub = rospy.Subscriber('topic_name', MsgType, callback, queue_size=10)

# Subscribe to any message type
sub = rospy.Subscriber('topic_name', rospy.AnyMsg, callback)
```

### Services

```python
# Service server
def handle_request(req):
    # Process request
    return ResponseType(result=value)

srv = rospy.Service('service_name', ServiceType, handle_request)

# Service client
rospy.wait_for_service('service_name')
client = rospy.ServiceProxy('service_name', ServiceType)
response = client(request_arg)
```

### Actionlib

```python
# Action server
import actionlib

def execute_callback(goal):
    # Process goal
    server.publish_feedback(feedback)

    if server.is_preempt_requested():
        server.set_preempted()
        return

    server.set_succeeded(result)

server = actionlib.SimpleActionServer('action_name', ActionType, execute_callback)
server.start()

# Action client
client = actionlib.SimpleActionClient('action_name', ActionType)
client.wait_for_server()
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
```

### Time & Rate

```python
# Time
now = rospy.Time.now()
time_from_secs = rospy.Time.from_sec(1234567890.5)
duration = rospy.Time.now() - earlier_time

# Duration
dur = rospy.Duration(secs=5, nsecs=500000000)
dur = rospy.Duration.from_sec(5.5)

# Rate control
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # do work
    rate.sleep()

# Timer
def timer_callback(event):
    rospy.loginfo("Timer fired")

timer = rospy.Timer(rospy.Duration(1.0), timer_callback, oneshot=False)

# Sleep
rospy.sleep(1.0)  # Sleep for 1 second
rospy.sleep(rospy.Duration(1.0))  # Same using Duration
```

### Parameters

```python
# Get parameter with default
value = rospy.get_param('~param_name', default_value)
value = rospy.get_param('/global/param', default_value)

# Set parameter
rospy.set_param('~param_name', value)

# Check if parameter exists
if rospy.has_param('~param_name'):
    # ...

# Get node name/namespace
node_name = rospy.get_name()
namespace = rospy.get_namespace()
```

### Logging

```python
# Basic logging
rospy.logdebug("Debug message")
rospy.loginfo("Info message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")
rospy.logfatal("Fatal message")

# Throttled logging (only log once per period)
while not rospy.is_shutdown():
    rospy.logwarn_throttle(5.0, "This warning appears at most once every 5 seconds")
    rospy.loginfo_throttle(1.0, "This info appears at most once per second")
```

## Known Limitations

### Features Not Supported

1. **Global Parameter Server:** ROS2 does not have a global parameter server. Parameters are node-scoped. Cross-node parameter access will not work.

2. **Dynamic Reconfigure:** Not supported. Use ROS2 parameter callbacks instead.

3. **Master API:** No equivalent to `rosnode`, `rostopic` Python APIs. Use ROS2 CLI tools (`ros2 node list`, etc.).

4. **Message Connection Headers:** Only available for `AnyMsg` subscriptions, not regular subscriptions.

5. **Parameter Deletion:** ROS2 does not support deleting declared parameters. `delete_param()` is a no-op.

### Behavioral Differences

1. **Threading Model:** ROS2 callbacks run in a background executor thread, similar to ROS1. This should be transparent for most code.

2. **Service Call Timeouts:** Service calls have a default 10-second timeout. Adjust if needed.

3. **Anonymous Node Names:** Node name format may differ slightly from ROS1 due to ROS2 naming restrictions.

4. **Signal Handling:** `disable_signals=True` is best-effort due to ROS2's different signal handling.

5. **AnyMsg Deserialization:** Requires manual deserialization using the message type, as ROS2 doesn't provide automatic type introspection.

## Testing with Asyncio-based Nodes

The compatibility layer works with asyncio-based nodes. The background executor thread is compatible with `asyncio.run_coroutine_threadsafe()` patterns used in your existing code (e.g., `winch_node.py`, `aml_ctd_node.py`).

## Troubleshooting

### Import Errors

If you see import errors related to `rclpy` or `action_msgs`:

```bash
# Make sure ROS2 is sourced
source /opt/ros/foxy/setup.bash  # or your ROS2 distro

# Rebuild the package
colcon build --packages-select rospy_compat
source install/setup.bash
```

### Actionlib Not Available

If actionlib import fails, ensure `action_msgs` is installed:

```bash
sudo apt install ros-foxy-action-msgs  # or your distro
```

### Parameters Not Persisting

Remember that in ROS2, parameters are node-scoped and don't persist after the node shuts down. Use parameter files or launch files to set parameters at startup.

### Service Timeouts

If service calls are timing out, the default timeout is 10 seconds. For longer operations, you may need to modify the timeout in `services.py` or handle timeouts explicitly.

## Architecture Notes

### Global Node Singleton

The package uses a single global node instance (safe given the single-node-per-process assumption for this project). The node is lazily initialized on first API call.

### Background Executor

A background `SingleThreadedExecutor` runs in a daemon thread to process callbacks (timers, subscriptions, services). This matches ROS1's threading behavior where callbacks run in separate threads.

### QoS Profiles

ROS1 parameters are automatically converted to ROS2 QoS profiles:
- `queue_size` → QoS `depth`
- `latch=True` → QoS `TRANSIENT_LOCAL` durability
- Default reliability: `RELIABLE`

## Migration Best Practices

1. **Test incrementally:** Migrate one node at a time and test thoroughly.

2. **Check parameters:** Review parameter usage and ensure they're scoped correctly.

3. **Verify timing:** Test rate-sensitive code as executor threading may have subtle timing differences.

4. **Update message imports:** Remember to import messages from their ROS2 package locations.

5. **Test with real hardware:** If your nodes interact with hardware, test in the actual deployment environment.

## Contributing

This package was developed specifically for the phyto-arm2 project. If you find bugs or have improvements, please update the code accordingly.

## License

MIT License

## Credits

Developed for the phyto-arm2 robotics project to ease migration from ROS1 Noetic to ROS2 Foxy.
