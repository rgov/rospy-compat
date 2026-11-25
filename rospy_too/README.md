# rospy_too

A rospy compatibility layer for ROS 2. Run unmodified ROS 1 Python nodes on ROS 2.

## Quick Start

From the root of a ROS 2 workspace containing this checkout and your application's ROS 2 interfaces, build and source the shim.

```bash
colcon build --packages-up-to rospy_too
source install/setup.bash
```

Build and source your application interfaces before launching the original Python nodes. The shim installs the `rospy` and `actionlib` imports.

```python
import rospy
from std_msgs.msg import String

rospy.init_node('talker')
pub = rospy.Publisher('/chatter', String, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish('hello')
    rate.sleep()
```

## Supported Features

- Node lifecycle: `init_node`, `spin`, `is_shutdown`, `on_shutdown`, `signal_shutdown`
- Topics: `Publisher`, `Subscriber`, `wait_for_message`
- Actions: `actionlib.SimpleActionClient`, `SimpleActionServer`, typed goal and result observers
- Services: `Service`, `ServiceProxy`, `wait_for_service`
- Parameters: `get_param`, `set_param`, `has_param`, `delete_param`, `search_param`
- Time: `Time`, `Duration`, `Rate`, `Timer`, `get_time`, `get_rostime`, `sleep`
- Logging: `loginfo`, `logwarn`, `logerr`, `logfatal` (plus `_once`, `_throttle`, `_throttle_identical`)
- Names: `get_name`, `get_namespace`, `resolve_name`, `get_caller_id`
- Messages: Positional constructor args, `AnyMsg`, automatic `Header` defaulting
- Remappings: `__name:=`, `__ns:=`, `myargv()`

## Deviations from ROS 1

### Startup and shutdown

Publishers, subscribers, timers, services and service proxies can be constructed before `init_node()`. The shim binds them when the node starts. Separate subscriptions and service clients can make progress while another callback waits for a service. Client shutdown hooks run while the node is still usable, followed by preshutdown hooks, then shutdown hooks.

### Parameters

The ROS 2 parameter store belongs to each node. Supply shared configuration to every node that reads it. For a node named `/fleet/driver`, the ROS 1 names `~gain`, `limits/max`, and `/site/id` use the ROS 2 parameter keys `fleet.driver.gain`, `fleet.limits.max`, and `site.id`. `get_namespace()` returns `/fleet/`.

**`delete_param()` only masks the parameter.** ROS 2 doesn't support parameter deletion. The parameter is hidden from `get_param`/`has_param` until re-set, but still exists in the node's parameter store.

### Services

**`persistent=True` is ignored.** ROS 2 service clients don't have a persistent connection mode. A warning is logged.

**Service handler failures don't propagate to clients.** In ROS 1, if a service handler raises an exception or returns `None`, the client receives a `ServiceException`. ROS 2's service transport cannot propagate handler failures back to clients. Instead:
- Server side: Exceptions and `None` returns are caught, logged as errors, and a default (zero-valued) response is returned.
- Client side: A 10-second default timeout prevents infinite hangs. Override with `proxy(request, timeout=N)`.

If you need to signal errors to clients, include an error field in your service response type.

### Topics

**`SubscribeListener` callbacks are not invoked.** `peer_subscribe` and `peer_unsubscribe` have no ROS 2 equivalent. The class exists for API compatibility but does nothing.

**`tcp_nodelay`, `buff_size`, `headers` are ignored.** ROS 2 uses DDS, not TCPROS.

A latched publisher retains its latest sample while it is alive. An ordinary subscriber checks publisher durability and reliability during discovery. When volatile and transient-local publishers are discovered together, it requests volatile delivery, which carries future samples from both. Recreate the subscriber if the mix changes later and a different QoS is needed.

### AnyMsg

An `AnyMsg` subscriber may start before the publisher. On both Foxy and Rolling it discovers the topic type asynchronously. Its connection header supplies `pkg/Type`, and a shim-imported instance of that type can call `deserialize(raw._buff)`. The buffer contains ROS 2 CDR. A ROS 1 TCPROS decoder requires a different wire format. Other connection-header fields provide only placeholders.

### Actions

A shim-hosted `SimpleActionServer` accepts native ROS 2 clients and forwards the application's result and feedback. Legacy observers may subscribe to `<action>/goal` and `<action>/result` using `FooActionGoal` and `FooActionResult`. Matching events share the goal UUID and its acceptance timestamp. The result event also carries the server's status text.

These observer topics use the bundled `rospy_too_interfaces/ActionEvent` type on the ROS 2 graph. They are receive-only from a legacy node's perspective. A server written directly in rclpy does not emit them. Actions support feedback and individual cancellation through the client API. ROS 1 wire-level goal submission, feedback/status/cancel topics, broadcast cancellation and timestamp-based cancellation are outside this subset. The native result response has no status text, so `get_goal_status_text()` returns an empty string. Binary sequences in a received action payload can retain their ROS 2 container type.

### Timer

**Timer cannot catch up on missed deadlines.** ROS1's thread-based Timer fires rapidly when behind schedule to catch up. ROS2's executor-based timers fire at fixed intervals. If a callback takes longer than the period, subsequent callbacks are delayed but not queued.

The `reset=True` option skips missed deadlines and reschedules from the current time. Default (`reset=False`) maintains accurate `current_expected` tracking.

### Time

**`Time` and `Duration` are the actual `builtin_interfaces` types**, not wrappers. Arithmetic and comparison operators are monkey-patched onto the classes. This means `isinstance(t, rospy.Time)` works and messages serialize correctly without conversion.

`Time.now()` follows the ROS 2 node clock. `Rate.sleep()` and `rospy.sleep()` use wall time. `rospy.rostime.get_rostime_cond()` supports wall-clock callers and raises `NotImplementedError` when simulated time is active.

### Logging

**Uses ROS 2 logging backend.** Output format differs slightly from ROS 1. The `_throttle_identical` variants use content hashing rather than ROS 1's exact implementation.

### Message Constructors

**Import hooks wrap message classes** to support positional arguments and `Header` auto-population. The wrapping happens at import time. If you import messages before `import rospy`, they won't be wrapped.

```python
# Do this:
import rospy
from std_msgs.msg import Header

# Not this:
from std_msgs.msg import Header
import rospy  # Too late, Header wasn't wrapped
```

`std_msgs/Header` accepts the ROS 1 `seq` argument. Reading it yields zero because the ROS 2 message has no sequence-number field.

### Binary fields

Shim-imported messages and typed topic and service callbacks expose variable-length `uint8[]` and `char[]` as Python `bytes`, including when nested. Constructors and assignments accept NumPy arrays for numeric sequences. Fixed-size arrays retain the generated ROS 2 representation.

## Not Implemented

- `rospy.wait_for_time()` - no sim time support
- `rospy.get_published_topics()` - use `ros2 topic list`
- `rospy.get_master()` - no master in ROS 2
- `rospy.spin_once()` - not needed, executor runs in background thread
- Message filters and TF1

## Testing

Tests run in containers to verify compatibility across ROS versions:

```bash
./tests/run.sh noetic   # Verify code works on ROS 1
./tests/run.sh foxy     # ROS 2 Foxy
./tests/run.sh rolling  # ROS 2 Rolling
./tests/run.sh all      # All of the above
```
