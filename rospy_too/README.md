# rospy_too

A rospy compatibility layer for ROS 2. Run unmodified ROS 1 Python nodes on ROS 2.

## Quick Start

```python
# Your existing ROS1 code just works
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
- Services: `Service`, `ServiceProxy`, `wait_for_service`
- Parameters: `get_param`, `set_param`, `has_param`, `delete_param`, `search_param`
- Time: `Time`, `Duration`, `Rate`, `Timer`, `get_time`, `get_rostime`, `sleep`
- Logging: `loginfo`, `logwarn`, `logerr`, `logfatal` (plus `_once`, `_throttle`, `_throttle_identical`)
- Names: `get_name`, `get_namespace`, `resolve_name`, `get_caller_id`
- Messages: Positional constructor args, `AnyMsg`, automatic `Header` defaulting
- Remappings: `__name:=`, `__ns:=`, `myargv()`

## Deviations from ROS 1

### Parameters

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

### AnyMsg

**Foxy only: Publisher must exist before creating AnyMsg subscriber.** On Foxy, we use `raw=True` subscriptions which require knowing the message type. The subscriber polls for up to 2 seconds to discover the type from existing publishers. If none found, raises `ROSException`.

Rolling/Humble+ use `SerializedMessage` and don't have this limitation.

### Timer

**Timer cannot catch up on missed deadlines.** ROS1's thread-based Timer fires rapidly when behind schedule to catch up. ROS2's executor-based timers fire at fixed intervals. If a callback takes longer than the period, subsequent callbacks are delayed but not queued.

The `reset=True` option skips missed deadlines and reschedules from the current time. Default (`reset=False`) maintains accurate `current_expected` tracking.

### Time

**`Time` and `Duration` are the actual `builtin_interfaces` types**, not wrappers. Arithmetic and comparison operators are monkey-patched onto the classes. This means `isinstance(t, rospy.Time)` works and messages serialize correctly without conversion.

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

### uint8[] Fields

**`uint8[]` and `char[]` fields are coerced to `bytes` on read.** ROS 1 stores these as Python `bytes`, but ROS 2 may return `list` or `array.array`. For compatibility, reading these fields always returns `bytes`.

**`numpy.ndarray` assignment requires explicit conversion.** ROS 1 accepts numpy arrays for `uint8[]` fields, but this layer requires explicit conversion: `msg.data = arr.tobytes()`.

## Not Implemented

- `rospy.wait_for_time()` - no sim time support
- `rospy.get_published_topics()` - use `ros2 topic list`
- `rospy.get_master()` - no master in ROS 2
- `rospy.spin_once()` - not needed, executor runs in background thread
- Message filters, TF1, actionlib (use ROS 2 equivalents)

## Testing

Tests run in containers to verify compatibility across ROS versions:

```bash
./tests/run.sh noetic   # Verify code works on ROS 1
./tests/run.sh foxy     # ROS 2 Foxy
./tests/run.sh rolling  # ROS 2 Rolling
./tests/run.sh all      # All of the above
```
