# Tutorial 008: On Shutdown - Changes from ROS1

## Files Modified
- `publish_on_shutdown.py`

## Changes Made

### 1. Shebang Update
**Original:** `#!/usr/bin/env python`
**Changed to:** `#!/usr/bin/env python3`
**Reason:** ROS2 uses Python 3

### 2. Import Statement
**Original:** `import rospy_compat`
**Changed to:** `import rospy_compat as rospy`
**Reason:** Use compatibility layer for ROS2

## No Other Changes Required

All ROS1 shutdown hook patterns work thanks to rospy:
- ✅ `rospy.on_shutdown()` for registering shutdown callbacks
- ✅ `rospy.signal_shutdown()` for triggering shutdown
- ✅ `rospy.is_shutdown()` for checking shutdown state
- ✅ `rospy.core.add_preshutdown_hook()` for pre-shutdown hooks
- ✅ Publishing messages in shutdown callbacks

## Shutdown Hook Features

rospy implements ROS1-compatible shutdown hooks:
- Callbacks execute when node shuts down (Ctrl+C, signal_shutdown, etc.)
- Pre-shutdown hooks execute before regular shutdown hooks
- Full compatibility with ROS1 shutdown patterns
