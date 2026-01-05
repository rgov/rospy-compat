# Tutorial 002: Headers - Changes from ROS1

## Files Modified
- `talker_header.py`
- `listener_header.py`

## Changes Made

### 1. Shebang Update
**Original:** `#!/usr/bin/env python`
**Changed to:** `#!/usr/bin/env python3`
**Reason:** ROS2 uses Python 3

### 2. Import Statement
**Original:** `import rospy_compat`
**Changed to:** `import rospy_compat as rospy`
**Reason:** Use compatibility layer for ROS2

### 3. Message Import
**Original:** `from rospy_tutorials.msg import HeaderString`
**Changed to:** `from rospy_tutorials.msg import HeaderString`
**Reason:** Use ROS2-compatible message package

## No Other Changes Required

All ROS1 code patterns work unchanged thanks to rospy import hooks:
- ✅ Positional arguments: `HeaderString(None, str)` works exactly like ROS1
- ✅ Auto-populated timestamps: Headers created with zero timestamps automatically get current time
- ✅ `header.stamp.to_sec()` method works (monkey-patched)
- ✅ `header.seq` field works (returns 0, ROS2 removed sequence tracking)

## Import Hook Features

The rospy import hook system transparently:
- Intercepts `.msg` module imports
- Enhances message classes to accept positional arguments
- Auto-populates `header.stamp` with current time when headers are zero
- Maintains full backward compatibility with keyword arguments
