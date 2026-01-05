# Tutorial 006: Parameters - Changes from ROS1

## Files Modified
- `param_talker.py`

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

All ROS1 parameter patterns work thanks to rospy:
- ✅ `rospy.get_param()` with default values
- ✅ `rospy.set_param()` for setting parameters
- ✅ `rospy.has_param()` for checking existence
- ✅ `rospy.delete_param()` (logs warning, no-op in ROS2)
- ✅ `rospy.search_param()` (simplified implementation)
- ✅ `rospy.get_param_names()` for listing parameters
- ✅ `rospy.resolve_name()` for name resolution

## Known Limitations

- **Parameter scope:** ROS2 parameters are node-scoped, not global
- **delete_param():** Logs warning (ROS2 doesn't support deleting declared parameters)
- **search_param():** Simplified implementation (ROS2 has no hierarchical parameter search)
- **Name resolution:** Basic implementation, may differ from ROS1 in complex namespace scenarios
