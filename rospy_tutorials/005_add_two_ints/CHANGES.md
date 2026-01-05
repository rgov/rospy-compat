# Tutorial 005: Add Two Ints (Services) - Changes from ROS1

## Files Modified
- `add_two_ints_server`
- `add_two_ints_client`

## Changes Made

### 1. Shebang Update
**Original:** `#!/usr/bin/env python`
**Changed to:** `#!/usr/bin/env python3`
**Reason:** ROS2 uses Python 3

### 2. Import Order Fix ⚠️  CRITICAL
**Original:**
```python
from rospy_tutorials.srv import *
import rospy_compat
```

**Changed to:**
```python
import rospy_compat as rospy
from rospy_tutorials.srv import *
```

**Reason:** The `rospy` import MUST come BEFORE any message/service imports because it installs the import hooks that enhance message and service classes. If you import messages/services before importing rospy, the hooks won't be active and the classes won't be enhanced.

### 3. Package Name Update
**Original:** `from rospy_tutorials.srv import *`
**Changed to:** `from rospy_tutorials.srv import *`
**Reason:** Use ROS2-compatible service package

### 4. Path Setup for Direct Execution
Added path setup code to allow scripts to find rospy when run directly:
```python
if 'AMENT_PREFIX_PATH' in os.environ:
    for path in os.environ['AMENT_PREFIX_PATH'].split(':'):
        site_packages = os.path.join(path, 'lib', 'python3.8', 'site-packages')
        if os.path.exists(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)
```

**Note:** Scripts work best when run with `python3 script_name` after sourcing the ROS2 environment.

## Running the Scripts

**Recommended approach:**
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Terminal 1: Server
python3 install/rospy_tutorials/lib/rospy_tutorials/add_two_ints_server

# Terminal 2: Client
python3 install/rospy_tutorials/lib/rospy_tutorials/add_two_ints_client 10 20
```

## No Other Changes Required

All ROS1 service patterns work unchanged thanks to rospy:
- ✅ `rospy.Service()` with service type and handler
- ✅ `rospy.ServiceProxy()` with service name and type
- ✅ `rospy.wait_for_service()` with timeout
- ✅ Service request/response with positional arguments (via import hooks)
- ✅ Handler signature: `handler(request) -> response`

## Import Hook Features

The rospy import hook system also handles `.srv` modules:
- Intercepts service type imports
- Enhances Request and Response classes to accept positional arguments
- Maintains full backward compatibility with keyword arguments
