#!/bin/bash
# Quick sanity check script for rospy_tutorials tests in ROS Foxy container
#
# This script:
# 1. Launches a ROS Foxy Docker container
# 2. Mounts package sources read-only
# 3. Creates a clean workspace inside the container
# 4. Copies sources and builds the package
# 5. Runs the test suite
# 6. Displays test results

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================="
echo "ROS Foxy Test Script for rospy_tutorials"
echo "=========================================="
echo ""
echo "Package source: $SCRIPT_DIR"
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed or not in PATH"
    exit 1
fi

# Pull ROS Foxy image if not already present
echo "Pulling ROS Foxy Docker image (if needed)..."
docker pull ros:foxy

echo ""
echo "Starting ROS Foxy container and running tests..."
echo ""

# Run the container with sources mounted read-only
# Note: Using --rm to automatically remove container after exit
docker run --rm \
    -v "$SCRIPT_DIR:/mnt/packages:ro" \
    ros:foxy \
    bash -c "
        set -e

        echo '=========================================='
        echo 'Step 1: Create clean workspace'
        echo '=========================================='
        mkdir -p /workspace/src

        echo 'Copying source packages...'
        cp -r /mnt/packages/rospy_compat /workspace/src/
        cp -r /mnt/packages/rospy_tutorials /workspace/src/

        echo 'Workspace contents:'
        ls -la /workspace/src/

        echo ''
        echo '=========================================='
        echo 'Step 2: Source ROS Foxy environment'
        echo '=========================================='
        source /opt/ros/foxy/setup.bash

        echo ''
        echo '=========================================='
        echo 'Step 3: Build packages'
        echo '=========================================='
        cd /workspace
        colcon build --packages-up-to rospy_tutorials

        echo ''
        echo '=========================================='
        echo 'Step 4: Source workspace'
        echo '=========================================='
        source install/setup.bash

        echo ''
        echo '=========================================='
        echo 'Step 5: Run test suite'
        echo '=========================================='
        colcon test --packages-select rospy_tutorials

        echo ''
        echo '=========================================='
        echo 'Step 6: Display test results'
        echo '=========================================='
        colcon test-result --all

        echo ''
        echo '=========================================='
        echo 'Step 7: Detailed test results'
        echo '=========================================='
        colcon test-result --verbose

        echo ''
        echo '=========================================='
        echo 'Test Summary'
        echo '=========================================='

        # Count passing/failing tests
        TEST_RESULT_FILE=\$(find build/rospy_tutorials/test_results -name '*.xml' | head -1)
        if [ -f \"\$TEST_RESULT_FILE\" ]; then
            echo 'Test result files found:'
            find build/rospy_tutorials/test_results -name '*.xml'
        else
            echo 'No test result files found'
        fi
    "

EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ All tests completed successfully!"
else
    echo "❌ Tests failed with exit code: $EXIT_CODE"
fi
echo "=========================================="

exit $EXIT_CODE
