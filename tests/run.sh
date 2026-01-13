#!/bin/bash
# Combined test runner for ROS 1 Noetic and ROS 2 Foxy/Rolling
# Usage: ./run.sh [--coverage] [noetic|foxy|rolling|all]

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( dirname "$SCRIPT_DIR" )"
COLLECT_COVERAGE=0

# Check for --coverage flag
if [[ "$1" == "--coverage" ]]; then
    COLLECT_COVERAGE=1
    shift
    # Create coverage output directories
    mkdir -p "$PROJECT_DIR/coverage/foxy"
    mkdir -p "$PROJECT_DIR/coverage/rolling"
fi

run_noetic() {
    echo "=========================================="
    echo "ROS 1 Noetic Test Suite"
    echo "=========================================="
    echo ""

    podman pull docker.io/ros:noetic

    podman run --rm \
        -v "$SCRIPT_DIR/scripts:/mnt/scripts:ro" \
        -v "$SCRIPT_DIR/ros1_pkg:/mnt/ros1_pkg:ro" \
        docker.io/ros:noetic \
        bash -c '
            set -e
            source /opt/ros/noetic/setup.bash

            echo "Setting up workspace..."
            mkdir -p /ws/src/rospy_too_tests

            cp /mnt/ros1_pkg/package.xml /ws/src/rospy_too_tests/
            cp /mnt/ros1_pkg/CMakeLists.txt /ws/src/rospy_too_tests/

            mkdir -p /ws/src/rospy_too_tests/scripts
            cp /mnt/scripts/*.py /ws/src/rospy_too_tests/scripts/
            chmod +x /ws/src/rospy_too_tests/scripts/*.py

            echo "Building workspace..."
            cd /ws
            catkin_make

            source devel/setup.bash

            echo "Starting roscore..."
            roscore &
            ROSCORE_PID=$!
            sleep 2

            echo "Installing pytest..."
            apt-get update -qq && apt-get install -qq -y python3-pip >/dev/null
            python3 -m pip install -q pytest

            echo "Running tests..."
            cd /ws/src/rospy_too_tests/scripts
            python3 run_all.py
            TEST_RESULT=$?

            kill $ROSCORE_PID 2>/dev/null || true
            exit $TEST_RESULT
        '

    echo ""
    echo "=========================================="
    echo "Noetic tests completed."
    echo "=========================================="
}

run_foxy() {
    echo "=========================================="
    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        echo "ROS 2 Foxy Test Suite (with coverage)"
    else
        echo "ROS 2 Foxy Test Suite"
    fi
    echo "=========================================="
    echo ""

    podman pull docker.io/ros:foxy

    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        podman run --rm \
            -v "$SCRIPT_DIR/scripts:/mnt/scripts:ro" \
            -v "$SCRIPT_DIR/ros2_pkg:/mnt/ros2_pkg:ro" \
            -v "$PROJECT_DIR/rospy_too:/mnt/rospy_too:ro" \
            -v "$PROJECT_DIR/coverage/foxy:/mnt/coverage:rw" \
            docker.io/ros:foxy \
            bash -c '
                set -e
                source /opt/ros/foxy/setup.bash

                echo "Setting up workspace..."
                mkdir -p /ws/src/rospy_too_tests
                mkdir -p /ws/src/rospy_too

                cp /mnt/ros2_pkg/package.xml /ws/src/rospy_too_tests/
                cp /mnt/ros2_pkg/CMakeLists.txt /ws/src/rospy_too_tests/

                cp -r /mnt/rospy_too/* /ws/src/rospy_too/

                mkdir -p /ws/src/rospy_too_tests/scripts
                cp /mnt/scripts/*.py /ws/src/rospy_too_tests/scripts/
                chmod +x /ws/src/rospy_too_tests/scripts/*.py

                echo "Building workspace..."
                cd /ws
                colcon build --symlink-install

                source install/setup.bash

                echo "Installing coverage and pytest..."
                apt-get update -qq && apt-get install -qq -y python3-pip >/dev/null
                python3 -m pip install -q coverage pytest

                echo "Running tests with coverage..."
                cd /ws/src/rospy_too_tests/scripts
                python3 -m coverage run --parallel-mode --source=/ws/src/rospy_too/rospy --branch run_all.py
                TEST_RESULT=$?

                echo ""
                echo "Combining coverage data from subprocesses..."
                python3 -m coverage combine

                echo ""
                echo "=========================================="
                echo "Coverage Report"
                echo "=========================================="
                python3 -m coverage report

                echo "Generating HTML report..."
                python3 -m coverage html -d /mnt/coverage/html
                python3 -m coverage xml -o /mnt/coverage/coverage.xml

                exit $TEST_RESULT
            '
    else
        podman run --rm \
            -v "$SCRIPT_DIR/scripts:/mnt/scripts:ro" \
            -v "$SCRIPT_DIR/ros2_pkg:/mnt/ros2_pkg:ro" \
            -v "$PROJECT_DIR/rospy_too:/mnt/rospy_too:ro" \
            docker.io/ros:foxy \
            bash -c '
                set -e
                source /opt/ros/foxy/setup.bash

                echo "Setting up workspace..."
                mkdir -p /ws/src/rospy_too_tests
                mkdir -p /ws/src/rospy_too

                cp /mnt/ros2_pkg/package.xml /ws/src/rospy_too_tests/
                cp /mnt/ros2_pkg/CMakeLists.txt /ws/src/rospy_too_tests/

                cp -r /mnt/rospy_too/* /ws/src/rospy_too/

                mkdir -p /ws/src/rospy_too_tests/scripts
                cp /mnt/scripts/*.py /ws/src/rospy_too_tests/scripts/
                chmod +x /ws/src/rospy_too_tests/scripts/*.py

                echo "Building workspace..."
                cd /ws
                colcon build --symlink-install

                source install/setup.bash

                echo "Installing pytest..."
                apt-get update -qq && apt-get install -qq -y python3-pip >/dev/null
                python3 -m pip install -q pytest

                echo "Running tests..."
                cd /ws/src/rospy_too_tests/scripts
                python3 run_all.py
                TEST_RESULT=$?

                exit $TEST_RESULT
            '
    fi

    echo ""
    echo "=========================================="
    echo "Foxy tests completed."
    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        echo "Coverage report: $PROJECT_DIR/coverage/foxy/html/index.html"
    fi
    echo "=========================================="
}

run_rolling() {
    echo "=========================================="
    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        echo "ROS 2 Rolling Test Suite (with coverage)"
    else
        echo "ROS 2 Rolling Test Suite"
    fi
    echo "=========================================="
    echo ""

    podman pull docker.io/ros:rolling

    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        podman run --rm \
            -v "$SCRIPT_DIR/scripts:/mnt/scripts:ro" \
            -v "$SCRIPT_DIR/ros2_pkg:/mnt/ros2_pkg:ro" \
            -v "$PROJECT_DIR/rospy_too:/mnt/rospy_too:ro" \
            -v "$PROJECT_DIR/coverage/rolling:/mnt/coverage:rw" \
            docker.io/ros:rolling \
            bash -c '
                set -e
                source /opt/ros/rolling/setup.bash

                echo "Setting up workspace..."
                mkdir -p /ws/src/rospy_too_tests
                mkdir -p /ws/src/rospy_too

                cp /mnt/ros2_pkg/package.xml /ws/src/rospy_too_tests/
                cp /mnt/ros2_pkg/CMakeLists.txt /ws/src/rospy_too_tests/

                cp -r /mnt/rospy_too/* /ws/src/rospy_too/

                mkdir -p /ws/src/rospy_too_tests/scripts
                cp /mnt/scripts/*.py /ws/src/rospy_too_tests/scripts/
                chmod +x /ws/src/rospy_too_tests/scripts/*.py

                echo "Building workspace..."
                cd /ws
                colcon build --symlink-install

                source install/setup.bash

                echo "Installing coverage and pytest..."
                apt-get update -qq && apt-get install -qq -y python3-pip >/dev/null
                python3 -m pip install -q --break-system-packages coverage pytest

                echo "Running tests with coverage..."
                cd /ws/src/rospy_too_tests/scripts
                python3 -m coverage run --parallel-mode --source=/ws/src/rospy_too/rospy --branch run_all.py
                TEST_RESULT=$?

                echo ""
                echo "Combining coverage data from subprocesses..."
                python3 -m coverage combine

                echo ""
                echo "=========================================="
                echo "Coverage Report"
                echo "=========================================="
                python3 -m coverage report

                echo "Generating HTML report..."
                python3 -m coverage html -d /mnt/coverage/html
                python3 -m coverage xml -o /mnt/coverage/coverage.xml

                exit $TEST_RESULT
            '
    else
        podman run --rm \
            -v "$SCRIPT_DIR/scripts:/mnt/scripts:ro" \
            -v "$SCRIPT_DIR/ros2_pkg:/mnt/ros2_pkg:ro" \
            -v "$PROJECT_DIR/rospy_too:/mnt/rospy_too:ro" \
            docker.io/ros:rolling \
            bash -c '
                set -e
                source /opt/ros/rolling/setup.bash

                echo "Setting up workspace..."
                mkdir -p /ws/src/rospy_too_tests
                mkdir -p /ws/src/rospy_too

                cp /mnt/ros2_pkg/package.xml /ws/src/rospy_too_tests/
                cp /mnt/ros2_pkg/CMakeLists.txt /ws/src/rospy_too_tests/

                cp -r /mnt/rospy_too/* /ws/src/rospy_too/

                mkdir -p /ws/src/rospy_too_tests/scripts
                cp /mnt/scripts/*.py /ws/src/rospy_too_tests/scripts/
                chmod +x /ws/src/rospy_too_tests/scripts/*.py

                echo "Building workspace..."
                cd /ws
                colcon build --symlink-install

                source install/setup.bash

                echo "Installing pytest..."
                apt-get update -qq && apt-get install -qq -y python3-pip >/dev/null
                python3 -m pip install -q --break-system-packages pytest

                echo "Running tests..."
                cd /ws/src/rospy_too_tests/scripts
                python3 run_all.py
                TEST_RESULT=$?

                exit $TEST_RESULT
            '
    fi

    echo ""
    echo "=========================================="
    echo "Rolling tests completed."
    if [[ "$COLLECT_COVERAGE" -eq 1 ]]; then
        echo "Coverage report: $PROJECT_DIR/coverage/rolling/html/index.html"
    fi
    echo "=========================================="
}

if [ "$#" -eq 0 ] || { [ "$#" -eq 1 ] && [ "$1" = "all" ]; }; then
    releases=(noetic foxy rolling)
else
    releases=("$@")
fi

for rel in "${releases[@]}"; do
    case "$(echo "$rel" | tr "[:upper:]" "[:lower:]")" in
        noetic) run_noetic ;;
        foxy) run_foxy ;;
        rolling) run_rolling ;;
        *)
            echo "Unknown release: $rel" >&2
            exit 1
            ;;
    esac
done
