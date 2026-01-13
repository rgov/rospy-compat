cmake_minimum_required(VERSION 3.5)
project(rospy_too_tests)

find_package(ament_cmake REQUIRED)

# Install test scripts
install(PROGRAMS
  scripts/test_import.py
  scripts/test_time.py
  scripts/test_node.py
  scripts/test_params.py
  scripts/test_logging.py
  scripts/test_messages.py
  scripts/test_pubsub.py
  scripts/test_services.py
  scripts/run_all.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
