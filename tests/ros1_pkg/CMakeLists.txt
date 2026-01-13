cmake_minimum_required(VERSION 3.0.2)
project(rospy_too_tests)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
)

catkin_package()

# Install test scripts
catkin_install_python(PROGRAMS
  scripts/test_import.py
  scripts/test_time.py
  scripts/test_node.py
  scripts/test_params.py
  scripts/test_logging.py
  scripts/test_messages.py
  scripts/test_pubsub.py
  scripts/test_services.py
  scripts/run_all.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
