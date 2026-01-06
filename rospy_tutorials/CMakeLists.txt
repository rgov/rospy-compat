cmake_minimum_required(VERSION 3.5)
project(rospy_tutorials)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Floats.msg"
  "msg/HeaderString.msg"
  "srv/AddTwoInts.srv"
  "srv/BadTwoInts.srv"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  001_talker_listener/talker.py
  001_talker_listener/listener.py
  001_talker_listener/talker_timer.py
  002_headers/talker_header.py
  002_headers/listener_header.py
  005_add_two_ints/add_two_ints_server
  005_add_two_ints/add_two_ints_client
  006_parameters/param_talker.py
  008_on_shutdown/publish_on_shutdown.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_001_talker_listener.py TIMEOUT 30)
  add_launch_test(test/test_001b_talker_timer.py TIMEOUT 30)
  add_launch_test(test/test_002_header_messages.py TIMEOUT 30)
  add_launch_test(test/test_005_add_two_ints.py TIMEOUT 30)
  add_launch_test(test/test_006_parameters.py TIMEOUT 30)
  add_launch_test(test/test_008_on_shutdown.py TIMEOUT 30)
endif()

ament_package()
