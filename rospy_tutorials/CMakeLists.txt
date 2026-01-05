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

ament_package()
