cmake_minimum_required(VERSION 2.8.3)
project(jsk_spot_plug_server)

find_package(catkin REQUIRED
    genmsg
    actionlib_msgs
)

add_action_files(
  FILES
  Plug.action
  Unplug.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/plug_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
