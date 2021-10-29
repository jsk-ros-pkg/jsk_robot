cmake_minimum_required(VERSION 3.0.2)
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
