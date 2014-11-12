cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_startup)

find_package(catkin REQUIRED COMPONENTS
  baxter_description
  openni_launch
  joy
  checkerboard_detector
  topic_tools
  sound_play
  baxter_core_msgs
  dynamic_tf_publisher
  rostwitter
  image_view
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_startup
#  CATKIN_DEPENDS baxter_interface baxter_tools openni_launch
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
