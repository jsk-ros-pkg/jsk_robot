cmake_minimum_required(VERSION 2.8.3)
project(roseus_remote)

find_package(catkin REQUIRED COMPONENTS
  jsk_network_tools
  std_msgs
  roseus
)

add_message_files(
  FILES
  RawCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jskeus_remote
#  CATKIN_DEPENDS jsk_network_tools jskeus
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
