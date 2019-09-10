cmake_minimum_required(VERSION 2.8.3)
project(jsk_fetch_diagnosis)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  BoardInfo.msg
)

add_service_files(
  DIRECTORY srv
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
