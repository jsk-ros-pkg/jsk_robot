cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  roscpp
  rospy
  pr2_mechanism_controllers
  diagnostic_msgs
  sensor_msgs
  roseus
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY lifelog util
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
