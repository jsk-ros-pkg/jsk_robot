cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_util)
find_package(catkin REQUIRED)
catkin_package(
)
install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
