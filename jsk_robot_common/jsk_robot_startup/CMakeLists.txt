cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
