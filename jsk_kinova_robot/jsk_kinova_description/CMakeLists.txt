cmake_minimum_required(VERSION 2.8.3)
project(jsk_kinova_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
