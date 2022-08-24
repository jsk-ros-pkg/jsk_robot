cmake_minimum_required(VERSION 2.8.0)
find_package(catkin)
project(jsk_panda_startup)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

