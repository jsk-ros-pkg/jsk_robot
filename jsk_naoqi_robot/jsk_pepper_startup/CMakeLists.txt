cmake_minimum_required(VERSION 2.8.3)
project(jsk_pepper_startup)

find_package(catkin REQUIRED COMPONENTS rostest)

add_rostest(test/check_launch.test)

catkin_package()

install(DIRECTORY launch nodes sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
