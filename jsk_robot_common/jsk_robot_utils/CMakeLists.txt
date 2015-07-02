cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_utils)

find_package(catkin REQUIRED COMPONENTS
  roseus pr2eus rostest)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roseus pr2eus rostest
)

add_rostest(test/angle_vector_compress.test)
add_rostest(test/joint_state_compressor.test)

install(DIRECTORY euslisp
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
