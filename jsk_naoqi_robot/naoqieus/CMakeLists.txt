cmake_minimum_required(VERSION 2.8.3)
project(naoqieus)

find_package(catkin REQUIRED rostest diagnostic_aggregator nao_interaction_msgs naoqi_driver roseus)

catkin_package(CFG_EXTRAS compile_naoqi_model.cmake)

if(NOT "$ENV{ROS_DISTRO}" STREQUAL "hydro") ## naoqi_bridge_msg is only for >= indigo
  add_rostest(test/naoqieus.test)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
endif()

install(FILES naoqi-interface.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


