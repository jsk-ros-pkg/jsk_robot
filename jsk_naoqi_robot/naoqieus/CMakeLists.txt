cmake_minimum_required(VERSION 2.8.3)
project(naoqieus)

find_package(catkin REQUIRED roseus rostest)

catkin_package(CFG_EXTRAS compile_naoqi_model.cmake)

## naoqi_bridge_msg is only for >= indigo
find_package(nao_interaction_msgs QUIET)
find_package(naoqi_bridge_msgs QUIET)
if(nao_interaction_msgs_FOUND AND naoqi_bridge_msgs_FOUND)

  add_rostest(test/naoqieus.test)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
endif()

install(FILES naoqi-interface.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


