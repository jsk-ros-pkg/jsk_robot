cmake_minimum_required(VERSION 2.8.3)
project(naoqieus)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo") # indigo requres to add find_pacakge(roseus) to compile roseus message
  set(ROSEUS_package roseus)
endif()
find_package(catkin REQUIRED
  ${ROSEUS_package} # required in indigo
  rostest)

catkin_package(CFG_EXTRAS compile_naoqi_model.cmake)

## naoqi_bridge_msg is only for >= indigo
find_package(nao_interaction_msgs QUIET)
find_package(naoqi_bridge_msgs QUIET)
if(nao_interaction_msgs_FOUND AND naoqi_bridge_msgs_FOUND)

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/naoqieus.test)
endif()

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
endif()

install(FILES naoqi-interface.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


