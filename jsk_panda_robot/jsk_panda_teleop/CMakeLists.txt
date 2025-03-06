cmake_minimum_required(VERSION 2.8.3)
project(jsk_panda_teleop)


find_package(catkin REQUIRED
  message_generation
  )

add_service_files(
  FILES
  ControlBilateral.srv
  )

generate_messages()

catkin_package()

install(DIRECTORY config launch script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  install(DIRECTORY python
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
else()
  install(DIRECTORY python
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN "*"
    PATTERN "*/*.py" EXCLUDE
    )

  file(GLOB SCRIPT_PROGRAMS python/*.py)
  catkin_install_python(
    PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/python
    )
endif()
