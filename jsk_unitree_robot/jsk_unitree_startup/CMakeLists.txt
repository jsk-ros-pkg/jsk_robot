cmake_minimum_required(VERSION 2.8.3)
project(jsk_unitree_startup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
file(GLOB_RECURSE SCRIPT_PROGRAMS scripts/*)
foreach(SCRIPT_PROGRAM ${SCRIPT_PROGRAMS})
  if("${SCRIPT_PROGRAM}" MATCHES ".*\\.py$")
    catkin_install_python(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    install(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
  endif()
endforeach()
install(DIRECTORY config launch apps autostart ino
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)

  file(GLOB LAUNCH_FILES launch/*.launch)
  # TODO: Do not do roslaunch_add_file_check()
  # foreach(LAUNCH_FILE ${LAUNCH_FILES})
  #   roslaunch_add_file_check(${LAUNCH_FILE})
  # endforeach()

  set(ROSLINT_PYTHON_OPTS --max-line-length=180 --ignore=E221,E222,E241) # skip multiple spaces before/after operator
  roslint_python()
  roslint_add_test()
endif()
