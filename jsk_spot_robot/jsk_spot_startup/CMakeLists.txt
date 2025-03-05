cmake_minimum_required(VERSION 2.8.3)
project(jsk_spot_startup)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############
install(DIRECTORY apps auth config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
)

file(GLOB_RECURSE SCRIPT_PROGRAMS node_scripts/*)
foreach(SCRIPT_PROGRAM ${SCRIPT_PROGRAMS})
  if("${SCRIPT_PROGRAM}" MATCHES ".*\\.py$")
    catkin_install_python(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts)
  else()
    install(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts)
  endif()
endforeach()


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  set(ROSLINT_PYTHON_OPTS --max-line-length=180 --ignore=E221,E222,E241) # skip multiple spaces before/after operator
  roslint_python()
  roslint_add_test()
endif()
