cmake_minimum_required(VERSION 2.8.3)
project(jsk_cobotta_startup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#######################
## Setup Cobotta ROS ##
#######################
# download cobotta-ros files
# set(DOWNLOAD_FILE denso_cobotta-melodic-1.2.1-20200729.tar.bz2)
# if(NOT EXISTS ${PROJECT_SOURCE_DIR}/${DOWNLOAD_FILE})
#   message(FATAL_ERROR "Plase download ${DOWNLOAD_FILE} from https://www.denso-wave.com/ja/robot/download/application/cobotta_driver_for_linux.html to ${PROJECT_SOURCE_DIR}")
# endif()

# # check kernel version
# execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
# message(-- " Kernel version: " ${UNAME_RESULT})
# if(${UNAME_RESULT} EQUAL "5.3.0-28-generic" OR ${UNAME_RESULT} EQUAL "5.4.0-42-generic")
# else()
#   message(FATAL_ERROR "Linux kernel version should be either 5.3.0-28-generic/ or 5.4.0-42-generic/ ")
# endif()

# check if kernel is 5.

#############
## Install ##
#############
install(DIRECTORY config launch scripts data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES jsk_cobotta.machine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  # set(ROSLINT_PYTHON_OPTS --max-line-length=180 --ignore=E221,E222,E241) # skip multiple spaces before/after operator
  # roslint_python()
  # roslint_add_test()
endif()
