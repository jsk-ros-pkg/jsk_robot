cmake_minimum_required(VERSION 2.8.3)
project(jsk_magni_startup)

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
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES jsk_magni.machine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  roslaunch_add_file_check(launch/magni_bringup.launch launch_teleop:=false) # disable launch_teleop because magni_auto_dock package
endif()
