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
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

macro(configure_email_topic_yaml rname unitree_name)
  set(UNITREE_NAME ${unitree_name})
  set(RECEIVER_MAIL ${rname})
  set(EMAIL_SENDER_ADDRESS ${UNITREE_NAME}@jsk.imi.i.u-tokyo.ac.jp)
  set(EMAIL_RECEIVER_ADDRESS ${RECEIVER_MAIL}@jsk.imi.i.u-tokyo.ac.jp)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../jsk_robot_common/jsk_robot_startup/config/email_topic.yaml.in
    ${CMAKE_CURRENT_SOURCE_DIR}/config/${UNITREE_NAME}_email_topic.yaml)
endmacro(configure_email_topic_yaml)

configure_email_topic_yaml(unitree unitreepro)

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
