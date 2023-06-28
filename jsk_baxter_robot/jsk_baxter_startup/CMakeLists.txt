cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_startup)

find_package(catkin)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_startup
#  CATKIN_DEPENDS baxter_interface baxter_tools openni_launch
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_env_hooks(99.jsk_baxter_startup SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY config jsk_baxter_joy jsk_baxter_lifelog jsk_baxter_moveit jsk_baxter_sensors jsk_baxter_tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES baxter.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Create email config for baxter
macro(configure_email_topic_yaml rname baxter_name)
  set(BAXTER_NAME ${baxter_name})
  set(RECEIVER_MAIL ${rname})
  set(EMAIL_SENDER_ADDRESS ${BAXTER_NAME}@jsk.imi.i.u-tokyo.ac.jp)
  set(EMAIL_RECEIVER_ADDRESS ${RECEIVER_MAIL}@jsk.imi.i.u-tokyo.ac.jp)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../jsk_robot_common/jsk_robot_startup/config/email_topic.yaml.in
    ${CMAKE_CURRENT_SOURCE_DIR}/config/${BAXTER_NAME}_email_topic.yaml)
endmacro(configure_email_topic_yaml)

configure_email_topic_yaml(baxter baxter)
