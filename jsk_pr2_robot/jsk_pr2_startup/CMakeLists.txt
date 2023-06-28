cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_startup)
find_package(catkin REQUIRED)

catkin_package()

catkin_add_env_hooks(99.jsk_pr2_startup SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY config jsk_pr2_image_transport
  jsk_pr2_joy jsk_pr2_lifelog jsk_pr2_move_base jsk_pr2_moveit
  jsk_pr2_sensors jsk_pr2_warning src sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB _LAUNCH_FILES ${PROJECT_SOURCE_DIR}/ *.launch) # add / to avoid to get current directory
foreach(_LAUNCH_FILE ${_LAUNCH_FILES})
    install(FILES ${_LAUNCH_FILE}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

install(FILES install_pr1040_description.sh jsk_pr2.machine plugin.xml startup.app
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Create email config for PR2
macro(configure_email_topic_yaml rname pr2_name)
  set(PR2_NAME ${pr2_name})
  set(RECEIVER_MAIL ${rname})
  set(EMAIL_SENDER_ADDRESS ${PR2_NAME}@jsk.imi.i.u-tokyo.ac.jp)
  set(EMAIL_RECEIVER_ADDRESS ${RECEIVER_MAIL}@jsk.imi.i.u-tokyo.ac.jp)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../jsk_robot_common/jsk_robot_startup/config/email_topic.yaml.in
    ${CMAKE_CURRENT_SOURCE_DIR}/config/${PR2_NAME}_email_topic.yaml)
endmacro(configure_email_topic_yaml)

configure_email_topic_yaml(pr2 pr1012)
configure_email_topic_yaml(pr2 pr1040)
