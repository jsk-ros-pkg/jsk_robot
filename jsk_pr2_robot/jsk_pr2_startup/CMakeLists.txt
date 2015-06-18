cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_startup)
find_package(catkin REQUIRED COMPONENTS
  jsk_network_tools
  move_base_msgs
  roseus)

catkin_package(
  CATKIN_DEPENDS jsk_network_tools jsk_robot_startup pr2_mannequin_mode pr2_gripper_sensor_action
)

catkin_add_env_hooks(99.jsk_pr2_startup SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY config jsk_pr2_image_transport
  jsk_pr2_joy jsk_pr2_lifelog jsk_pr2_move_base jsk_pr2_moveit
  jsk_pr2_sensors jsk_pr2_warning src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES install_pr1040_description.sh jsk_pr2.machine plugin.xml
  pr2.launch pr2_bringup.launch pr2_jsk_interactive.launch 	rviz.launch startup.app
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
