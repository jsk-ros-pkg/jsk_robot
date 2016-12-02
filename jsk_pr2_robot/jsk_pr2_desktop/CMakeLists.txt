cmake_minimum_required(VERSION 2.4.6)

set(SETUP_DIRECTORY $ENV{XDG_DESKTOP_DIR})
if ("${SETUP_DIRECTORY}" STREQUAL "")
  set(SETUP_DIRECTORY $ENV{HOME}/Desktop)
endif ("${SETUP_DIRECTORY}" STREQUAL "")

set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(PROJECT_SCRIPTS_DIR ${PROJECT_SOURCE_DIR}/scripts)
set(PR2_USER applications)
set(USER_NAME $ENV{USER})
set(ROS_DISTRO $ENV{ROS_DISTRO})
set(MAKE_CLEAN_FILES "")
# scripts
configure_file(${PROJECT_SOURCE_DIR}/pr2_jsk_startup.sh.in
  ${PROJECT_SCRIPTS_DIR}/pr2_jsk_startup.sh)
configure_file(${PROJECT_SOURCE_DIR}/rviz.sh.in
  ${PROJECT_SCRIPTS_DIR}/rviz.sh)
configure_file(${PROJECT_SOURCE_DIR}/pr2_marker_control_main.sh.in
  ${PROJECT_SCRIPTS_DIR}/pr2_marker_control_main${PR2_NAME}.sh)
configure_file(${PROJECT_SOURCE_DIR}/detect_cans_demo.sh.in
  ${PROJECT_SCRIPTS_DIR}/detect_cans_demo${PR2_NAME}.sh)
configure_file(${PROJECT_SOURCE_DIR}/pr2_teleop_robot.sh.in
  ${PROJECT_SCRIPTS_DIR}/pr2_teleop_robot${PR2_NAME}.sh)
configure_file(${PROJECT_SOURCE_DIR}/pr2_teleop_remote.sh.in
  ${PROJECT_SCRIPTS_DIR}/pr2_teleop_remote${PR2_NAME}.sh)

macro(configure_and_register src dst)
  configure_file(${src} ${dst})
  set(MAKE_CLEAN_FILES ${MAKE_CLEAN_FILES} ${dst})
endmacro(configure_and_register)

macro(configure_icon_files icol iname)
  set(ICON_COLOR ${icol})
  set(PR2_NAME ${iname})
  # Desktop Icons
  configure_and_register(${PROJECT_SOURCE_DIR}/PR2Dashboard.desktop.in
    ${SETUP_DIRECTORY}/PR2Dashboard${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/PR2JSKStartup.desktop.in
    ${SETUP_DIRECTORY}/PR2JSKStartup${PR2_NAME}.desktop)
#  configure_file(${PROJECT_SOURCE_DIR}/PR2MarkerControl.desktop.in
#    ${SETUP_DIRECTORY}/PR2MarkerControl${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/StartPR2.desktop.in
    ${SETUP_DIRECTORY}/StartPR2${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/StopPR2.desktop.in
    ${SETUP_DIRECTORY}/StopPR2${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/RViz.desktop.in
    ${SETUP_DIRECTORY}/RViz${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/PR2TeleopRobot.desktop.in
    ${SETUP_DIRECTORY}/PR2TeleopRobot${PR2_NAME}.desktop)
  configure_and_register(${PROJECT_SOURCE_DIR}/PR2TeleopRemote.desktop.in
    ${SETUP_DIRECTORY}/PR2TeleopRemote${PR2_NAME}.desktop)

endmacro(configure_icon_files)

configure_icon_files(blue pr1012)
configure_icon_files(red pr1040)

add_custom_target(desktop ALL DEPENDS
  ${PROJECT_SOURCE_DIR}/PR2Dashboard.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2JSKStartup.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2MarkerControl.desktop.in
  ${PROJECT_SOURCE_DIR}/StartPR2.desktop.in
  ${PROJECT_SOURCE_DIR}/StopPR2.desktop.in
  ${PROJECT_SOURCE_DIR}/RViz.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2TeleopRobot.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2TeleopRemote.desktop.in
#  ${PROJECT_SOURCE_DIR}/pr2_jsk_startup.sh.in
  ${PROJECT_SOURCE_DIR}/pr2_marker_control_main.sh.in
  ${PROJECT_SOURCE_DIR}/detect_cans_demo.sh.in
  ${PROJECT_SOURCE_DIR}/pr2_teleop_robot.sh.in
  ${PROJECT_SOURCE_DIR}/pr2_teleop_remote.sh.in
  )
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
#  "${SETUP_DIRECTORY}/PR2Dashboard.desktop;${SETUP_DIRECTORY}/PR2JSKStartup.desktop;${SETUP_DIRECTORY}/PR2MarkerControl.desktop;${SETUP_DIRECTORY}/StartPR2.desktop;${SETUP_DIRECTORY}/StopPR2.desktop;${PROJECT_SCRIPTS_DIR}/rviz.sh;${PROJECT_SCRIPTS_DIR}/pr2_jsk_startup.sh;${PROJECT_SCRIPTS_DIR}/pr2_marker_control_main.sh;${PROJECT_SCRIPTS_DIR}/detect_cans_demo.sh"
#  )
set(MAKE_CLEAN_FILES ${MAKE_CLEAN_FILES} "${PROJECT_SCRIPTS_DIR}/rviz.sh;${PROJECT_SCRIPTS_DIR}/pr2_jsk_startup.sh;${PROJECT_SCRIPTS_DIR}/pr2_marker_control_main.sh;${PROJECT_SCRIPTS_DIR}/detect_cans_demo.sh")
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${MAKE_CLEAN_FILES}")
