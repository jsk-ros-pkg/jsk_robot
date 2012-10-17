cmake_minimum_required(VERSION 2.4.6)

set(SETUP_DIRECTORY $ENV{XDG_DESKTOP_DIR})
if ("${SETUP_DIRECTORY}" STREQUAL "")
  set(SETUP_DIRECTORY $ENV{HOME}/Desktop)
endif ("${SETUP_DIRECTORY}" STREQUAL "")

set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(PROJECT_SCRIPTS_DIR ${PROJECT_SOURCE_DIR}/scripts)
set(PR2_USER applications)
set(USER_NAME $ENV{USER})

# Desktop Icons
configure_file(${PROJECT_SOURCE_DIR}/PR2Dashboard.desktop.in
               ${SETUP_DIRECTORY}/PR2Dashboard.desktop)
configure_file(${PROJECT_SOURCE_DIR}/PR2JSKStartup.desktop.in
               ${SETUP_DIRECTORY}/PR2JSKStartup.desktop)
configure_file(${PROJECT_SOURCE_DIR}/PR2MarkerControl.desktop.in
               ${SETUP_DIRECTORY}/PR2MarkerControl.desktop)
configure_file(${PROJECT_SOURCE_DIR}/StartPR2.desktop.in
               ${SETUP_DIRECTORY}/StartPR2.desktop)
configure_file(${PROJECT_SOURCE_DIR}/StopPR2.desktop.in
               ${SETUP_DIRECTORY}/StopPR2.desktop)

# scripts
configure_file(${PROJECT_SOURCE_DIR}/pr2_jsk_startup.sh.in
               ${PROJECT_SCRIPTS_DIR}/pr2_jsk_startup.sh)
configure_file(${PROJECT_SOURCE_DIR}/pr2_marker_control_main.sh.in
               ${PROJECT_SCRIPTS_DIR}/pr2_marker_control_main.sh)
configure_file(${PROJECT_SOURCE_DIR}/detect_cans_demo.sh.in
               ${PROJECT_SCRIPTS_DIR}/detect_cans_demo.sh)

add_custom_target(desktop ALL DEPENDS
  ${PROJECT_SOURCE_DIR}/PR2Dashboard.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2JSKStartup.desktop.in
  ${PROJECT_SOURCE_DIR}/PR2MarkerControl.desktop.in
  ${PROJECT_SOURCE_DIR}/StartPR2.desktop.in
  ${PROJECT_SOURCE_DIR}/StopPR2.desktop.in
  ${PROJECT_SOURCE_DIR}/pr2_jsk_startup.sh.in
  ${PROJECT_SOURCE_DIR}/pr2_marker_control_main.sh.in
  ${PROJECT_SOURCE_DIR}/detect_cans_demo.sh.in
  )
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
  "${SETUP_DIRECTORY}/PR2Dashboard.desktop;${SETUP_DIRECTORY}/PR2JSKStartup.desktop;${SETUP_DIRECTORY}/PR2MarkerControl.desktop;${SETUP_DIRECTORY}/StartPR2.desktop;${SETUP_DIRECTORY}/StopPR2.desktop;${PROJECT_SCRIPTS_DIR}/pr2_jsk_startup.sh;${PROJECT_SCRIPTS_DIR}/pr2_marker_control_main.sh;${PROJECT_SCRIPTS_DIR}/detect_cans_demo.sh"
  )
