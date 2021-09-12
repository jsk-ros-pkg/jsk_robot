cmake_minimum_required(VERSION 2.8.3)
project(jsk_fetch_startup)
if($ENV{ROS_DISTRO} STREQUAL "hydro")
  message(STATUS "fetch is not supported on hydro")
  find_package(catkin)
  catkin_package()
  return()
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin)

###################################
## catkin specific configuration ##
###################################
catkin_package()

catkin_add_env_hooks(99.jsk_fetch_startup SHELLS bash zsh sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

#############
## Install ##
#############
install(DIRECTORY config launch scripts data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES jsk_fetch.machine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Create Rviz icon for fetch
macro(configure_icon_files icol iname)
  set(ICON_COLOR ${icol})
  set(FETCH_NAME ${iname})
  configure_file(config/RVizfetch.desktop.in
    $ENV{HOME}/.local/share/applications/RViz${FETCH_NAME}.desktop)
  # If HOSTNAME is "fetchxxx", create desktop icons
  cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
  if(${_host_name} MATCHES "^${FETCH_NAME}")
    configure_file(config/RVizfetch.desktop.in
      $ENV{HOME}/Desktop/RViz${FETCH_NAME}.desktop)
    configure_file(config/Supervisorfetch.desktop.in
      $ENV{HOME}/Desktop/Supervisor${FETCH_NAME}.desktop)
    # Multitouch gesture program for touchscrenn on Fetch's back
    configure_file(config/touchegg.conf
      $ENV{HOME}/.config/touchegg/touchegg.conf)
    configure_file(config/touchegg.desktop
      $ENV{HOME}/.config/autostart/touchegg.desktop)
  endif()
endmacro(configure_icon_files)

configure_icon_files(blue fetch15)
configure_icon_files(red fetch1075)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  # https://github.com/ros/ros_comm/pull/730 improve roslaunch-check to not fail if recursive dependencies lack dependencies (1.11.17)
  # https://github.com/ros/ros_comm/pull/998 respect if/unless for roslaunch-check (1.13.1)
  # https://github.com/ros/ros_comm/pull/1455 make roslaunch-check respect arg remappings with command line argument (1.14.4)
  set(roslaunch_check_script ${PROJECT_SOURCE_DIR}/scripts/roslaunch-check)
  roslaunch_add_file_check(launch/fetch_bringup.launch launch_teleop:=false) # disable launch_teleop because fetch_auto_dock package exists only in the fetch's PC
  roslaunch_add_file_check(launch/rviz.launch)
  roslaunch_add_file_check(launch/fetch_gazebo_bringup.launch)
  set(roslaunch_check_script ${roslaunch_DIR}/../scripts/roslaunch-check)
endif()
