cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_startup)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############
install(DIRECTORY config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
)

###################################
## Create desktop icons for spot ##
###################################
##
macro(configure_spot_ros_icon_files iname)
    set(ROBOT_NAME ${iname})
    configure_file(shortcuts/SpotROS.desktop.in
        $ENV{HOME}/.local/share/applications/SpotROS${ROBOT_NAME}.desktop)
    cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
    if(${_host_name} MATCHES "^${ROBOT_NAME}")
        configure_file(shortcuts/SpotROS.desktop.in
            $ENV{HOME}/Desktop/SpotROS${ROBOT_NAME}.desktop)
    endif()
endmacro(configure_spot_ros_icon_files)
##
macro(configure_rviz_icon_files iname)
    set(ROBOT_NAME ${iname})
    configure_file(shortcuts/RVizSpot.desktop.in
        $ENV{HOME}/.local/share/applications/RViz${ROBOT_NAME}.desktop)
    cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
    if(${_host_name} MATCHES "^${ROBOT_NAME}")
        configure_file(shortcuts/RVizSpot.desktop.in
            $ENV{HOME}/Desktop/RViz${ROBOT_NAME}.desktop)
    endif()
endmacro(configure_rviz_icon_files)

configure_spot_ros_icon_files(spot-jsk)
configure_rviz_icon_files(spot-jsk)


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
