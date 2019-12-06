cmake_minimum_required(VERSION 2.8.3)
project(jsk_aero_startup)
if($ENV{ROS_DISTRO} STREQUAL "hydro")
  message(STATUS "aero is not supported on hydro")
  find_package(catkin)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch raw_maps scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES aero.machine sim.machine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  # https://github.com/ros/ros_comm/pull/998
  set(roslaunch_check_script ${PROJECT_SOURCE_DIR}/test/roslaunch-check)
  roslaunch_add_file_check(test/aero.test.xml)
  set(roslaunch_check_script ${roslaunch_DIR}/../scripts/roslaunch-check)
endif()
