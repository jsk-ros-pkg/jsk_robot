cmake_minimum_required(VERSION 2.8.3)
project(jsk_pepper_startup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch nodes sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING AND NOT $ENV{ROS_DISTRO} STREQUAL "hydro")
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  # https://github.com/ros/ros_comm/pull/730
  set(roslaunch_check_script ${PROJECT_SOURCE_DIR}/script/roslaunch-check)
  roslaunch_add_file_check(launch/jsk_pepper_startup.launch "ROS_IP=10.68.0.2 NAO_IP=10.68.0.3")
endif()
