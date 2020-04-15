cmake_minimum_required(VERSION 2.8.3)
project(jsk_fetch_gazebo_demo)

find_package(catkin)
catkin_package()

install(DIRECTORY euslisp launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  if ("$ENV{ROS_DISTRO}" STRGREATER "lunar")  # >= melodic
    find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
    roslaunch_add_file_check(launch/demo.launch)
  endif()
endif()
