cmake_minimum_required(VERSION 2.8.3)
project(jsk_aero_startup)

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
  roslaunch_add_file_check(test/aero.test.xml)
endif()
