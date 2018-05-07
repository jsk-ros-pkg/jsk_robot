cmake_minimum_required(VERSION 2.8.3)
project(jsk_aero_startup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch raw_maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES aero.machine sim.machine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
