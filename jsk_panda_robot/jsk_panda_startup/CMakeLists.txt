cmake_minimum_required(VERSION 2.8.0)
find_package(catkin)
project(jsk_panda_startup)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


catkin_add_env_hooks(99.jsk_panda_startup SHELLS bash zsh sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
