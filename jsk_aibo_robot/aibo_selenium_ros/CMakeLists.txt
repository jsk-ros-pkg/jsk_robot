cmake_minimum_required(VERSION 2.8.3)
project(aibo_selenium_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)

catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
