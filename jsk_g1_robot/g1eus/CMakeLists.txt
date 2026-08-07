cmake_minimum_required(VERSION 3.8)
project(g1eus)

# g1eus is a ROS 2 only package. On ROS 1 (catkin) build it as a dummy
# package that installs nothing, so that `catkin build` keeps working.
# CATKIN_IGNORE is intentionally not used, because it would also hide this
# package from colcon (ROS 2).
if(NOT DEFINED ENV{ROS_VERSION} OR "$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  FILES g1.l g1-interface.l
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config figs models)
