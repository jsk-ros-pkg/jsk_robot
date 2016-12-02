cmake_minimum_required(VERSION 2.8.3)
project(pr2_base_trajectory_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  control_msgs
  geometry_msgs
  nav_msgs
  roscpp
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS actionlib control_msgs geometry_msgs nav_msgs
    INCLUDE_DIRS include
    LIBRARIES
)

add_executable(pr2_base_trajectory_action
  src/pr2_base_trajectory_action_controller.cpp
  src/pr2_base_trajectory_action_controller_node.cpp)
target_link_libraries(pr2_base_trajectory_action ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS pr2_base_trajectory_action
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  catkin_add_gtest(spline_test
    test/spline_test.cpp)
  target_link_libraries(spline_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#  add_rostest(test/pr2_base_trajectory_action.test)
endif()
