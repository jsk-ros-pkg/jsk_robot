cmake_minimum_required(VERSION 2.8.3)
project(pr2_base_trajectory_action)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib geometry_msgs nav_msgs trajectory_msgs pr2_controllers_msgs actionlib_msgs pr2_mechanism_model angles)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS  roscpp actionlib geometry_msgs nav_msgs trajectory_msgs pr2_controllers_msgs actionlib_msgs pr2_mechanism_model angles
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_executable(pr2_base_trajectory_action src/pr2_base_trajectory_action.cpp)
target_link_libraries(pr2_base_trajectory_action ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS pr2_base_trajectory_action
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

