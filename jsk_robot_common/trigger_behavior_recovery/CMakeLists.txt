cmake_minimum_required(VERSION 2.8.3)
project(trigger_behavior_recovery)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
        genmsg
        actionlib_msgs
)

add_action_files(DIRECTORY action FILES TriggerBehavior.action)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trigger_behavior_recovery
  CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

# Abort if indigo or kinetic
if ( $ENV{ROS_DISTRO} STREQUAL "indigo" OR $ENV{ROS_DISTRO} STREQUAL "kinetic" )
    return()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(trigger_behavior_recovery src/trigger_behavior_recovery.cpp)
add_dependencies(trigger_behavior_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trigger_behavior_recovery ${catkin_LIBRARIES})

#
# Install
#
install(TARGETS trigger_behavior_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES trigger_behavior_recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#
# Testgin
#
#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#
#    catkin_add_executable_with_gtest(${PROJECT_NAME}_simple_node test/trigger_behavior_recovery_simple_node.cpp)
#    target_link_libraries(${PROJECT_NAME}_simple_node ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#    add_rostest(test/trigger_behavior_recovery.test)
#endif()
