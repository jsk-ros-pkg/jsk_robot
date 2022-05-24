cmake_minimum_required(VERSION 2.8.3)
project(speak_and_wait_recovery)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES speak_and_wait_recovery
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

add_library(speak_and_wait_recovery src/speak_and_wait_recovery.cpp)
add_dependencies(speak_and_wait_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speak_and_wait_recovery ${catkin_LIBRARIES})

#
# Install
#
install(TARGETS speak_and_wait_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES speak_and_wait_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#
# Testing
#
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_executable_with_gtest(${PROJECT_NAME}_simple_node test/speak_and_wait_recovery_simple_node.cpp)
    target_link_libraries(${PROJECT_NAME}_simple_node ${catkin_LIBRARIES} ${PROJECT_NAME})

    add_rostest(test/speak_and_wait_recovery.test)
endif()
