cmake_minimum_required(VERSION 3.0.2)
project(complex_recovery)

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
  LIBRARIES complex_recovery
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

add_library(complex_recovery
    src/sequential_complex_recovery.cpp
    src/parallel_complex_recovery.cpp
    src/utils.cpp
)
add_dependencies(complex_recovery
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(complex_recovery
    ${catkin_LIBRARIES}
)

install(TARGETS complex_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES complex_recovery_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#
# Testing
#
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_executable_with_gtest(sequential_complex_recovery_test_node
        tests/sequential_complex_recovery_test_node.cpp
    )
    target_link_libraries(sequential_complex_recovery_test_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )
    add_rostest(tests/sequential_complex_recovery.test)

    catkin_add_executable_with_gtest(parallel_complex_recovery_test_node
        tests/parallel_complex_recovery_test_node.cpp
    )
    target_link_libraries(parallel_complex_recovery_test_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )
    add_rostest(tests/parallel_complex_recovery.test)
endif()
