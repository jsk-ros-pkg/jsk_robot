cmake_minimum_required(VERSION 2.8.3)
project(update_move_base_parameter_recovery)

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
  LIBRARIES update_move_base_parameter_recovery
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

add_library(update_move_base_parameter_recovery
    src/update_inflation_layer_parameter_recovery.cpp
    src/update_costmap_parameter_recovery.cpp
)
add_dependencies(update_move_base_parameter_recovery
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(update_move_base_parameter_recovery
    ${catkin_LIBRARIES}
)

#
# Install
#
install(TARGETS update_move_base_parameter_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES update_move_base_parameter_recovery_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#
# Testing
#
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_executable_with_gtest(update_inflation_layer_parameter_recovery_test_node
        tests/update_inflation_layer_parameter_recovery_test_node.cpp
    )
    target_link_libraries(update_inflation_layer_parameter_recovery_test_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )
    add_rostest(tests/update_inflation_layer_parameter_recovery.test)

    catkin_add_executable_with_gtest(update_costmap_parameter_recovery_test_node
        tests/update_costmap_parameter_recovery_test_node.cpp
    )
    target_link_libraries(update_costmap_parameter_recovery_test_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )
    add_rostest(tests/update_costmap_parameter_recovery.test)
endif()
