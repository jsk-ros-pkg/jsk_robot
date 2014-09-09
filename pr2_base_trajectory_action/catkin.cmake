# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_base_trajectory_action)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp actionlib geometry_msgs nav_msgs trajectory_msgs pr2_controllers_msgs actionlib_msgs pr2_mechanism_model angles)
find_package(Boost REQUIRED COMPONENTS thread)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp actionlib geometry_msgs nav_msgs trajectory_msgs pr2_controllers_msgs actionlib_msgs pr2_mechanism_model angles
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_executable(pr2_base_trajectory_action src/pr2_base_trajectory_action.cpp)
target_link_libraries(pr2_base_trajectory_action ${catkin_LIBRARIES} ${Boost_LIBRARIES})