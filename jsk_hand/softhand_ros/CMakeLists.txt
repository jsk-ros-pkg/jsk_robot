cmake_minimum_required(VERSION 2.8.3)
project(softhand_ros)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
)

catkin_python_setup()

add_action_files(
    DIRECTORY action
    FILES CalibJoint.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
)
