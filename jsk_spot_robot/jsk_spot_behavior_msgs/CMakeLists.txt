cmake_minimum_required(VERSION 2.8.3)
project(jsk_spot_behavior_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib actionlib_msgs)

add_action_files(
    DIRECTORY action
    FILES Navigation.action
)

add_service_files(
    DIRECTORY srv
    FILES ResetCurrentNode.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  LIBRARIES jsk_spot_behavior_msgs
  CATKIN_DEPENDS message_runtime
)
