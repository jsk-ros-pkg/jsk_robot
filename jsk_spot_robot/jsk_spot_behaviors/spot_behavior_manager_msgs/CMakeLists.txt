cmake_minimum_required(VERSION 3.0.2)
project(spot_behavior_manager_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib actionlib_msgs)


add_action_files(
    DIRECTORY action
    FILES LeadPerson.action
)

add_service_files(
    DIRECTORY srv
    FILES ResetCurrentNode.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  LIBRARIES spot_behavior_manager_msgs
  CATKIN_DEPENDS message_runtime
)
