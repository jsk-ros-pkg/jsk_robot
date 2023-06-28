cmake_minimum_required(VERSION 3.0.2)
project(trigger_behavior_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
)

add_action_files(DIRECTORY action FILES TriggerBehavior.action)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package()
