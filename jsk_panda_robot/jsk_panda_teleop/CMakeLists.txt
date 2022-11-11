cmake_minimum_required(VERSION 2.8.3)
project(jsk_panda_teleop)


find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  )

add_service_files(
  FILES
  ControlBilateral.srv
  CommandImitationAgent.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  )

catkin_package()

install(DIRECTORY config euslisp launch python script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
