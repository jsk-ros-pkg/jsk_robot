cmake_minimum_required(VERSION 2.8.3)
project(jsk_panda_teleop)


find_package(catkin REQUIRED
  message_generation
  )

add_service_files(
  FILES
  ControlBilateral.srv
  )

generate_messages()

catkin_package()

install(DIRECTORY config euslisp launch python script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
