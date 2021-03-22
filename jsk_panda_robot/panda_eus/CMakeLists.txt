cmake_minimum_required(VERSION 2.8.0)
project(panda_eus)

find_package(catkin REQUIRED
  franka_description
  xacro
  roseus
  )

catkin_package(CATKIN_DEPENDS pr2eus)

###
### dual_panda.l generation
###
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dual_panda.l
  COMMAND rosrun euscollada collada2eus -I dual_panda.urdf -C dual_panda.yaml -O dual_panda.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf ${PROJECT_SOURCE_DIR}/models/dual_panda.yaml)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf
  COMMAND rosrun xacro xacro --inorder dual_panda.urdf.xacro > dual_panda.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf.xacro)

add_custom_target(generate_panda_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.l)

#############
## Install ##
#############

install(DIRECTORY models euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

