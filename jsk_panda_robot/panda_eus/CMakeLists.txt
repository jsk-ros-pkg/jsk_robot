cmake_minimum_required(VERSION 2.8.0)
project(panda_eus)

find_package(catkin REQUIRED
  euscollada
  roseus
  xacro
  )
find_package(franka_description)  # Just in case when description is not released. Avoid compile failing

catkin_package()

###
### dual_panda.l generation
###
if(franka_description_FOUND AND (NOT ("$ENV{ROS_DISTRO}" STRLESS "melodic")))
  # xacro.load_yaml cannot be recognized under melodic, while it is recommended on melodic and upper.
  # PR introducing xacro.load_yaml: https://github.com/ros/xacro/pull/283
  # Related issue: https://github.com/ros/xacro/issues/298
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dual_panda.l
    COMMAND rosrun euscollada collada2eus -I dual_panda.urdf -C dual_panda.yaml -O dual_panda.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
    DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf ${PROJECT_SOURCE_DIR}/models/dual_panda.yaml)

  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf
    COMMAND rosrun xacro xacro --inorder dual_panda.urdf.xacro > dual_panda.urdf
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
    DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.urdf.xacro)

  add_custom_target(generate_panda_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/dual_panda.l)
else()
  message(WARNING "Dependency is not met, so skip generating dual_panda.l")
endif()


#############
## Install ##
#############

install(DIRECTORY euslisp meshes models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

