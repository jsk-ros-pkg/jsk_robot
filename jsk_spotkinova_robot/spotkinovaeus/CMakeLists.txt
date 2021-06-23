cmake_minimum_required(VERSION 2.8.3)
project(spotkinovaeus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roseus
  euscollada
  )
# Just in case when description is not released. Avoid compile failing
find_package(kortex_description)
find_package(spot_description)
find_package(kinovaeus)
find_package(spoteus)

catkin_package()

if(NOT kortex_description_FOUND OR NOT spot_description_FOUND)
  message(WARNING "kortex_description is not found, so skip generating eus models.")
  message(WARNING "Install kortex_description from https://github.com/Kinovarobotics/ros_kortex.git")
  return()
endif()


###########
## Build ##
###########
set(_spoteus ${spoteus_SOURCE_PREFIX})
set(_kinova_urdf ${kortex_description_SOURCE_PREFIX}/robots)
set(_kinovaeus ${kinovaeus_SOURCE_PREFIX})
set(_spot_kinova_urdf ${PROJECT_SOURCE_DIR}/robots)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("_spoteus: ${_spoteus}")
message("_kinova_urdf: ${_kinova_urdf}")
message("_kinovaeus: ${_kinovaeus}")
message("_spot_kinova_urdf: ${_spot_kinova_urdf}")
message("collada2eus: ${_collada2eus}")

# Gen3 robot with robotiq gripper
# Gen3 Lite robot
# xacro command: https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description
foreach(ROBOT_TYPE gen3_lite_gen3_lite_2f gen3_robotiq_2f_85 gen3_robotiq_2f_140)
  # Create spot_${ROBOT_TYPE}.urdf
  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/spot_${ROBOT_TYPE}.urdf
    COMMAND xacro ${_spot_kinova_urdf}/spot_kinova.xacro robot_type:=${ROBOT_TYPE} prefix:=kinova_ > ${PROJECT_BINARY_DIR}/spot_${ROBOT_TYPE}.urdf
    DEPENDS ${_spot_kinova_urdf}/spot_kinova.xacro)
  # Create spot_${ROBOT_TYPE}.l
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/spot_${ROBOT_TYPE}.l
    COMMAND echo "${_collada2eus} ${PROJECT_BINARY_DIR}/spot_${ROBOT_TYPE}.urdf spot_${ROBOT_TYPE}.l"
    COMMAND ${_collada2eus} ${PROJECT_BINARY_DIR}/spot_${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/spot_${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/spot_${ROBOT_TYPE}.l
    DEPENDS ${PROJECT_BINARY_DIR}/spot_${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/spot_${ROBOT_TYPE}.yaml ${_collada2eus})
  add_custom_target(compile_${ROBOT_TYPE} ALL DEPENDS ${PROJECT_SOURCE_DIR}/spot_${ROBOT_TYPE}.l)
endforeach()


install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES spot-kinova.l spot-kinova.yaml spot-kinova-interface.l spot-kinova-util.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
endif()
