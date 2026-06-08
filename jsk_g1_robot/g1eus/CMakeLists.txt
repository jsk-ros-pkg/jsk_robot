cmake_minimum_required(VERSION 3.0.2)
project(g1eus)

find_package(catkin REQUIRED COMPONENTS
  euscollada
  pr2eus
  unitree_controller
)

catkin_package(
  CATKIN_DEPENDS euscollada pr2eus unitree_controller
)

set(G1_DESCRIPTION_DIR ${unitree_controller_SOURCE_PREFIX}/../robots/g1_description)
set(G1_SOURCE_URDF ${G1_DESCRIPTION_DIR}/g1_29dof.urdf)
set(G1_GENERATED_URDF ${PROJECT_SOURCE_DIR}/models/g1.urdf)
set(G1_EUSLISP ${PROJECT_SOURCE_DIR}/g1.l)

if(NOT EXISTS ${G1_SOURCE_URDF})
  message(FATAL_ERROR "Could not find G1 URDF at ${G1_SOURCE_URDF}")
endif()

file(READ ${G1_SOURCE_URDF} G1_URDF_CONTENT)
string(REPLACE
  "filename=\"meshes/"
  "filename=\"package://unitree_controller/../robots/g1_description/meshes/"
  G1_URDF_CONTENT
  "${G1_URDF_CONTENT}")
string(REPLACE
  "<robot name=\"g1_29dof\">"
  "<robot name=\"g1\">"
  G1_URDF_CONTENT
  "${G1_URDF_CONTENT}")
file(WRITE ${G1_GENERATED_URDF} "${G1_URDF_CONTENT}")

add_custom_command(
  OUTPUT ${G1_EUSLISP}
  COMMAND rosrun euscollada collada2eus -I ${G1_GENERATED_URDF} -N g1 -C ${PROJECT_SOURCE_DIR}/config/g1.yaml -O ${G1_EUSLISP}
  DEPENDS ${G1_GENERATED_URDF} ${PROJECT_SOURCE_DIR}/config/g1.yaml
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating G1 EusLisp model"
)

add_custom_target(generate_g1_eus_model ALL DEPENDS ${G1_EUSLISP})

install(DIRECTORY config euslisp models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES ${G1_GENERATED_URDF} ${G1_EUSLISP} g1-interface.l
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL)
