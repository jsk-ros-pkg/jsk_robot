cmake_minimum_required(VERSION 2.8.3)
project(kinovaeus)

find_package(catkin REQUIRED COMPONENTS
  roseus
  euscollada
  kortex_description
)

catkin_package()

if(NOT kortex_description_FOUND)
  message(WARNING "kortex_description is not found, so skip generating eus models.")
  message(WARNING "Install kortex_description from https://github.com/Kinovarobotics/ros_kortex.git")
  return()
endif()


###########
## Build ##
###########
if(EXISTS ${kortex_description_SOURCE_PREFIX}/robots)
  set(_kinova_urdf ${kortex_description_SOURCE_PREFIX}/robots)
else()
  set(_kinova_urdf ${kortex_description_PREFIX}/share/kortex_description/robots)
endif()
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("kinova_urdf: ${_kinova_urdf}")
message("collada2eus: ${_collada2eus}")

# Gen3 robot with robotiq gripper
# xacro command: https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description
add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/gen3.urdf
  COMMAND xacro ${_kinova_urdf}/gen3_robotiq_2f_85.xacro sim:=false > ${PROJECT_BINARY_DIR}/gen3.urdf
  DEPENDS ${_kinova_urdf}/gen3_robotiq_2f_85.xacro)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/gen3.l
  COMMAND echo "${_collada2eus} ${PROJECT_BINARY_DIR}/gen3.urdf gen3.l"
  COMMAND ${_collada2eus} ${PROJECT_BINARY_DIR}/gen3.urdf ${PROJECT_SOURCE_DIR}/gen3.yaml ${PROJECT_SOURCE_DIR}/gen3.l
  DEPENDS ${PROJECT_BINARY_DIR}/gen3.urdf ${PROJECT_SOURCE_DIR}/gen3.yaml ${_collada2eus})
add_custom_target(compile_gen3 ALL DEPENDS ${PROJECT_SOURCE_DIR}/gen3.l)

# Gen3 Lite robot
# xacro command: https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description
add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/gen3-lite.urdf
  COMMAND xacro ${_kinova_urdf}/gen3_lite_gen3_lite_2f.xacro sim:=false > ${PROJECT_BINARY_DIR}/gen3-lite.urdf
  DEPENDS ${_kinova_urdf}/gen3_lite_gen3_lite_2f.xacro)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/gen3-lite.l
  COMMAND echo "${_collada2eus} ${PROJECT_BINARY_DIR}/gen3-lite.urdf gen3-lite.l"
  COMMAND ${_collada2eus} ${PROJECT_BINARY_DIR}/gen3-lite.urdf ${PROJECT_SOURCE_DIR}/gen3-lite.yaml ${PROJECT_SOURCE_DIR}/gen3-lite.l
  DEPENDS ${PROJECT_BINARY_DIR}/gen3-lite.urdf ${PROJECT_SOURCE_DIR}/gen3-lite.yaml ${_collada2eus})
add_custom_target(compile_gen3_lite ALL DEPENDS ${PROJECT_SOURCE_DIR}/gen3-lite.l)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES gen3.l gen3-lite.l gen3-interface.l gen3-lite-interface.l gen3-utils.l gen3-lite-utils.l gen3.yaml gen3-lite.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  # Test with Kinematic Simulator
  add_rostest(test/test-gen3.test)
  add_rostest(test/test-gen3-lite.test)
  # TODO: Test with Gazebo
  # add_rostest(test/test-gen3-gazebo.test)
  # add_rostest(test/test-gen3-lite-gazebo.test)
endif()
