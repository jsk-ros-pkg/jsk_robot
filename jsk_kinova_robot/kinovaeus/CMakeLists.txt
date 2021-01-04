cmake_minimum_required(VERSION 2.8.3)
project(kinovaeus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roseus
  euscollada
)
find_package(kortex_description) # Just in case when description is not released. Avoid compile failing

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
# Gen3 Lite robot
# xacro command: https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description
foreach(ROBOT_TYPE gen3_lite_gen3_lite_2f gen3_robotiq_2f_85 gen3_robotiq_2f_140)
  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/${ROBOT_TYPE}.urdf
    COMMAND xacro ${_kinova_urdf}/${ROBOT_TYPE}.xacro sim:=false > ${PROJECT_BINARY_DIR}/${ROBOT_TYPE}.urdf
    DEPENDS ${_kinova_urdf}/${ROBOT_TYPE}.xacro)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    COMMAND echo "${_collada2eus} ${PROJECT_BINARY_DIR}/${ROBOT_TYPE}.urdf ${ROBOT_TYPE}.l"
    COMMAND ${_collada2eus} ${PROJECT_BINARY_DIR}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    DEPENDS ${PROJECT_BINARY_DIR}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${_collada2eus})
  add_custom_target(compile_${ROBOT_TYPE} ALL DEPENDS ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l)
endforeach()


install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES kinova.l kinova-interface.l kinova-util.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  # Test only roseus
  add_rostest(test/test-kinova.test)
  # Test with Kinematic Simulator
  add_rostest(test/test-gen3.test)
  add_rostest(test/test-gen3-lite.test)
  # Test with Gazebo and MoveIt!
  add_rostest(test/test-gen3-moveit.test)
  add_rostest(test/test-gen3-lite-moveit.test)
endif()


