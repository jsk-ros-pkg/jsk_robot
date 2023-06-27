cmake_minimum_required(VERSION 2.8.3)
project(unitreeeus)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

find_package(go1_description) # Just in case when description is not released. Avoid compile failing
if(NOT go1_description_FOUND)
  message(WARNING "go1_description is not found, so skip generating eus models.")
  return()
endif()

find_package(euscollada) # Just in case when euscollada not released. Avoid compile failing
if(NOT euscollada_FOUND)
  message(WARNING "euscollada is not found, so skip generating eus models.")
  return()
endif()



###########
## Build ##
###########
if(EXISTS ${go1_description_SOURCE_PREFIX}/urdf)
  set(_unitree_urdf ${go1_description_SOURCE_PREFIX}/urdf)
else()
  set(_unitree_urdf ${go1_description_PREFIX}/share/go1_description/urdf)
endif()
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("unitree_urdf: ${_kinova_urdf}")
message("collada2eus: ${_collada2eus}")

foreach(ROBOT_TYPE go1)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    COMMAND echo "${_collada2eus} ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${ROBOT_TYPE}.l"
    COMMAND ${_collada2eus} ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    DEPENDS ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${_collada2eus})
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l
    COMMAND echo "${_collada2eus} --simple_geometry ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${ROBOT_TYPE}-simple.l"
    COMMAND ${_collada2eus} --simple_geometry ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l
    DEPENDS ${_unitree_urdf}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${_collada2eus})
  add_custom_target(compile_${ROBOT_TYPE} ALL DEPENDS ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l)
endforeach()


install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES go1.l unitree-interface.l go1-utils.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  # Test only roseus
  add_rostest(test/test-unitree.test)
endif()
