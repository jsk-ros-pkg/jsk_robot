cmake_minimum_required(VERSION 2.8.3)
project(cobottaeus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roseus
  euscollada
)
find_package(denso_cobotta_descriptions) # Just in case when description is not released. Avoid compile failing

catkin_package()

if(NOT denso_cobotta_descriptions_FOUND)
  message(WARNING "denso_cobotta_descriptions is not found, so skip generating cobotta.l")
  message(WARNING "Install denso_cobotta_descriptions from https://github.com/DENSORobot/denso_cobotta_ros")
  return()
endif()


###########
## Build ##
###########
if(EXISTS ${denso_cobotta_descriptions_SOURCE_PREFIX}/cobotta_description)
  set(_cobotta_urdf ${denso_cobotta_descriptions_SOURCE_PREFIX}/cobotta_description)
else()
  set(_cobotta_urdf ${denso_cobotta_descriptions_PREFIX}/share/denso_cobotta_descriptions/cobotta_description)
endif()
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)


message("cobotta_urdf: ${_cobotta_urdf}")
message("collada2eus: ${_collada2eus}")

add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/cobotta.urdf
  COMMAND xacro ${_cobotta_urdf}/cobotta.urdf.xacro > ${PROJECT_BINARY_DIR}/cobotta.urdf
  DEPENDS ${_cobotta_urdf}/cobotta.urdf.xacro)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/cobotta.l
  COMMAND echo "${_collada2eus} ${PROJECT_BINARY_DIR}/cobotta.urdf cobotta.l"
  COMMAND ${_collada2eus} ${PROJECT_BINARY_DIR}/cobotta.urdf ${PROJECT_SOURCE_DIR}/cobotta.yaml ${PROJECT_SOURCE_DIR}/cobotta.l
  DEPENDS ${PROJECT_BINARY_DIR}/cobotta.urdf ${PROJECT_SOURCE_DIR}/cobotta.yaml ${_collada2eus})
  add_custom_target(compile_cobotta ALL DEPENDS ${PROJECT_SOURCE_DIR}/cobotta.l)


install(DIRECTORY euslisp test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES cobotta.l cobotta-interface.l cobotta-util.l cobotta.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test-cobotta.test)
endif()
