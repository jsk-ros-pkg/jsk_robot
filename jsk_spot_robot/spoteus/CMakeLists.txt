cmake_minimum_required(VERSION 2.8.3)
project(spoteus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  collada_urdf
  roseus
  euscollada
  spot_description
)

catkin_package()

###########
## Build ##
###########
if(EXISTS ${spot_description_SOURCE_PREFIX}/urdf)
  set(_spot_urdf ${spot_description_SOURCE_PREFIX}/urdf)
else()
  set(_spot_urdf ${spot_description_PREFIX}/share/spot_description/urdf)
endif()
set(_urdf_to_collada ${collada_urdf_PREFIX}/lib/collada_urdf/urdf_to_collada)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("spot_urdf: ${_spot_urdf}")
message("urdf_to_collada: ${_urdf_to_collada}")
message("collada2eus: ${_collada2eus}")

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/spot.urdf
  COMMAND xacro ${_spot_urdf}/spot.urdf.xacro > ${PROJECT_SOURCE_DIR}/spot.urdf
  DEPENDS ${_spot_urdf}/spot.urdf.xacro)

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/spot.l)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/spot.dae
    COMMAND echo "${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/spot.urdf spot.dae"
    COMMAND ${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/spot.urdf ${PROJECT_SOURCE_DIR}/spot.dae
    DEPENDS ${PROJECT_SOURCE_DIR}/spot.urdf)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/spot.l
    COMMAND echo "${_collada2eus} spot.dae spot.l"
    COMMAND ${_collada2eus} ${PROJECT_SOURCE_DIR}/spot.dae ${PROJECT_SOURCE_DIR}/spot.yaml ${PROJECT_SOURCE_DIR}/spot.l
    DEPENDS ${PROJECT_SOURCE_DIR}/spot.dae ${PROJECT_SOURCE_DIR}/spot.yaml ${_collada2eus})
  add_custom_target(compile_spot ALL DEPENDS ${PROJECT_SOURCE_DIR}/spot.l)
endif()


install(DIRECTORY euslisp test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES spot.l spot-interface.l spot-util.l spot.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test-spot.test)
endif()
