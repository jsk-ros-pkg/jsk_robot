cmake_minimum_required(VERSION 2.8.3)
project(sciurus17eus)

## Find catkin macros and libraries
if($ENV{ROS_DISTRO} STREQUAL "indigo") # indigo requires to add find_package(roseus) to compile roseus message
  set(ROSEUS_package roseus)
endif()

find_package(catkin REQUIRED COMPONENTS
  ${ROSEUS_package} # required in indigo
  collada_urdf
  euscollada
  sciurus17_description
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sciurus17eus
  CATKIN_DEPENDS collada_urdf
)

###########
## Build ##
###########

if(EXISTS ${sciurus17_description_SOURCE_PREFIX}/urdf)
  set(_sciurus17_urdf ${sciurus17_description_SOURCE_PREFIX}/urdf)
else()
  set(_sciurus17_urdf ${sciurus17_description_PREFIX}/share/sciurus17_description/urdf)
endif()

set(_urdf_to_collada ${collada_urdf_PREFIX}/lib/collada_urdf/urdf_to_collada)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("sciurus17_urdf: ${_sciurus17_urdf}")
message("urdf_to_collada: ${_urdf_to_collada}")
message("collada2eus: ${_collada2eus}")

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/sciurus17.urdf
  COMMAND xacro ${_sciurus17_urdf}/sciurus17.urdf.xacro > ${PROJECT_SOURCE_DIR}/sciurus17.urdf
  DEPENDS ${_sciurus17_urdf}/sciurus17.urdf.xacro)

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/sciurus17.l)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/sciurus17.dae
    COMMAND echo "${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/sciurus17.urdf sciurus17.dae"
    COMMAND ${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/sciurus17.urdf ${PROJECT_SOURCE_DIR}/sciurus17.dae
    DEPENDS ${PROJECT_SOURCE_DIR}/sciurus17.urdf)

  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/sciurus17.l
    COMMAND echo "${_collada2eus} sciurus17.dae sciurus17.l"
    COMMAND ${_collada2eus} ${PROJECT_SOURCE_DIR}/sciurus17.dae ${PROJECT_SOURCE_DIR}/sciurus17.yaml ${PROJECT_SOURCE_DIR}/sciurus17.l
    DEPENDS ${PROJECT_SOURCE_DIR}/sciurus17.dae ${PROJECT_SOURCE_DIR}/sciurus17.yaml ${_collada2eus})

  add_custom_target(compile_sciurus17 ALL DEPENDS ${PROJECT_SOURCE_DIR}/sciurus17.l)
endif()

install(DIRECTORY euslisp test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES sciurus17.l sciurus17-interface.l sciurus17-utils.l sciurus17.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test-sciurus17-robot.test)
  if ("$ENV{ROS_DISTRO}" STRGREATER "indigo")  # >= kinetic
    add_rostest(test/sciurus17-moveit.test)
  elseif ("$ENV{ROS_DISTRO}" STRGREATER "hydro")  # >= indigo
    add_rostest(test/sciurus17-moveit-nogui.test)
  else()
    message(WARNING "Skip sciurus17-moveit.test")
  endif()
endif()
