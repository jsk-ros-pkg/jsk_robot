cmake_minimum_required(VERSION 2.8.3)
project(baxtereus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  collada_urdf
  euscollada
  baxter_description
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxtereus
#  CATKIN_DEPENDS collada_urdf euslisp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(baxtereus
#   src/${PROJECT_NAME}/baxtereus.cpp
# )

## Declare a cpp executable
# add_executable(baxtereus_node src/baxtereus_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(baxtereus_node baxtereus_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(baxtereus_node
#   ${catkin_LIBRARIES}
# )
if(EXISTS ${baxter_description_SOURCE_DIR}/urdf/baxter.urdf)
set(_baxter_urdf ${baxter_description_SOURCE_DIR}/urdf/baxter.urdf)
else()
set(_baxter_urdf ${baxter_description_PREFIX}/share/baxter_description/urdf/baxter.urdf)
endif()
set(_urdf_to_collada ${collada_urdf_PREFIX}/lib/collada_urdf/urdf_to_collada)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

# as of 2014/1/20, *_simple.DAE files are commited in https://github.com/k-okada/baxter_common
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.urdf
  COMMAND echo \"sed s/.DAE/_simple.DAE/ ${_baxter_urdf} | sed s/base_link_simple.DAE/base_link_collision.DAE/ | sed s/PEDESTAL_simple.DAE/pedestal_link_collision.DAE/ > ${PROJECT_SOURCE_DIR}/baxter.urdf\"
  COMMAND sed s/.DAE/_simple.DAE/ ${_baxter_urdf} | sed s/base_link_simple.DAE/base_link_collision.DAE/ | sed s/PEDESTAL_simple.DAE/pedestal_link_collision.DAE/ > ${PROJECT_SOURCE_DIR}/baxter.urdf
  DEPENDS ${_baxter_urdf})
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.dae
  COMMAND echo "${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/baxter.urdf baxter.dae"
  COMMAND ${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/baxter.urdf ${PROJECT_SOURCE_DIR}/baxter.dae
  DEPENDS ${PROJECT_SOURCE_DIR}/baxter.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.l
  COMMAND echo "${_collada2eus} baxter.dae baxter.l"
  COMMAND ${_collada2eus} ${PROJECT_SOURCE_DIR}/baxter.dae ${PROJECT_SOURCE_DIR}/baxter.yaml ${PROJECT_SOURCE_DIR}/baxter.l
  DEPENDS ${PROJECT_SOURCE_DIR}/baxter.dae ${PROJECT_SOURCE_DIR}/baxter.yaml ${_collada2eus})
add_custom_target(compile_baxter ALL DEPENDS ${PROJECT_SOURCE_DIR}/baxter.l)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baxtereus baxtereus_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxtereus.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
