cmake_minimum_required(VERSION 2.8.3)
project(baxtereus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roseus
  collada_urdf
  euscollada
#  baxter_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxtereus
#  CATKIN_DEPENDS collada_urdf euslisp
#  DEPENDS system_lib
)

###########
## Build ##
###########

# ---------------------------------
# Generate Baxter's eus robot model
# ---------------------------------
# xacro -> urdf
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.urdf
  COMMAND rosrun xacro xacro --inorder baxter.urdf.xacro end_effector:=false > baxter.urdf
  DEPENDS baxter.urdf.xacro
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
# urdf -> collada
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.dae
  COMMAND rosrun collada_urdf urdf_to_collada baxter.urdf baxter.dae
  DEPENDS baxter.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
# colllda -> eus
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/baxter.l
  COMMAND rosrun euscollada collada2eus baxter.dae baxter.yaml baxter.l
  DEPENDS baxter.dae baxter.yaml ${_collada2eus}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_target(compile_baxter ALL DEPENDS ${PROJECT_SOURCE_DIR}/baxter.l)


install(DIRECTORY euslisp test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES baxter.l baxter-interface.l baxter-util.l baxter-moveit.l baxter.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test-baxter.test DEPENDENCIES compile_baxter)
endif()
