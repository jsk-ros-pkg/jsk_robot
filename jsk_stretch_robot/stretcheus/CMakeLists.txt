cmake_minimum_required(VERSION 2.8.3)
project(stretcheus)

find_package(catkin REQUIRED COMPONENTS
    stretch_description
    roseus
)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/stretch.l
    COMMAND rosrun euscollada collada2eus stretch.dae stretch.yaml stretch.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${PROJECT_SOURCE_DIR}/stretch.dae ${PROJECT_SOURCE_DIR}/stretch.yaml)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/stretch.dae
    COMMAND rosrun collada_urdf urdf_to_collada ${PROJECT_SOURCE_DIR}/stretch.urdf stretch.dae
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${PROJECT_SOURCE_DIR}/stretch.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/stretch.urdf
  COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH}
          rosrun xacro xacro ${stretch_description_SOURCE_PREFIX}/urdf/stretch_description.xacro 
          use_nominal_extrinsics:=true > stretch.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  DEPENDS ${stretch_description_SOURCE_PREFIX}/urdf/stretch_description.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_dex_wrist.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_main.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_aruco.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_d435i.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_laser_range_finder.xacro
          ${stretch_description_SOURCE_PREFIX}/urdf/stretch_respeaker.xacro)

find_package(PkgConfig)
pkg_check_modules(COLLADA collada-dom>=2.4.4)
if(${COLLADA_FOUND})
  add_custom_target(generate_stretch_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/stretch.l)
else()
  pkg_check_modules(COLLADA collada-dom)
  message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stretcheus
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
