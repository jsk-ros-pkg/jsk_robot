cmake_minimum_required(VERSION 2.8.3)
project(aeroeus)

find_package(catkin REQUIRED)

catkin_package(
)

## rosrun euscollada collada2eus_urdfmodel $(rospack find aero_description)/robots/aero.urdf $(rospack find aero_description)/robots/aero.yaml aero.l

file(GLOB LISP_FILES *.l)
install(FILES ${LISP_FILES} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
