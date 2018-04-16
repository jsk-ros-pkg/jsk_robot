cmake_minimum_required(VERSION 2.8.3)
project(aeroeus)

find_package(catkin REQUIRED)

catkin_package(
)

## rosrun euscollada collada2eus_urdfmodel $(rospack find aero_description)/robots/aero.urdf $(rospack find aero_description)/robots/aero.yaml aero.l

install(FILES aero-interface.l aero-hand.l aero-wheels.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
