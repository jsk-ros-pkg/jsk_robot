cmake_minimum_required(VERSION 2.8.3)
project(aeroeus)

find_package(catkin REQUIRED)

catkin_package(
)

install(FILES aero-typeF_fcsc-interface.l aero-hand.l aero-wheels.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
