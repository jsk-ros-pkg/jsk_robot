cmake_minimum_required(VERSION 2.8.3)
project(spot_basic_behaviors)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
