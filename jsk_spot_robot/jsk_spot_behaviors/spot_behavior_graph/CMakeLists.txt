cmake_minimum_required(VERSION 2.8.3)
project(spot_behavior_graph)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAM scripts/visualize_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATIN}
)
