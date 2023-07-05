cmake_minimum_required(VERSION 2.8.3)
project(spot_behavior_manager_server)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS node_scripts/behavior_manager_server.py node_scripts/interactive_behavior_executor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
