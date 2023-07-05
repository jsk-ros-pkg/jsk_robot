cmake_minimum_required(VERSION 2.8.3)
project(spot_basic_behaviors)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/car_tracker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
