cmake_minimum_required(VERSION 2.8.3)
project(spot_person_lead_behaviors)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS node_scripts/car_tracker.py node_scripts/person_tracker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
