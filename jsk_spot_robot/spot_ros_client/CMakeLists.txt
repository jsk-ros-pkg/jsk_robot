cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_client)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/demo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
