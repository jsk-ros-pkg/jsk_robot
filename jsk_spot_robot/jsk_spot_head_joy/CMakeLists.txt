cmake_minimum_required(VERSION 2.8.3)
project(jsk_spot_head_joy)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/joy_topic_completion.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
