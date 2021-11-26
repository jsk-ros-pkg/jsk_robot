cmake_minimum_required(VERSION 2.8.3)
project(jsk_spot_apps)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY apps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
