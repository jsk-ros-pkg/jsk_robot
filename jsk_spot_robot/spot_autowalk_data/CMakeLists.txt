cmake_minimum_required(VERSION 2.8.3)
project(spot_autowalk_data)

find_package(catkin REQUIRED)

add_custom_target(${PROJECT_NAME}_download_autowalk_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/download_autowalk_data.py)

catkin_package()

install(DIRECTORY autowalk
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/view_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
