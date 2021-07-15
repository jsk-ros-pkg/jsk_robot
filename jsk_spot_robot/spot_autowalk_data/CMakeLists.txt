cmake_minimum_required(VERSION 3.0.2)
project(spot_autowalk_data)

find_package(catkin REQUIRED)

add_custom_target(${PROJECT_NAME}_download_autowalk_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/download_autowalk_data.py)

catkin_package(
)
