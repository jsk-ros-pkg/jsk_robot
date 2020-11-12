cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_teleop)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_srvs
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        std_srvs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    joystick_teleop
    src/joystick_teleop.cpp
)
target_link_libraries(
    joystick_teleop
    ${catkin_LIBRARIES}
)
