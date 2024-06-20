cmake_minimum_required(VERSION 2.8.3)
project(touchence_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
)

add_definitions(-std=c++11)

include_directories(
  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

add_executable(touchence_serial_node src/touchence_serial_node.cpp)

target_link_libraries(touchence_serial_node
   ${catkin_LIBRARIES}
)
