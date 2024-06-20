cmake_minimum_required(VERSION 2.8.3)
project(seedhand_std)

find_package(catkin REQUIRED COMPONENTS
  aero_std
  std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_definitions(-std=c++11)

add_service_files(
  FILES
  SetHand.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seedhand_interface
  CATKIN_DEPENDS
  aero_std
  )

add_library(seedhand_interface src/SeedHandInterface.cc)
target_link_libraries(seedhand_interface ${catkin_LIBRARIES})
add_dependencies(seedhand_interface ${PROJECT_NAME}_gencpp)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_test src/SeedHandInterface_test.cc)
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} seedhand_interface)

add_executable(seedhand_server src/SeedHandServer.cc)
target_link_libraries(seedhand_server ${catkin_LIBRARIES} seedhand_interface)
add_dependencies(seedhand_server seedhand_server_generate_messages_cpp)
