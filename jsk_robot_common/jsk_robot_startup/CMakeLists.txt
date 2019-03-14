cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

# for ROS indigo compile without c++11 support
if(DEFINED ENV{ROS_DISTRO})
  if(NOT $ENV{ROS_DISTRO} STREQUAL "indigo")
    add_compile_options(-std=c++11)
    message(STATUS "Building with C++11 support")
  else() 
    message(STATUS "ROS Indigo: building without C++11 support")
  endif()
else()
  message(STATUS "Environmental variable ROS_DISTRO not defined, checking OS version")
  file(STRINGS /etc/os-release RELEASE_CODENAME
		REGEX "VERSION_CODENAME=")
  if(NOT ${RELEASE_CODENAME} MATCHES "trusty") 
    add_compile_options(-std=c++11)
    message(STATUS "OS distro is not trusty: building with C++11 support")
  else()
    message(STATUS "Ubuntu Trusty: building without C++11 support")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  jsk_topic_tools
  mongodb_store
  sensor_msgs
  urdf
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/OdometryOffsetReconfigure.cfg
  cfg/OdometryFeedbackWrapperReconfigure.cfg
  cfg/ConstantHeightFramePublisherReconfigure.cfg
  cfg/JointStatesThrottle.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

macro(add_lifelog_nodelet _cpp _cls _bin)
  jsk_nodelet(${_cpp} ${_cls} ${_bin}
    ${PROJECT_NAME}_LIFELOG_NODELET_SRCS
    ${PROJECT_NAME}_LIFELOG_NODELET_EXECUTABLES)
endmacro()

add_lifelog_nodelet(src/lightweight_logger_nodelet.cpp "jsk_robot_lifelog/LightweightLogger" "lightweight_logger")

add_library(jsk_robot_lifelog SHARED
  ${${PROJECT_NAME}_LIFELOG_NODELET_SRCS}
  src/message_store_singleton.cpp)

target_link_libraries(jsk_robot_lifelog
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(jsk_robot_lifelog ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(joint_states_throttle
  src/joint_states_throttle_node.cpp)
target_link_libraries(joint_states_throttle
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(joint_states_throttle ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

install(DIRECTORY lifelog util launch image config cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(TARGETS jsk_robot_lifelog ${${PROJET_NAME}_LIFELOG_NODELET_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Disabled since cannot set environment variables for rostest for now.
  # see https://github.com/PR2/pr2_simulator/pull/126
  # add_rostest(lifelog/test/logger_pr2.test)
endif()
