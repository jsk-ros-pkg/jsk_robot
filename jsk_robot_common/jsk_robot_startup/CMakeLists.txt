cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

# for ROS indigo compile without c++11 support
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STRGREATER "indigo")
    add_compile_options(-std=c++11)
    message(STATUS "Building with C++11 support")
  else() 
    message(STATUS "ROS Indigo and before: building without C++11 support")
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
  dynamic_reconfigure
  jsk_topic_tools
  mongodb_store
  sensor_msgs
  urdf
)

find_package(Boost REQUIRED COMPONENTS program_options thread system)
if ("${BOOST_MAJOR_VERSION}" VERSION_EQUAL "1" AND "${Boost_MINOR_VERSION}" VERSION_GREATER "46")
  find_package(Boost REQUIRED COMPONENTS chrono)
endif()

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

# jsk_topic_tools >= 2.2.27 only available from indigo
# mongodb_store >= 0.4.4 required in kinetic
# mongodb_store >= 0.5.2 required in melodic
if($ENV{ROS_DISTRO} STRGREATER "hydro")
  if (("${mongodb_store_VERSION}" VERSION_GREATER "0.4.3" AND "${mongodb_store_VERSION}" VERSION_LESS "0.5.0")
      OR "${mongodb_store_VERSION}" VERSION_GREATER "0.5.1")
    add_lifelog_nodelet(src/lightweight_logger_nodelet.cpp "jsk_robot_lifelog/LightweightLogger" "lightweight_logger")
  endif()
endif()

add_library(jsk_robot_lifelog SHARED
  ${${PROJECT_NAME}_LIFELOG_NODELET_SRCS}
  src/message_store_singleton.cpp)

target_link_libraries(jsk_robot_lifelog
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(jsk_robot_lifelog ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

find_package(PkgConfig)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
if ("${URDFDOM_VERSION}" VERSION_EQUAL "1.0.0" OR "${URDFDOM_VERSION}" VERSION_GREATER "1.0.0")
  add_definitions(-DURDFDOM_1_0_0_API)
endif()

add_executable(joint_states_throttle
  src/joint_states_throttle_node.cpp)
target_link_libraries(joint_states_throttle
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(joint_states_throttle ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

install(DIRECTORY lifelog util launch images config cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(TARGETS jsk_robot_lifelog joint_states_throttle ${${PROJET_NAME}_LIFELOG_NODELET_EXECUTABLES}
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
