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

find_package(catkin COMPONENTS # remove REQUIRED for mongodb
  angles
  dynamic_reconfigure
  jsk_topic_tools
  message_generation
  mongodb_store
  sensor_msgs
  urdf
)
if(NOT(dynamic_reconfigure_FOUND AND jsk_topic_tools_FOUND AND sensor_msgs_FOUND AND urdf_FOUND))
  message(FATAL_ERROR "requried package is not found
dynamic_reconfigure_FOUND : ${dynamic_reconfigure_FOUND}
jsk_topic_tools_FOUND : ${jsk_topic_tools_FOUND}
sensor_msgs_FOUND : ${sensor_msgs_FOUND}
urdf_FOUND : ${urdf_FOUND}")
endif()


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

add_message_files(
  FILES
  RoomLight.msg
  Email.msg
  EmailBody.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
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
if(mongodb_store_FOUND)
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

else(mongodb_store_FOUND)
  message(AUTHOR_WARNING "===\nros-$ENV{ROS_DISTRO}-mongodb-store package is not found, may be it is not releaed yet\n===")
endif(mongodb_store_FOUND)

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

if($ENV{ROS_DISTRO} STRGREATER "indigo") # quadruped_joystick_teleop.cpp uses c++11 and also msg.volume
  add_executable(quadruped_joystick_teleop
    src/quadruped_joystick_teleop.cpp)
  target_link_libraries(quadruped_joystick_teleop
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES})
  add_dependencies(quadruped_joystick_teleop ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
  install(TARGETS quadruped_joystick_teleop
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

install(DIRECTORY lifelog util launch images config cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
file(GLOB_RECURSE SCRIPT_PROGRAMS lifelog/*)
foreach(SCRIPT_PROGRAM ${SCRIPT_PROGRAMS})
  if("${SCRIPT_PROGRAM}" MATCHES ".*\\.py$")
    catkin_install_python(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    install(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endforeach()

if(mongodb_store_FOUND)
install(TARGETS jsk_robot_lifelog
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif(mongodb_store_FOUND)
install(TARGETS joint_states_throttle ${${PROJET_NAME}_LIFELOG_NODELET_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Disabled since cannot set environment variables for rostest for now.
  # see https://github.com/PR2/pr2_simulator/pull/126
  # add_rostest(lifelog/test/logger_pr2.test)
  add_rostest(lifelog/test/test_common_logger.test)
endif()
