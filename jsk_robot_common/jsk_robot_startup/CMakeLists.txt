cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/OdometryOffsetReconfigure.cfg
  config/OdometryFeedbackWrapperReconfigure.cfg
  config/ConstantHeightFramePublisherReconfigure.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY lifelog util
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Disabled since cannot set environment variables for rostest for now.
  # add_rostest(lifelog/test/logger_pr2.test)
endif()
