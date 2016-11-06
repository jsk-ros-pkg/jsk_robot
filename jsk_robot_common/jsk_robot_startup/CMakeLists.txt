cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_startup)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roseus # for geneus
)

catkin_python_setup()

add_message_files(
  FILES
  ActionEvent.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

generate_dynamic_reconfigure_options(
  config/OdometryOffsetReconfigure.cfg
  config/OdometryFeedbackWrapperReconfigure.cfg
  config/ConstantHeightFramePublisherReconfigure.cfg
)

catkin_package(
)

install(DIRECTORY lifelog util
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(lifelog/test/test-logging_pr2.test)
endif()
