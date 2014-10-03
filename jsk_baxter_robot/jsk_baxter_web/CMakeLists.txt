cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_web)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  baxter_description
  robot_state_publisher
  tf2_web_republisher
)

execute_process(COMMAND rospack find baxter_description OUTPUT_VARIABLE baxter_description_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND cp -Ru ${baxter_description_PACKAGE_PATH}  ${PROJECT_SOURCE_DIR}/www/ OUTPUT_VARIABLE OUTPUT OUTPUT_STRIP_TRAILING_WHITESPACE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_web
#  CATKIN_DEPENDS rosbridge_server roswww
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
