cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_web)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  roswww
  baxter_description
  robot_state_publisher
  joint_state_publisher
  tf2_web_republisher
)

##Set the symbolic link below www
execute_process(COMMAND rospack find baxter_description OUTPUT_VARIABLE baxter_description_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ln -s ${baxter_description_PACKAGE_PATH}  ${PROJECT_SOURCE_DIR}/www/baxter_description OUTPUT_VARIABLE OUTPUT OUTPUT_STRIP_TRAILING_WHITESPACE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_web
#  CATKIN_DEPENDS rosbridge_server roswww
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
