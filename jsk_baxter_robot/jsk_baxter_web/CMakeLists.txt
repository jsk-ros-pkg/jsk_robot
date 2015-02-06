cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_web)

find_package(catkin REQUIRED)
find_package(baxter_description)
if(baxter_description_FOUND)
  if (EXISTS ${baxter_description_SOURCE_DIR}/meshes)
    set(_baxter_description_location ${baxter_description_SOURCE_DIR}/meshes)
  elseif (EXISTS ${baxter_description_SOURCE_PREFIX}/meshes)
    set(_baxter_description_location ${baxter_description_SOURCE_PREFIX}/meshes)
  elseif (EXISTS ${baxter_description_PREFIX}/share/baxter_description/meshes)
    set(_baxter_description_location ${baxter_description_PREFIX}/share/baxter_description/meshes)
  else (EXISTS ${baxter_description_SOURCE_DIR}/meshes)
    message(FATAL_ERROR "cannot find baxter_description")
  endif(EXISTS ${baxter_description_SOURCE_DIR}/meshes)

  execute_process(COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/www/baxter_description/meshes/)
  execute_process(COMMAND cp -Ru  ${_baxter_description_location}  ${PROJECT_SOURCE_DIR}/www/baxter_description/ OUTPUT_VARIABLE OUTPUT OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_web
#  CATKIN_DEPENDS rosbridge_server roswww
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
