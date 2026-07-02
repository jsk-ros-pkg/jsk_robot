cmake_minimum_required(VERSION 2.8.3)
project(mycoboteus)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if($ENV{ROS_DISTRO} STREQUAL "indigo") # indigo requres to add find_pacakge(roseus) to compile roseus message
  set(ROSEUS_package roseus)
endif()
find_package(catkin REQUIRED COMPONENTS
  ${ROSEUS_package} # required in indigo
)


find_package(mycobot_description) # Just in case when description is not released. Avoid compile failing

catkin_package()

if(NOT mycobot_description_FOUND)
  message(WARNING "mycobot_description is not found, so skip generating eus models.")
  message(WARNING "Install mycobot_description from https://github.com/tongtybj/mycobot_ros.git")
  return()
endif()

find_package(euscollada) # Just in case when euscollada not released. Avoid compile failing
if(NOT euscollada_FOUND)
  message(WARNING "euscollada is not found, so skip generating eus models.")
  return()
endif()


###########
## Build ##
###########
set(_mycobot_urdf ${mycobot_description_SOURCE_PREFIX}/urdf/mycobot_280_m5)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("mycobot_urdf: ${_mycobot_urdf}")
message("collada2eus: ${_collada2eus}")

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/mycobot.l
  COMMAND echo "${_collada2eus} -I ${_mycobot_urdf}/mycobot_280_m5.urdf -N mycobot -C ${PROJECT_SOURCE_DIR}/mycobot.yaml -O ${PROJECT_SOURCE_DIR}/mycobot.l"
  COMMAND ${_collada2eus} -I ${_mycobot_urdf}/mycobot_280_m5.urdf -N mycobot -C ${PROJECT_SOURCE_DIR}/mycobot.yaml -O ${PROJECT_SOURCE_DIR}/mycobot.l
  DEPENDS ${_mycobot_urdf}/mycobot_280_m5.urdf ${PROJECT_SOURCE_DIR}/mycobot.yaml ${_collada2eus})
add_custom_target(compile_mycobot ALL DEPENDS ${PROJECT_SOURCE_DIR}/mycobot.l)

#find_package(catkin REQUIRED COMPONENTS
#  mycobot_description
#)

#catkin_package()

#set(_mycobot_urdf ${mycobot_description_SOURCE_PREFIX}/urdf/mycobot_280_m5)
#set(_venv_dir ${PROJECT_BINARY_DIR}/urdf2eus_venv)

#add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/mycobot.l
#  COMMAND python3 -m venv ${_venv_dir}
#  COMMAND ${_venv_dir}/bin/pip install urdfeus
#  COMMAND ${_venv_dir}/bin/urdf2eus 
#    ${_mycobot_urdf}/mycobot_280_m5.urdf 
#    ${PROJECT_SOURCE_DIR}/mycobot.l 
#    --name mycobot 
#    --yaml-path ${PROJECT_SOURCE_DIR}/mycobot.yaml
#  COMMAND rm -rf ${_venv_dir}
#  DEPENDS ${_mycobot_urdf}/mycobot_280_m5.urdf ${PROJECT_SOURCE_DIR}/mycobot.yaml
#  COMMENT "Generating mycobot.l using urdf2eus in temporary venv")

#add_custom_target(compile_mycobot ALL DEPENDS ${PROJECT_SOURCE_DIR}/mycobot.l)

#############
## Install ##
#############

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES mycobot.l mycobot-interface.l mycobot-utils.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  # Test only roseus
  add_rostest(test/test-mycobot.test)
  # Test with Kinematic Simulator
  add_rostest(test/test-mycobot-kinematic-simulator.test)
  # Test with Eus + Moveit
  add_rostest(test/test-mycobot-eus-moveit.test ARGS headless:=true)
endif()
