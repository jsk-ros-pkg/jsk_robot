cmake_minimum_required(VERSION 2.8.3)
project(jsk_robot_utils)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/angle_vector_compress.test)
  add_rostest(test/joint_state_compressor.test)
endif()

file(GLOB_RECURSE SCRIPT_PROGRAMS scripts/*)
foreach(SCRIPT_PROGRAM ${SCRIPT_PROGRAMS})
  if("${SCRIPT_PROGRAM}" MATCHES ".*\\.py$")
    catkin_install_python(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    install(PROGRAMS ${SCRIPT_PROGRAM} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
  endif()
endforeach()
install(DIRECTORY euslisp test
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
