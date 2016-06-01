cmake_minimum_required(VERSION 2.8.3)
project(fetcheus)
if($ENV{ROS_DISTRO} STREQUAL "hydro")
  message(STATUS "fetch is not supported on hydro")
  find_package(catkin)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED fetch_description)

catkin_package()

###
### fetch.l generation
###
if(EXISTS ${fetch_description_PREFIX}/share/fetch_description/robots/fetch.urdf)
  set(fetch_urdf ${fetch_description_PREFIX}/share/fetch_description/robots/fetch.urdf)
elseif(EXISTS ${fetch_description_SOURCE_PREFIX}/robots/fetch.urdf)
  set(fetch_urdf ${fetch_description_SOURCE_PREFIX}/robots/fetch.urdf)
else()
  message(WARNING "Could not found fetch.urdf in ${fetch_description_PREFIX}/share/fetch_description/urdf/${urdf_version}/fetch.urdf and ${fetch_description_SOURCE_PREFIX}/urdf/${urdf_version}/fetch.urdf")
endif()
if (EXISTS ${fetch_urdf})
  message(STATUS "Found fetch.urdf at ${fetch_urdf}")
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/fetch.l
    COMMAND rosrun euscollada collada2eus fetch.dae fetch.yaml fetch.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS fetch.dae fetch.yaml)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/fetch.dae
    COMMAND rosrun collada_urdf urdf_to_collada ${fetch_urdf} fetch.dae
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${fetch_urdf})

  # get collada-dom version
  find_package(PkgConfig)
  pkg_check_modules(COLLADA collada-dom>=2.4.4 )
  if ( ${COLLADA_FOUND} )
    add_custom_target(generate_fetch_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/fetch.l)
  else()
    pkg_check_modules(COLLADA collada-dom)
    message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
  endif()
endif()


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/fetcheus.test)
  add_rostest(test/fetch-moveit.test)
endif()


