cmake_minimum_required(VERSION 2.8.3)
project(peppereus)

find_package(catkin REQUIRED naoqi_driver naoqi_sensors nao_pose pr2eus diagnostic_aggregator naoqi_msgs roseus pepper_description euscollada)

catkin_package()

###
### pepper.l generation
###
if(EXISTS ${pepper_description_PREFIX}/share/pepper_description/urdf/pepper1.0_generated_urdf/pepper.urdf)
  set(pepper_urdf ${pepper_description_PREFIX}/share/pepper_description/urdf/pepper1.0_generated_urdf/pepper.urdf)
elseif(EXISTS ${pepper_description_SOURCE_PREFIX}/urdf/pepper1.0_generated_urdf/pepper.urdf)
  set(pepper_urdf ${pepper_description_SOURCE_PREFIX}/urdf/pepper1.0_generated_urdf/pepper.urdf)
else()
  message(WARNING "Could not found pepper.urdf in ${pepper_description_PREFIX}/share/pepper_description/urdf/pepper1.0_generated_urdf/pepper.urdf and ${pepper_description_SOURCE_PREFIX}/urdf/pepper1.0_generated_urdf/pepper.urdf")
endif()
if (EXISTS ${pepper_urdf})
  message(STATUS "Found pepper.urdf at ${pepper_urdf}")
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/pepper.l
    COMMAND rosrun euscollada collada2eus pepper.dae pepper.yaml pepper.l
    COMMAND sed -i 's@julietteY20MP@pepper@g' pepper.l
    COMMAND sed -i 's@juliettey20mp@pepper@g' pepper.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS pepper.dae)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/pepper.dae
    COMMAND rosrun collada_urdf urdf_to_collada ${pepper_urdf} pepper.dae || echo "ok?" # urdf_to_collada fail to exit program, but generated dae is ok.
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${pepper_urdf})

  find_package(pepper_meshes)
  if(pepper_meshes_FOUND)
    add_custom_target(generate_pepper_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/pepper.l)
  else()
    message(WARNING "Please install ros-\$ROS_DISTRO-pepper-meshes manually")
  endif()
endif()
