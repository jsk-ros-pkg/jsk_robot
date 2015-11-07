
macro(compile_naoqi_model robot_name urdf_version)
  if(EXISTS ${${robot_name}_description_PREFIX}/share/${robot_name}_description/urdf/${urdf_version}/${robot_name}.urdf)
    set(${robot_name}_urdf ${${robot_name}_description_PREFIX}/share/${robot_name}_description/urdf/${urdf_version}/${robot_name}.urdf)
  elseif(EXISTS ${${robot_name}_description_SOURCE_PREFIX}/urdf/${urdf_version}/${robot_name}.urdf)
    set(${robot_name}_urdf ${${robot_name}_description_SOURCE_PREFIX}/urdf/${urdf_version}/${robot_name}.urdf)
  else()
    message(WARNING "Could not found ${robot_name}.urdf in ${${robot_name}_description_PREFIX}/share/${robot_name}_description/urdf/${urdf_version}/${robot_name}.urdf and ${${robot_name}_description_SOURCE_PREFIX}/urdf/${urdf_version}/${robot_name}.urdf")
  endif()
  if (EXISTS ${${robot_name}_urdf})
    message(STATUS "Found ${robot_name}.urdf at ${${robot_name}_urdf}")
    add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${robot_name}.l
      COMMAND rosrun euscollada collada2eus ${robot_name}.dae ${robot_name}.yaml ${robot_name}.l
      COMMAND sed -i 's@JulietteY20MP@${robot_name}@g' ${robot_name}.l
      COMMAND sed -i 's@julietteY20MP@${robot_name}@g' ${robot_name}.l
      COMMAND sed -i 's@juliettey20mp@${robot_name}@g' ${robot_name}.l
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      DEPENDS ${robot_name}.dae)
    add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${robot_name}.dae
      COMMAND rosrun collada_urdf urdf_to_collada ${${robot_name}_urdf} ${robot_name}.dae || echo "ok?" # urdf_to_collada fail to exit program, but generated dae is ok.
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      DEPENDS ${${robot_name}_urdf})

    find_package(${robot_name}_meshes)
    if(${robot_name}_meshes_FOUND)
      add_custom_target(generate_${robot_name}_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/${robot_name}.l)
    else()
      message(WARNING "Please install ros-\$ROS_DISTRO-${robot_name}-meshes manually")
    endif()
  endif()
endmacro(compile_naoqi_model robot_name urdf_version)

#    add_rostest(test/${robot_name}eus.test)
