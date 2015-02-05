cmake_minimum_required(VERSION 2.8.3)
project(pepper_description)

find_package(catkin REQUIRED)

catkin_package()

set(_simulator_sdk_path $ENV{HOME}/Downloads/simulator-sdk-2.1.2.17-linux64)
set(_pepper_name JULIETTEY20MP)
set(_pepper_xml ${_simulator_sdk_path}/share/alrobotmodel/models/${_pepper_name}.xml)
if(EXISTS ${_pepper_xml})
  file(GLOB _meshfiles ${_simulator_sdk_path}/share/alrobotmodel/meshes/juliette/DVT/*.mesh
                       ${_simulator_sdk_path}/share/alrobotmodel/meshes/juliette/*.mesh) # detail
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/meshes/
    COMMAND cmake -E make_directory ${PROJECT_SOURCE_DIR}/meshes/)
  foreach(_meshfile ${_meshfiles})
    get_filename_component(_meshfile_name ${_meshfile} NAME_WE)
    add_custom_command(OUTPUT ${_meshfile_name}Link.dae
      COMMAND OgreXMLConverter -q ${_meshfile}
      COMMAND assimp export ${_meshfile}.xml ${PROJECT_SOURCE_DIR}/meshes/${_meshfile_name}Link.dae
      COMMAND sed -i 's@<unit name="meter" meter="1" />@<unit name="meter" meter="10" />@g' ${PROJECT_SOURCE_DIR}/meshes/${_meshfile_name}Link.dae
      DEPENDS ${PROJECT_SOURCE_DIR}/meshes/)
    add_custom_target(generate_${_meshfile_name}.dae ALL
      DEPENDS ${_meshfile_name}Link.dae)
    message("-- convert ${_meshfile}")
  endforeach()
  file(GLOB _meshfiles ${_simulator_sdk_path}/share/alrobotmodel/meshes/juliette/P2/*.mesh
                       ${_simulator_sdk_path}/share/alrobotmodel/meshes/juliette/*.mesh) # simple
  foreach(_meshfile ${_meshfiles})
    get_filename_component(_meshfile_name ${_meshfile} NAME_WE)
    add_custom_command(OUTPUT ${_meshfile_name}Link.stl
      COMMAND OgreXMLConverter -q ${_meshfile}
      COMMAND assimp export ${_meshfile}.xml ${PROJECT_SOURCE_DIR}/meshes/${_meshfile_name}Link_0.10.stl -fstlb
      DEPENDS ${PROJECT_SOURCE_DIR}/meshes/)
    add_custom_target(generate_${_meshfile_name}.stl ALL
      DEPENDS ${_meshfile_name}Link.stl)
    message("-- convert ${_meshfile}")
  endforeach()
  add_custom_command(OUTPUT pepper.urdf
    COMMAND cmake -E make_directory ${PROJECT_SOURCE_DIR}/urdf/pepper_generated_urdf
    COMMAND ${_simulator_sdk_path}/bin/robot_description --urdf --mesh-path=share/alrobotmodel/meshes/ --model-file=${_pepper_xml} > ${PROJECT_SOURCE_DIR}/urdf/pepper_generated_urdf/${_pepper_name}.urdf
    COMMAND rosrun naoqi_tools generate_urdf.py -i ${PROJECT_SOURCE_DIR}/urdf/pepper_generated_urdf/${_pepper_name}.urdf -x urdf
    COMMAND sed -i s/Tibia/TorsoLink/g ${PROJECT_SOURCE_DIR}/urdf/pepper_generated_urdf/pepper.urdf
    COMMAND sed -i 's/0 0 -0.334/0 0 -0.800/g' ${PROJECT_SOURCE_DIR}/urdf/pepper_generated_urdf/pepper.urdf
    )
  add_custom_target(generate_pepper_urdf ALL
    DEPENDS pepper.urdf)
else()
  message(WARNING "please download simulator-sdk under Downloads directory")
endif()


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

