cmake_minimum_required(VERSION 2.8.3)
project(peppereus)

if($ENV{ROS_DISTRO} STREQUAL "indigo") # indigo requres to add find_pacakge(roseus) to compile roseus message
  set(ROSEUS_package roseus)
endif()
find_package(catkin REQUIRED
  naoqieus
  ${ROSEUS_package} # required in indigo
  rostest)
find_package(pepper_description) # Just in case when description is not released. Avoid compile failing

catkin_package()

###
### pepper.l generation
###
compile_naoqi_model(pepper pepper1.0_generated_urdf)
if(pepper_meshes_FOUND AND CATKIN_ENABLE_TESTING)
  add_rostest(test/peppereus.test)
endif()


