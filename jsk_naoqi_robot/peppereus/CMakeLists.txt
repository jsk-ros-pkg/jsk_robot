cmake_minimum_required(VERSION 2.8.3)
project(peppereus)

find_package(catkin REQUIRED pepper_description naoqieus rostest)

catkin_package()

###
### pepper.l generation
###
compile_naoqi_model(pepper pepper1.0_generated_urdf)
if(pepper_meshes_FOUND)
  add_rostest(test/peppereus.test)
endif()


