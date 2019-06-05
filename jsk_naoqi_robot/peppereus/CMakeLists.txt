cmake_minimum_required(VERSION 2.8.3)
project(peppereus)

find_package(catkin REQUIRED naoqieus roseus rostest)
find_package(pepper_description) # Just in case when description is not released. Avoid compile failing

catkin_package()

###
### pepper.l generation
###
compile_naoqi_model(pepper pepper1.0_generated_urdf)
if(pepper_meshes_FOUND)
  add_rostest(test/peppereus.test)
endif()


