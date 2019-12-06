cmake_minimum_required(VERSION 2.8.3)
project(naoeus)

find_package(catkin REQUIRED naoqieus roseus rostest)
find_package(nao_description) # Just in case when description is not released. Avoid compile failing

catkin_package()

###
### nao.l generation
###
compile_naoqi_model(nao naoV50_generated_urdf)
if(nao_meshes_FOUND)
  add_rostest(test/naoeus.test)
endif()
