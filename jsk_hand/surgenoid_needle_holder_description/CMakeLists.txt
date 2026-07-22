cmake_minimum_required(VERSION 3.8)
project(surgenoid_needle_holder_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/loop_closure_relay.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
