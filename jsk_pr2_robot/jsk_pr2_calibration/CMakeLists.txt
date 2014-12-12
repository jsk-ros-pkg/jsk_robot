cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_calibration)

find_package(catkin REQUIRED COMPONENTS
  euscollada
  pr2_description
)

catkin_package()
