cmake_minimum_required(VERSION 3.8)
project(dynamixel_detachable_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rcl_interfaces REQUIRED)
find_package(roseus REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelMotor.msg"
  "msg/HandDescription.msg"
  "msg/ToolState.msg"
  "srv/GetToolState.srv"
  "srv/SetTool.srv"
  DEPENDENCIES builtin_interfaces
)

ament_python_install_package(dynamixel_detachable_hand_common
  PACKAGE_DIR src/dynamixel_detachable_hand_common
)
ament_python_install_package(hand_controller
  PACKAGE_DIR src/hand_controller
)

message(STATUS "dynamixel_detachable_hand scripts use their source shebangs")

set(_python_entrypoint_scripts
  scripts/hand_command.py
  scripts/joint_state_coupling_relay.py
  scripts/modular_hand_manager.py
  scripts/print_robot_description.py
  scripts/render_tool_views.py
  scripts/static_transform_publisher_xyz_rpy.py
  scripts/tool_auto_detector.py
)

install(PROGRAMS
  ${_python_entrypoint_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  euslisp
  figs
  launch
  meshes
  udev
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_hand_controller test/test_hand_controller.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_smoke_cli test/test_smoke_cli.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_launch_files test/test_launch_files.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_modular_hand_manager test/test_modular_hand_manager.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
