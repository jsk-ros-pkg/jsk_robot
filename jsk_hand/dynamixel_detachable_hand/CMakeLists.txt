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

set(_venv_python "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python3")
if(EXISTS "${_venv_python}")
  set(DYNAMIXEL_DETACHABLE_HAND_PYTHON "${_venv_python}")
else()
  set(DYNAMIXEL_DETACHABLE_HAND_PYTHON "${Python3_EXECUTABLE}")
endif()
message(STATUS "dynamixel_detachable_hand scripts use ${DYNAMIXEL_DETACHABLE_HAND_PYTHON}")

set(_python_entrypoint_scripts
  scripts/hand_command.py
  scripts/joint_state_coupling_relay.py
  scripts/modular_hand_manager.py
  scripts/print_robot_description.py
  scripts/tool_auto_detector.py
)

set(_generated_python_entrypoint_scripts)
foreach(_script IN LISTS _python_entrypoint_scripts)
  set(_source_script "${CMAKE_CURRENT_SOURCE_DIR}/${_script}")
  set(_generated_script "${CMAKE_CURRENT_BINARY_DIR}/${_script}")
  get_filename_component(_generated_script_dir "${_generated_script}" DIRECTORY)
  file(MAKE_DIRECTORY "${_generated_script_dir}")
  file(READ "${_source_script}" _script_content)
  string(REGEX REPLACE "^#![^\n]*(\n|$)" "#!${DYNAMIXEL_DETACHABLE_HAND_PYTHON}\n"
         _script_content "${_script_content}")
  file(WRITE "${_generated_script}" "${_script_content}")
  list(APPEND _generated_python_entrypoint_scripts "${_generated_script}")
endforeach()

install(PROGRAMS
  ${_generated_python_entrypoint_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  euslisp
  figs
  launch
  udev
  urdf
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
