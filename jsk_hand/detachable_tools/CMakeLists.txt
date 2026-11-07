# ROS_VERSION is unset only on pre-ros_environment (ROS 1) distros
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
  cmake_minimum_required(VERSION 3.0.2)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()
project(detachable_tools)

# no messages of its own: diagnostic_msgs, std_msgs and std_srvs are used

set(_python_entrypoint_scripts
  scripts/hand_command.py
  scripts/joint_state_coupling_relay.py
  scripts/modular_hand_manager.py
  scripts/print_robot_description.py
  scripts/render_tool_views.py
  scripts/static_transform_publisher_xyz_rpy.py
)
# ROS 1 only: they use dynamixel_general_hw topics / roslaunch.
set(_ros1_scripts
  scripts/dual_hand_auto.py
  scripts/dynamixel_temperature_guages.py
  scripts/monitor_connection.py
)
# ROS 2 only
set(_ros2_scripts
  scripts/apriltag_fork_sim.py
  scripts/detect_hand_tools.py
  scripts/loop_closure_relay.py
)

# <name>.launch (ROS 1) and <name>.launch.xml (ROS 2, also installed as <name>.launch) share arguments
set(_launch_names
  dual_hand_auto
  dual_detacher
  dual_hand
  dual_hand_optional
  hand_control
  hand_detacher
  hand_model
  left_detacher
  left_hand
  right_detacher
  right_hand
)
# ROS 1 only, kept for compatibility
set(_ros1_launch_names
  hand_tool_model
)

if("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(jsk_ros1_ros2_compat REQUIRED)
  # roseus generates the EusLisp messages of the packages found here (ament_package hook)
  find_package(roseus REQUIRED)
  foreach(_msg_pkg
      action_msgs builtin_interfaces control_msgs diagnostic_msgs
      sensor_msgs std_msgs std_srvs trajectory_msgs)
    find_package(${_msg_pkg} REQUIRED)
  endforeach()

  ament_python_install_package(detachable_tools_common
    PACKAGE_DIR src/detachable_tools_common
  )
  ament_python_install_package(hand_controller
    PACKAGE_DIR src/hand_controller
  )

  jsk_ros1_ros2_compat_install_python(PROGRAMS ${_python_entrypoint_scripts} ${_ros2_scripts})

  install(DIRECTORY
    config
    euslisp
    figs
    meshes
    rviz
    srdf
    udev
    urdf
    DESTINATION share/${PROJECT_NAME}
  )
  install(FILES launch/surgenoid_needle_holder_display.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
  foreach(_launch ${_launch_names})
    install(FILES launch/${_launch}.launch.xml
      DESTINATION share/${PROJECT_NAME}/launch)
    install(FILES launch/${_launch}.launch.xml
      DESTINATION share/${PROJECT_NAME}/launch
      RENAME ${_launch}.launch)
  endforeach()

  install(FILES
    README.md
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

else() # ROS 1

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS ${_python_entrypoint_scripts} ${_ros1_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config euslisp figs meshes rviz srdf udev urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
foreach(_launch ${_launch_names} ${_ros1_launch_names})
  install(FILES launch/${_launch}.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
endforeach()
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

endif()
