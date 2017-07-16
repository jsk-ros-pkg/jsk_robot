^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_fetch_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------

* Enable safe teleop for fetch (`#801 <https://github.com/jsk-ros-pkg/jsk_robot/issues/801>`_)
  * fetch_gazebo_bringup.launch and fetch_teleop.xml both starts cmd_vel_mux, add roslaunch_add_file_check  fetch_gazebo_bringup.launch
  * add roslaunch_depends.py from https://github.com/ros/ros_comm/pull/998 to support if, https://github.com/ros/ros_comm/issues/953 could not load launch file with args directory
  * jsk_fetch_startup/package.xml: missing joy, topic_tools, fetch_teleop depends
  * launch/fetch_teleop.xml run unsafe_warning.l directory, not by roseus package
  * jsk_fetch_startup/package.xml: missing fetch_moveit_config depends
  * [jsk_fetch_startup] exclude fetch_bringup.launch from check
  * [jsk_fetch_startup] move unsafe_warning.l to jsk_robot_startup / enable unsafe_warning on fetch
  * [jsk_fetch_startup] add launch for safe teleop

* [jsk_pr2_startup] fix: init pose parameter typo for gazebo (`#753 <https://github.com/jsk-ros-pkg/jsk_robot/issues/753>`_)
  * [jsk_fetch_startup][fetch_gazebo_73b2.launch] fix: param name typo

* [jsk_fetch_startup][fetch_bringup.launch] fix: robot/type robot/name (`#752 <https://github.com/jsk-ros-pkg/jsk_robot/issues/752>`_)
* [jsk_fetch_startup][warning.py] fix: suppress warning: 'self.robot_state_msgs is not initialized' (`#750 <https://github.com/jsk-ros-pkg/jsk_robot/issues/750>`_ )

* Contributors: Kei Okada, Yuki Furuta

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* add network instruction to fetch README
* [jsk_fetch_startup] add fetch bringup launch files for gazebo (`#692 <https://github.com/jsk-ros-pkg/jsk_robot/issues/692>`_ )

  * add gazebo/fetch_gazebo_73b2.launch
  * add fetch_gazebo_bringup.launch
  * fetch_bringup.launch: cleanup launch file
  * fetch_bringup.launch: add launch_move_base args
  * add fetch_driver_msgs to package.xml
  * fetch_bringup.launch: add launch_moveit args
  * add more admin docs

* [jsk_robot_lifelog] move logging program from  jsk_pr2_startup/jsk_pr2_lifelog to jsk_robot_startup/lifelog (`#672 <https://github.com/jsk-ros-pkg/jsk_robot/issues/672>`_ )
* [jsk_fetch_startup/scripts/warning.py] bugfix: error with no robot_state msg is subscribed
* Contributors: Kei Okada, Masaki Murooka, Yuki Furuta

1.0.6 (2016-06-17)
------------------

1.0.5 (2016-04-18)
------------------

1.0.4 (2016-03-21)
------------------
* fetch_bringup.launch: fix arg boot_sound
* Contributors: Kei Okada

1.0.3 (2016-03-05)
------------------
* add jsk_fetch_robot package
* Contributors: Kei Okada

1.0.2 (2016-02-14)
------------------

1.0.1 (2015-11-19)
------------------

1.0.0 (2015-11-06 15:17)
------------------------

0.0.13 (2015-11-06 15:04)
-------------------------

0.0.12 (2015-11-06 14:47)
-------------------------

0.0.11 (2015-09-01)
-------------------

0.0.10 (2015-08-16)
-------------------

0.0.9 (2015-08-03)
------------------

0.0.8 (2015-07-16)
------------------

0.0.7 (2015-06-11)
------------------

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------

0.0.1 (2014-12-25)
------------------
