^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus_remote
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2016-06-17)
------------------

1.0.5 (2016-04-18)
------------------

1.0.4 (2016-03-21)
------------------

1.0.3 (2016-03-05)
------------------

1.0.2 (2016-02-14)
------------------
* [roseus_remote] Fix missing configurations
  See
  - https://github.com/ros/ros_tutorials/blob/jade-devel/roscpp_tutorials/CMakeLists.txt
  - https://github.com/ros/ros_tutorials/blob/jade-devel/roscpp_tutorials/package.xml
  Lint tool to check this kind of things
  - https://github.com/fkie/catkin_lint
  Modified:
  - jsk_robot_common/roseus_remote/CMakeLists.txt
  - jsk_robot_common/roseus_remote/package.xml
* Fix genjava problem caused by not listed message_generation
  Modified:
  - jsk_robot_common/roseus_remote/package.xml
* Contributors: Kentaro Wada

1.0.1 (2015-11-19)
------------------

1.0.0 (2015-11-06)
------------------

0.0.13 (2015-11-06)
-------------------

0.0.12 (2015-11-06)
-------------------

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
* [roseus_remote] Remove null characters from the strings from server and
  return result from $ macro.
  You can use $ like:
  (setq awesome-code-result ($ (awesome-code)))
* [roseus_remote] Add LAUNCH_PREFIX argument to roseus_eusserver.launch and
  roseus_eusclient.launch to use ports which require super user permission
* [remote_roseus] Add client in euslisp implementation
* [roseus_remote] use ros::spin instead of ros::spin-once loop
* [roseus_remote] add server powered by roseus
* [jsk_robot] add roseus remote server/client for silverhummer
* Contributors: Yuki Furuta, Ryohei Ueda

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
