^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pr2_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2015-07-16)
------------------
* [jsk_pr2_startup] add option map_frame to change eng2/eng8
* [jsk_pr2_startup/pr2_gazebo.launch] include rgbd_launch to rectify kinect rgb image
* [jsk_pr2_startup] add pr2_gazebo.launch
* [jsk_pr2_startup] use env ROBOT for including machine tags
* [jsk_pr2_startup/jsk_pr2_sensors/kinect_head.launch] add deprecated relay for openni_c2 rgb, depth, depth_registered topics
* [jsk_pr2_startup] use kinect_head(_c2) instead of openni(_c2) following pr2 default naming
* [jsk_pr2_startup/jsk_pr2_move_base] fix topic name /base_scan_filtered -> base_scan
* [jsk_pr2_startup/jsk_pr2_move_base] split name space along with modules; use hydro-based costmap params
* [jsk_pr2_startup/jsk_pr2_move_base] enable clear params option to move_base_node; increase nice value
* [jsk_pr2_start_up] set ROBOT=pr2 in rossetpr10XX
* Contributors: Kentaro Wada, Yuki Furuta

0.0.7 (2015-06-11)
------------------
* solve not updating problem after recharge
* speak the percentage of the battery with min charge
* [jsk_pr2_startup] warn more detail batrery information
* Contributors: Yuki Furuta, Chi Wun Au

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------
* [jsk_pr2_startup] Add rossetpr1012 and rossetpr1040 automatically by env-hooks
* add deps jsk_interactive_marker for jsk_pr2_startup
* add pr2 deps package for build test
* use only catkin; add deps for running pr2.launch
* add dwa_local_planner to build/run dependencies
* add move_base_msgs, roseus to build dependencies
* update readme for launching mongodb by multi users
* [jsk_pr2_startup] Remove collider related roslaunch
* launch mongodb when robot starts
* add action_result_db to record action result/goal and joint_states
* add tilt_scan_interpolated topic
* add openni_cloud_self_filter to launch as default and publish color pointclouds
* tested objectdetection for all camera on PR2
* tested on PR2
* fix option of db_client launch
* add debug message to objectdetection_db.py
* [jsk_pr2_robot] Use jsk_network_tools' euslisp code to
  compress/decompress joint angles
* migrate pr2 move_base, objectdetection db from postgre to mongodb
* Contributors: Ryohei Ueda, Yuki Furuta, Yuto Inagaki

0.0.4 (2015-01-30)
------------------
* [jsk_pr2_startup] Remove unrequired return-from in pr2-compressed-angle-vector-interface
* rename pr2-compressed-angle-vector-interface.l
* use string to set data
* fix typo
* update to work
* add jsk_pr2_teleop

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------
* add install commands to cmake
* [jsk_pr2_startup] Disable collider node, it's out of date
* Merge pull request #232 from garaemon/rename-hydro-recognition
  [jsk_pr2_startup] rename hydro_recognition.launch to people_detection.launch and start it up default
* [jsk_pr2_startup] Remove torso_lift_link from self filtering of
  tilt laser to avoid too much filtering of points. And update padding
  of shoulder links to remove veiling noise
* [jsk_pr2_startup] rename hydro_recognition.launch to people_detection.launch
  and start it up in default.
* Merge pull request #230 from garaemon/move-image-processing-to-c2
  [jsk_pr2_startup] Move several image processing to c2 to avoid heavy network communication between c1 and c2
* [jsk_pr2_startup] Move several image processing to c2 to avoid heavy
  network communication between c1 and c2
* [jsk_pr2_startup] Throttle before applying image_view2 to decrease
  CPU load
* use robot-actions.l
* Fix parameter namespace to slow down pr2_gripper_sensor_action
* Use longer priod to check openni soundness
* use rostwitter and python_twoauth
* Contributors: Kei Okada, Ryohei Ueda, Yusuke Furuta

0.0.1 (2014-12-25)
------------------
* Restarting kinect paranoiac
  1) usb reset
  2) kill nodelet manager
  3) kill child processing
  4) restart openni.launch (hardcoded!)
* Add rviz_mouse_point_to_tablet.py to pr2.launch
* Use larger value to detect gound object by PR2 to avoid small noises
* Add sound when launching pr2.launch
* kill nodelet manager and processes rather than killing openni/driver
* Say something at the end of pr2.launch
* Use low framerate for gripper sensors to avoid high load
* move twitter related program to robot_common from jsk_pr2_startup
* modify launch file for gazebo
* add yaml file for gazebo
* delete LaserScanIntensityFilter
* modify sensors_kinect and add sensors
* move pr2 related package under jsk_pr2_robot
* Contributors: Ryohei Ueda, Yuto Inagaki, Yusuke Furuta
