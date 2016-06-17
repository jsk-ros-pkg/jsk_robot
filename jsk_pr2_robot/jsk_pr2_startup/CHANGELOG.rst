^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pr2_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2016-06-17)
------------------
* fix too long file name in deb build (`#618 <https://github.com/jsk-ros-pkg/jsk_robot/issues/618>`_)
  * fix too long file name in deb build
  * update maintainer names
* Contributors: Kei Okada

1.0.5 (2016-04-18)
------------------
* [jsk_pr2_startup/CMakeLists.txt] install launch files related gazebo
* Merge pull request `#577 <https://github.com/jsk-ros-pkg/jsk_robot/issues/577>`_ from mmurooka/add-run-dependency
  [jsk_pr2_startup/package.xml] add run_dependency.
* [jsk_pr2_startup/package.xml] add run_dependency.
* [jsk_pr2_startup/jsk_pr2.rosinstall] update jsk_robot to 1.0.2
  for applying the fix in `#539 <https://github.com/jsk-ros-pkg/jsk_robot/issues/539>`_
* Contributors: Yuki Furuta, Kei Okada, Masaki Murooka

1.0.4 (2016-03-21)
------------------

1.0.3 (2016-03-05)
------------------

1.0.2 (2016-02-14)
------------------
* [jsk_pr2_startup/jsk_pr2_lifelog/objectdetection_db.py] unsubscribe objectdetection topics which has no other subscribers
* [jsk_pr2_startup/jsk_pr2_move_base/costmap_common_params.yaml] fix: initial inflation range
* [jsk_pr2_startup/jsk_pr2_lifelog] add limit for querying lifelog related documents
* [jsk_pr2_startup/package.xml] add eusurdf to run_depend
  ref: https://github.com/jsk-ros-pkg/jsk_robot/issues/512
* [jsk_pr2_startup/jsk_pr2_sensors/check_openni_node.py] refactor check_openni_node.py
* [jsk_pr2_startup/jsk_pr2_move_base] remove nice +10 of move_base_node
* [jsk_pr2_startup/jsk_pr2_move_base] increase update frequency to 10.0 Hz
* [jsk_pr2_startup/pr2_gazebo.launch] change gripper stall_velocity_threshold param to avoid stuck on move-gripper :wait t
* [jsk_pr2_startup/pr2_gazebo.launch] add relay to /kinect_head_c2/depth_registered/points_throttle
* [jsk_pr2_startup/jsk_pr2_move_base/initialpose_publisher.l] add initialpose_publisher.l
* [jsk_pr2_startup/sample/pr2_gazebo_73b2|610.launch] add initial pose args
* [jsk_pr2_startup/pr2_gazebo.launch] fix: disable launch only for real robot; avoid check_cable
* [jsk_pr2_startup/package.xml] add missing run dependencies
* [jsk_pr2_startup/jsk_pr2_move_base] split global_costmap_params for avoiding segfault
* [jsk_pr2_startup] Divide hand / base Tracking in Each Node for cameraman rocon
* Merge branch 'add-rocon' of github.com:aginika/jsk_robot into add-rocon
* add rocon launch/configs
* add rocon launch/configs
* update .rosinstall for pr2 demos
* Contributors: Yuki Furuta, Yuto Inagaki

1.0.1 (2015-11-19)
------------------
* Record battery info before pwer go off `#474 <https://github.com/jsk-ros-pkg/jsk_robot/issues/474>`_ 
* Contributors: Chi Wun Au

1.0.0 (2015-11-06)
------------------

0.0.13 (2015-11-06)
-------------------

0.0.12 (2015-11-06)
-------------------
* support mongodb-based life-log

  * [db_client] add machine option for mongodb client
  * [jsk_pr2_startup/jsk_pr2_lifelog/action_result_db.py] fix: ignore useless type subscription
  * [jsk_pr2_startup/pr2_bringup.launch] use replication dump temp path on /removable instead of /tmp
  * [jsk_pr2_startup] add db client params and map_frame in pr2.launch
  * [jsk_pr2_startup/jsk_pr2_lifelog/db_client.launch] add result of face recognition data for recording to db
  * [jsk_pr2_startup/jsk_pr2_lifelog/db_client.launch] fix excluding topics for db record
  * [jsk_pr2_startup/jsk_pr2_lifelog/objectdetection_db.py] store NOT '_agg' topics instead of '_agg'
    for support applications that publish msgs without "_agg" topic
  * [jsk_pr2_startup/jsk_pr2_lifelog/action_result_db_config.yaml] remove gripper action from black_list
  * [jsk_pr2_startup/jsk_pr2_lifelog/visualization] add visualization nodes for database
  * [jsk_pr2_startup/jsk_pr2_lifelog/mongodb_log.py] fix mongodb_log.py to support logging with jsk name convention
  * [jsk_pr2_startup/jsk_pr2_lifelog/pr2-tweet-log.l] add twitter node for logging

* Sounds

  * [jsk_pr2_startup] remap sound_play nodes to robotsound
  * [jsk_pr2_startup/pr2_hark] fix issue https://github.com/jsk-ros-pkg/jsk_robot/issues/359
    - Update Transfer function data for Hark >2.1
    - Update batch files for new transfer function data
    - support stand-alone launch of `pr2_hark.launch`
    - Update README

* Misc updates

  * [jsk_pr2_startup] Wording in plugin description
  * [jsk_pr2_startup] modify typo in jsk_pr2.rosinstall
  * [jsk_pr2_startup] add some args to pr2.launch
  * [.rosinstall] fix face_recognition to upstream
  * [jsk_pr2_startup] add face_recognition
  * [jsk_pr2_startup] update global costmap params
  * [jsk_pr2_startup] change the order of local costmap plugin
  * [jsk_pr2_startup] add pr2_gazebo_610.launch as sample

* Contributors: Yuki Furuta, Isaac IY Saito, Yuto Inagaki

0.0.11 (2015-09-01)
-------------------

0.0.10 (2015-08-16)
-------------------
* [jsk_pr2_startup] logging images/pointclouds/tf/jointstates/people
* [jsk_pr2_startup] enable logging pr2_gripper_action
* [jsk_pr2_startup] add pr2 heightmap sample launch
* [jsk_pr2_startup/package.xml] add missing deps for pr2
* [jsk_pr2_startup/pr2_gazebo.launch] use relay/republish instead of rgbd_launch for creating rectified images
* [jsk_pr2_startup/package.xml] add social_navigation_layers to run_depends
* [jsk_robot_startup] use param "robot/name"
  [jsk_pr2_startup] use daemon mongod
* Revert "[jsk_robot] unified database"
* [jsk_pr2_startup/jsk_pr2.rosinstall] add temporal missing package mongodb_store
* Contributors: Yuki Furuta, Yuto Inagaki

0.0.9 (2015-08-03)
------------------
* [jsk_pr2_startup] add 73b2 sample launch file
* [jsk_pr2_startup/people_detection.launch] add people tracker
* [jsk_pr2_startup] add rosinstall for jsk pr2
* [jsk_robot] use common database jsk_robot_lifelog, with identify with collection name ROBOT_NAME
* [jsk_pr2_startup/pr2_bringup.launch] use daemon mode mongod for pr2
* change openni namespace to kinect_head
* [jsk_pr2_startup/pr2_gazebo.launch] add initial pose of pr2 in gazebo
* [jsk_pr2_startup] fix typo in pr2.launch
* Contributors: Yuki Furuta, Yuto Inagaki, Chi Wun Aau, Hitoshi Kamada

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
