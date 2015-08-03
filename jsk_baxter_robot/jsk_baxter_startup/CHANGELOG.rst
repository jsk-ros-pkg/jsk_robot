^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_baxter_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2015-08-03)
------------------
* [jsk_robot] use common database jsk_robot_lifelog, with identify with collection name ROBOT_NAME
* remove old rosmake related files
* Contributors: Yuki Furuta, Kei Okada

0.0.8 (2015-07-16)
------------------

0.0.7 (2015-06-11)
------------------
* [jsk_baxter_startup]add mongodb_launch option
* [jsk_baxter_startup] remove face_recognition from baxter.launch
* [jsk_baxter_startup] remove clear_params from joint actionservers in baxter.launch
* [jsk_baxter_robot] add rossetbaxter env-hooks
* Contributors: Yuto Inagaki

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------
* [jsk_baxter_startup/baxter.launch] head_trajectory_action is available after v1.1.0
* [jsk_baxter_sensors] add kinecct2 use_machine parameter
* [jsk_baxter_startup] update to add position diff paramter for tweet
* [jsk_baxter_startup] update rviz config
* [jsk_baxter_startup] add head trajectory server for baxter.launch
* [jsk_baxter_startup] modify to prevent baxter.launch fail
* [jsk_baxter_startup] add more dependencies to jsk_baxter_startup
* [jsk_baxter_startup] shift to use kinect2 from kinect
* [jsk_baxter_startup] remove checkerboard yaml rosparam
* [jsk_baxter_startup] add use_color and keep_organized option to baxter self_filter.launch
* [jsk_baxter_startup] add self_filter launch and config to jsk_baxter
* Contributors: Kei Okada, Yuto Inagaki

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------
* add install commands to cmake
* remove jtalk voice
* Contributors: Kei Okada, Yuto Inagaki

0.0.1 (2014-12-25)
------------------
* fix typo in baxter_tweet
* add time singal in baxter startup
* move twitter related program to robot_common from jsk_pr2_startup
* repair mongodb.launch and add param
* add gripper action server
* add camera info fixer launch in baxter.launch
* use face_recognition package(procrob_functional)
* add camera_info_fixer, camera_info_std publishes with original param and roi, and camera_info publishes cropped image with same roi, it seems something wrong...
* remove unnecessary components
* add wrench publisher
* add depends
* add image saver
* add sound tools and eus speak-en
* modify params
* modify package name
* add baxter tweet
* mv catkin.cmake to CMakeLists.txt
* fix jsk_baxter_startup/package.xml
* remove baxter_interface and baxter_tools from find_package, they do not need to load as COMPONENTS
* remove unneeded lines
* delete objectdetection_tf_publisher and use checker_board_detector's
* add baxter_description
* update kinect.launch
* delete files correctly
* delete voice directory and move file
* delete text2wave and modify voice_echo.l
* Update jsk_baxter_startup
  We added files in jsk_baxter_sensors
  - for kinect.launch
  - add voice set
  - change joy device name
* add baxter joy dir and launch
* add baxter rviz config file for default baxter nodes
* update manifest
* add tmp groovy manifest file
* one more openni => openni_launch space
* update and add catkin.cmake (just rename CMakeLists.txt to catkin.cmake)
* add baxter startup launch file
* Contributors: Kei Okada, Tomoya Yoshizawa, Yuto Inagaki
