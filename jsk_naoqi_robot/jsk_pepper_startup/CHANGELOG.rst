^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pepper_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* add launch_joy in jsk_pepper_startup launch
* [jsk_naoqi_robot] speak when program is started (by unix:sleep) or terminated (by overriden roseus-sigint-handler)
* [jsk_pepper_startup] remove meta package dependencies
* [jsk_pepper_startup] add launch_twitter arg for jsk_pepper_startup.launch
* Contributors: Kanae Kochigami, Shingo Kitagawa, Yuki Furuta

1.0.6 (2016-06-17)
------------------
* change from naoqi_msgs to naoqi_bridge_msgs (`#614 <https://github.com/jsk-ros-pkg/jsk_robot/issues/614>`_)
* Contributors: Kanae Kochigami

1.0.5 (2016-04-18)
------------------

1.0.4 (2016-03-21)
------------------
* package.xml add naoqi_pose depends to  jsk_pepper_startup
* Contributors: Kei Okada

1.0.3 (2016-03-05)
------------------

1.0.2 (2016-02-14)
------------------
* jsk_pepper_startup: enable test https://github.com/furushchev/jsk_robot/commit/4b39a93b972008f5155e27a201b67a061c527a26
* [jsk_pepper_startup/CMakeLists.txt] disable test for a while
* [jsk_pepper_startup] use roslaunch_add_file_check instead of roslaunch-check in test file and skip hydro
* Contributors: Yuki Furuta, Kei Okada

1.0.1 (2015-11-19)
------------------
* move jsk_pepper_startup under jsk_naoqi_robot

1.0.0 (2015-11-06)
------------------
* [launch/jsk_pepper_startup.launch]  use teleop_twist_joy instead of turtlebot_telop
* [package.xml] add depend to naoqi_bridge and naoqi_dashboard
* [joy-client.l] use naoqi_bridge_msgs instead of naoqi_msgs, update to pepper_robot/pose namespace
* [launch/jsk_pepper_startup.launch] remove old nao_app launch files
* fix to include pepper_full.launch for pepper_bringup > 0.1.4 (#452 <https://github.com/jsk-ros-pkg/jsk_robot/issues/452>)
* Contributors: Kei Okada

0.0.13 (2015-11-06)
-------------------

0.0.12 (2015-11-06)
-------------------

0.0.11 (2015-09-01)
-------------------
* set robot/type, robot/name
* Contributors: Kei Okada

0.0.10 (2015-08-16)
-------------------

0.0.9 (2015-08-03)
------------------
* package.xml: add joy
* [jsk_pepper_startup] add test_code to check if pepper launch is valid
* Contributors: Hitoshi Kamada, Kei Okada

0.0.8 (2015-07-16)
------------------
* use front/camera until https://github.com/ros-naoqi/pepper_robot/pull/1/files is merged
* Contributors: Kanae Kochigami

0.0.7 (2015-06-11)
------------------
* [package.xml] add depends to nao_apps
* [jsk_pepper_startup] add joy-client.l
* Contributors: Kanae Kochigami, Kei Okada

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------
* modify msg name and launch file name
* Contributors: Jiang Jun

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------
* add install commands to cmake
* Contributors: Kei Okada

0.0.1 (2014-12-25)
------------------
* add depends to pepper_bringup
* fix launch file as of Dec 14
* use jsk_pepper_bringup and now naoqi repos
* add more depends
* tweet when imu is learge
* deleted displaying installed behaviors (only it was test)
* conversation added to face recognition
* remove nao_driver from depends
* add comment to how to modify voices
* add learn face example
* check timestamp to publish images
* add nao_interaction_msgs
* update sample, without face recognition
* use all cameras (top/bottom/depeth)
* use key for recognize-word
* use naoqi_sensors
* add simple demo code
* add nao_dashboard
* add sample/sample.l
* fix package.xml
* move tweet.l under nodes directory, listen /pepper_tweet
* some voice added
* some bugs fixed
* pepper speaking function added
* add jsk_peper_robot (add CMakeLists.txt launch/jsk_pepper_startup.launch package.xml tweet.l)
* Contributors: Kanae Kochigami, Kei Okada
