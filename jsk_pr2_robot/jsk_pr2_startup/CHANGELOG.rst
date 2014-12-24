^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pr2_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
