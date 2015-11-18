^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_robot_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2015-11-19)
------------------
* [jsk_robot_startup] Fix namespace of param for pointcloud_to_laserscan
* Contributors: Eisoku Kuroiwa

1.0.0 (2015-11-06)
------------------

0.0.13 (2015-11-06)
-------------------
* [jsk_robot_startup] Add scripts to caclulate odometry with particle filter to integrate odometries (from pattern generator or visual odometry etc) and imu
* [jsk_robot_startup] Add script to set offset from a frame (like init_odom) to odometry source
* Contributors: Iori Kumagai

0.0.12 (2015-11-06)
-------------------
* [jsk_robot_startup/lifelog/mongodb.launch] use machine attribute for mongodb server/client ref: https://github.com/strands-project/mongodb_store/pull/151
* [jsk_robot_startup] Modify pose difference threshould from sigma to 3*sigma
* [jsk_robot_startup] Rename twist_proportional_covariance to twist_proportional_sigma for accuracy
* [jsk_robot_startup] Add twist proportional sigma option to odometry feedback wrapper
* [db_client] add machine option for mongodb client
* [jsk_robot_startup] Fix timestamp problem of transform and odom in feedback process
* [jsk_robot_startup] use deepcopy instead of copy because coipy method copies reference of object members
* [jsk_robot_startup] Reset odometry buffer when initialize_odometry
* [jsk_robot_startup] Remove unnecessary lock in initialize
* [jsk_robot_startup] Prevent dead lock in initialize_odometry
* [jsk_robot_startup] Initialize odometry using odom_init_frame in tf instead of init_odom topic
* [jsk_robot_startup] Add init_signal subscriber to catch contact signal to ground and reset odometry wrapper
* [jsk_robot_startup] Revert calculation of orientation, which is probably deleted by mistake
* [jsk_robot_startup] Modify parameters for real robot
* [jsk_robot_startup] Fix description of integration
* [jsk_robot_startup] Modify integration method from rectangular to trapezoidal, and add prev_global_twist as argument of update_pose
* [jsk_robot_startup] Extend queue_size from 1 to 100
* [jsk_robot_startup] Modify ref_frame_change_method parameter from 0 to 1 to prevent drift in viso
* [jsk_robot_startup] Add init_odom to indicate initialize soruce of odom
* [jsk_robot_startup] Update documents for ConstantHeightFramePublisher
* [jsk_robot_startup] Add arguments to select odom frame name of ConstantHeightFramePublisher
* [jsk_robot_startup] Fix typo in error warning
* [jsk_robot_startup] Print warning when faield to solve tf
* [jsk_robot_startup] Pass odom frame name as rosparam in ConstantHeightFramePublisher
* [jsk_robot_startup] Add script to integrate odometry soruce
* [jsk_robot_startup] Add wrapper script to odometry feedback
* [jsk_robot_startup/lifelog/periodic_replicator_client.py] cancel replication when no wired network connection
* [jsk_robot_startup] Add args to determine frame name of odom and map to gmapping
* [jsk_robot_startup] Add invert_viso_tf option to use invert_tf of viso, which is invert parent and child of viso_odom transformation
* [jsk_robot_startup/lifelog/periodic_replicator_client.py] fix fetching argument
* [jsk_robot_startup] Respawn viso to restart by rosnode kill
* [jsk_robot_startup] Add args to remap image topic name for viso
* [jsk_robot_startup/lifelog/tweet.launch] use image_saver instead of extract_images for tweeting with image
* [jsk_robot_startup] add jenkins/musca to database replication node
* Contributors: Yuki Furuta, Iori Kumagai

0.0.11 (2015-09-01)
-------------------
* [jsk_robot_startup] Add visualization node for viso odom_combined
* [jsk_robot_startup] Add viso.launch for visual odometry
* Contributors: Iori Kumagai

0.0.10 (2015-08-16)
-------------------
* [jsk_robot_startup] fix camera namespace openni -> kinect_head
* [jsk_robot_startup] Add odometry accuracy parameters for gmapping
* [jsk_robot_startup] Add scripts to reset slam and heightmap according to /odom_init_trigger
  topic
* [jsk_robot_startup] Add gmapping.rviz for gmapping.launch
* [jsk_robot_startup] Add delta/particle/minimum_score parameters for gmapping
* [jsk_robot_startup] use param "robot/name"
  [jsk_pr2_startup] use daemon mongod
* [jsk_robot_startup] Add rate param to modify tf publish rate and set 10.0 as defalut
* add run depend for mapping
* [jsk_robot_startup] Enable inf value in pointcloud_to_laserscan to prevent robot from obtaining wrong obstacles
* Contributors: Yuki Furuta, Ryohei Ueda, Yu Ohara, Iori Kumagai

0.0.9 (2015-08-03)
------------------
* [jsk_robot_startup] Modify node name of gmapping and pointcloud_to_laserscan
* [jsk_robot_startup] Add respawn to gmapping
* [jsk_robot_startup] Add angle_max and angle_min arguments to determine horizontal scan range
* [jsk_robot_startup] Fix x, y and yaw of pointcloud_toscan_base to parent, roll and pitch to /odom
* [jsk_robot_startup] Fix roll and pitch angle of cosntant height frame same as /odom
* [jsk_robot_startup] Add gmapping to run_depend
* [jsk_robot_startup] Add scripts and launch files for gmapping
* [jsk_robot_startup] support daemon mode mongod; enable replication to jsk robot-database
* Contributors: Iori Kumagai, Yuki Furuta

0.0.8 (2015-07-16)
------------------

0.0.7 (2015-06-11)
------------------

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------
* [jsk_baxter_startup] update to add position diff paramter for tweet
* [jsk_baxter_startup] modify to prevent baxter.launch fail
* [jsk_robot_startup/package.xml: add diagnostic_msgs, pr2_mechanism_controllers, sensor_msgs to build dependencies
* [sk_robot_startup/CMakeLists.txt] update to set permission for installed script files
* [jsk_robot_startup] modfiy CMakeLists.txt to install jsk_robot_startup correctly
* [jsk_robot_startup/lifelog/active_user.l] repair tweet lifelog
* [jsk_robot_startup/lifelog/mongodb.launch] fix typo of option in launch
* [jsk_robot_startup/lifelog/mongodb.launch: add mongodb launch; mongod kill watcher
* Contributors: Yuki Furuta, Yuto Inagaki

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------

0.0.1 (2014-12-25)
------------------
* check joint state and set movep for odom disable robot
* Add sound when launching pr2.launch
* Say something at the end of pr2.launch
* move twitter related program to robot_common from jsk_pr2_startup
* add ros-info
* robot time signal
* add tweet.l, see jsk_nao_startup.launch for example
* repiar mongodb.launch
* repair mongodb.launch and add param
* add jsk_robot_common/jsk_robot_startup
* Contributors: Kanae Kochigami, Ryohei Ueda, Yuto Inagaki, Yusuke Furuta
