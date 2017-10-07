^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_robot_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------
* Enable safe teleop for fetch (`#801 <https://github.com/jsk-ros-pkg/jsk_robot/issues/801>`_ )
  * [jsk_fetch_startup] move unsafe_warning.l to jsk_robot_startup / enable unsafe_warning on fetch
  * [jsk_pr2_startup] move mux_selector.py to jsk_robot_startup

* run active_user.l with 1hz (`#787 <https://github.com/jsk-ros-pkg/jsk_robot/issues/787>`_ )
* [jsk_robot_startup, multisense_local.launch] add USE_HEIGHTMAP parameter (`#783 <https://github.com/jsk-ros-pkg/jsk_robot/issues/783>`_ )
* [jsk_robot_startup] add record launch files for SLAM (`#760 <https://github.com/jsk-ros-pkg/jsk_robot/issues/760>`_)
* [jsk_robot_startup][mongodb.launch] pass through arg test_mode (`#755 <https://github.com/jsk-ros-pkg/jsk_robot/issues/755>`_)
* [jsk_robot_startup] Tuned odometry params for jaxon (`#732 <https://github.com/jsk-ros-pkg/jsk_robot/issues/732>`_)
* Contributors: Kei Okada, Yohei Kakiuchi, Yuki Furuta, Iori Kumagai

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* Merge pull request `#672 <https://github.com/jsk-ros-pkg/jsk_robot/issues/672>`_ from furushchev/move-logapp
  [jsk_robot_lifelog] move logging program from jsk_pr2_startup/jsk_pr2_lifelog to jsk_robot_startup/lifelog
* [jsk_robot_startup] Fix imu_rootlink calculation bug
* [jsk_naoqi_robot] speak when program is started (by unix:sleep) or terminated (by overriden roseus-sigint-handler)
* [jsk_robot_startup] move initialpose_publisher.l from jsk_pr2_startup/jsk_pr2_move_base
* [jsk_robot_lifelog] move logging program from jsk_pr2_startup/jsk_pr2_lifelog to jsk_robot_startup/lifelog
* removed unnecessary packages from find_package in CMakeLists.txt
* [jsk_robot_startup] Rewrite ImuRootlinkCalculator using tf api functions to remove transformation bag
* [jsk_robot_startup] Add imu topic argument to localization launch files
* [jsk_robot_startup] Preserve compatibility in heightmap accumulation when USE_BIPED_LOCALIZATION is false
* Fix slam pointcloud accumulation (`#649 <https://github.com/jsk-ros-pkg/jsk_robot/issues/649>`_)
  * [jsk_robot_startup] Publish slam_odom continuously
  * [jsk_robot_startup] Enable use_slam_feedback option
  * [jsk_robot_startup] Fix slam_laser_nodelets name and use slam based pointcloud for heightmap
  * [jsk_robot_startup] Use map as assemble_base_frame
* [jsk_robot_startup] Use standard normal distribution in yaw error weights
* [jsk_robot_startup] Use standard normal distribution for error weighting function to prevent too large weights when small sigma is given
* Apply ekf update in source odom callback (`#643 <https://github.com/jsk-ros-pkg/jsk_robot/issues/643>`_)
  * [jsk_robot_startup] Publish diagnostics for debug when publish odometry
  * [jsk_robot_startup] Add node name to diagnostics status
  * [jsk_robot_startup] Remap diagnostics topic to global
  * [jsk_robot_startup] Apply ekf_update in source_odom_callback and update twist and header in ekf_update
* Publish diagnostics in odometry (`#642 <https://github.com/jsk-ros-pkg/jsk_robot/issues/642>`_)
  * [jsk_robot_startup] Publish diagnostics for debug when publish odometry
  * [jsk_robot_startup] Add node name to diagnostics status
  * [jsk_robot_startup] Remap diagnostics topic to global
* [jsk_robot_startup] Make cov smaller when robot is stopping to trust stopping state (`#641 <https://github.com/jsk-ros-pkg/jsk_robot/issues/641>`_)
* [jsk_robot_startup] Add launch files for rtabmap (`#639 <https://github.com/jsk-ros-pkg/jsk_robot/issues/639>`_)
  * [jsk_robot_startup] Add launch files for rtabmap
  * [jsk_robot_startup] Add respawn option to rtabmap
* [jsk_robot_common/jsk_robot_startup/util/start_launch_sound.py] PLAY_START -> PLAY_ONCE (`#631 <https://github.com/jsk-ros-pkg/jsk_robot/issues/631>`_)
* [jsk_robot_startup] Fix transformation from odom to map in slam_odom topic
* Prevent particle degeneration (`#629 <https://github.com/jsk-ros-pkg/jsk_robot/issues/629>`_)
  * [jsk_robot_startup] Run ekf_update only when dt > 0 (= source_odom is updated)
  * [jsk_robot_startup] Move GPF process to measuremnt callback and add min_sampling_rate option
  * [jsk_robot_startup] Tune JAXON_RED odometry params to prevent particle degeneration
  * [jsk_robot_startup] Run sampling every time when min_sampling_rate is not larger than 0
  * [jsk_robot_startup] Tune min_sampling_rate for JAXON_RED
* [jsk_robot_startup] Fix oodm_frame of biped_odom_particle when use_odometry_iir_filter is false (`#626 <https://github.com/jsk-ros-pkg/jsk_robot/issues/626>`_)
* [jsk_robot_startup/multisense_local.launch] Add biped_localization flag
* Contributors: Yuki Furuta, Iori Kumagai, Kanae Kochigami, Kei Okada, Kohei Kimura, Masahiro Bando, leus, Satoshi Otsubo

1.0.6 (2016-06-17)
------------------
* [jsk_robot_startup] Define publisher before callback definition (`#615 <https://github.com/jsk-ros-pkg/jsk_robot/issues/615>`_)
* Combine EKF and GPF for more constant odometry output (`#617 <https://github.com/jsk-ros-pkg/jsk_robot/issues/617>`_)
  * [jsk_robot_startup] Add script for odometry calculation with combination of EKF and GPF
  * [jsk_robot_startup] Fix some misses
  * [jsk_robot_startup] Use ekf+gpf odometry instead of particle odometry and thus iir filter is no longer used
  * [jsk_robot_startup] Tune jaxon_red odometry param
* [jsk_robot_startup] add multisense_local.launch
* Use only important partilces when estimating output distribution of particle odom  (`#601 <https://github.com/jsk-ros-pkg/jsk_robot/issues/601>`_)
  * [jsk_robot_startup] Use only important particles to guess output distribution
  * [jsk_robot_startup] Rename parameter name for valid particles
  * [jsk_robot_startup] Use 1/3 particles for estimate normal distribution and make initial distribution narrower
  * [jsk_robot_startup] Rename value name to match with rosparam name
  * [jsk_robot_startup] Add comments
* activate_user.l : fix for non pr2 robot, see `#589 <https://github.com/jsk-ros-pkg/jsk_robot/issues/589>`_ (`#590 <https://github.com/jsk-ros-pkg/jsk_robot/issues/590>`_)
* [jsk_robot_startup] Add jsk_recognition_msgs to dependency because of histgram msgs
* [jsk_robot_startup] Fix typo
* [jsk_robot_startup] Add publish_histogram option to visualize particles
* [jsk_robot_startup] Use parameter_yaml arg instead of ROBOT environmental value for odometry params
* [jsk_robot_startup] Tune z direction parameter for JAXON_RED odometry
* Contributors: Iori Kumagai, Kei Okada, Yohei Kakiuchi

1.0.5 (2016-04-18)
------------------
* [jsk_robot_startup] Use robot_center_pointcloud_bbox_clipped as input for octomap server
* Contributors: leus

1.0.4 (2016-03-21)
------------------
* [jsk_robot_startup] Overwrite pdf threshould was too small: 1e-6->1e-3
* Contributors: Iori Kumagai

1.0.3 (2016-03-05)
------------------
* jsk_robot_startup/lifelog: fix `#259 <https://github.com/jsk-ros-pkg/jsk_robot/issues/259>`_ rename mongod_watcher.py -> mongod_kill_watcher.py
* [jsk_robot_startup] Tune cutoff freq for iir filter in JAXON_RED
* [jsk_robot_startup] Adjust filter rate to 40Hz, which is same as raw particle_odometry rate
* [jsk_robot_startup] Rename node of IIRFilter
* [jsk_robot_startup] Fix some bags in IIRFilter
* [jsk_robot_startup] Apply iir filter to particle odometry to improve stability of map
* Contributors: Kei Okada, Iori Kumagai

1.0.2 (2016-02-14)
------------------
* [jsk_robot_startup] Make distribution more larger in x and y in JAXON_RED
* [jsk_robot_startup] Use prev_rpy to prevent orientation jump at around 180[deg]. This approach may not work at singular points because it only consider arctan/arcsin uncertainty
* [jsk_robot_startup] theta + pi is not same pose as theta
* [jsk_robot_startup] Consider previous euler angle in transform_quaternion_to_euler, but this solution is temporal
* [jsk_robot_startup] Automatically clear octomap when robot stands on the ground
* [jsk_robot_startup] Add octomap launch using slam
  Added:
  - jsk_robot_common/jsk_robot_startup/launch/slam_octomap.launch
* [jsk_robot_startup] Update yaw rotation sigma for ignoring stopping state model
* [jsk_robot_startup] Remove / from tf frame name which is not supported by tf2
* [jsk_robot_startup] Make distribution larger to make estimation more robust to pose
* [jsk_robot_startup] Also fix old version odometry param definitions in defaut_odometry_params.yaml
* [jsk_robot_startup] Tune HRP2JSKNT odometry params
* [jsk_robot_startup] Fix default and min/max values in OdometryOffsetReconfigure
* [jsk_robot_startup] Fix odometry param definitions for HRP2JSKNT which was a old version
* [jsk_robot_startup] Use odom_init_transform from footcoords instead of HrpsysSeqStateROSBridge
* [jsk_robot_startup] Fix frame_id of imu_rootlink to base_link
* [jsk_robot_startup] Add ImuRootlinkCalculator, calculate base_link relative imu orientation for ParticleOdometry to be compatible with HrpsysSeqStateROSBridge
* [jsk_robot_startup] It seems that transformations in tfMessage needs to be sorted by timestamp
* [jsk_robot_startup] Use original publisher for tfMessage because tf.broadcaster cannot receive transformation msg list
* [jsk_robot_startup] Put broadcast_tf function together in execute function to reduce tf rate
* [jsk_robot_startup] Configure odometry parameters for JAXON_RED
* [jsk_robot_startup] Publish slam_odom topic because transformations of odometry topics are already separated into /localization/tf
* [jsk_robot_startup] Fix bag in stop condicion check when overwrite pdf
* [jsk_robot_startup] Output base_link->pointcloud_scan transform to /tf for simualted scan
* [jsk_robot_startup] Remap /tf to /tf_null to reduce unnecessary tf
* [jsk_robot_startup] Fix topic name bugs and remap bags
* [jsk_robot_startup] Separate odometry transform and make only one broadcaster
* [jsk_robot_startup] Use odom_init_transform from HrpsysSeqStateROSBridge
* [jsk_robot_startup] Use imu_rootlink, base_link_frame relative imu orientation
* [jsk_robot_startup] Use calculate_init_to_base_link_transform as initial transform of odometry_offset and particle odometry
* [jsk_robot_startup] Add node to calculate odom_init->base_link transform using odom->base_link and odom->odom_init topics without tf
* [jsk_robot_startup] publish slam_odom only when use_slam_feedback is true
* [jsk_robot_startup] Fix launch remaps and params for new offset calculation
* [jsk_robot_startup] Remove tf listener and use odometry and transformation topics in offset calculation
* [jsk_robot_startup] Remove unnecessary groups in biped_localization.launch
* [jsk_robot_startup] Remove transform listener in feedback wrapper which is no longer needed
* [jsk_robot_startup] forgot import broadcast_transform in ParticleOdometry
* [jsk_robot_startup] Twist proportional sigma option should be processed by individual class, not common utils
* [jsk_robot_startup] Remove twist_proportional_sigma from OdometryFeedbackWrapper
* [jsk_robot_startup] Trust stopping status when mean offset is accumulated to twist in OdometryOffset
* [jsk_robot_startup] Fix import bug of CameraToBaseOffset
* [jsk_robot_startup] Set default publish_tf as False in unnecessary tfs and do not make broadcast when publish_tf is false
* [jsk_robot_startup] Update default odometry paremeter set to overwrite viso covariance in OdometryOffset
* [jsk_robot_startup] Use common odometry utilities in ParticleOdometry
* [jsk_robot_startup] Put odometry calculation together in OdometryOffset and OdometryFeedbackWrapper is only calculate feedback
* [jsk_robot_startup] Fix bags related to feedback wrapper and odoemtry utils
* [jsk_robot_startup] Remove lookup transforms using odometry topic information
* [jsk_robot_startup] Separate commonly used utilities for odometry calculation
* [jsk_robot_startup] Remove use_imu option from launch files and describe in config file
* [jsk_robot_startup] Fix calculation for initial offset of viso camera offset to reduce linalg.inv
* [jsk_robot_startup] Fix calculation for imu rotation and modify base coordinate from base_link to odom
* [jsk_robot_startup] Calculate imu rotation when imu coordinate is not same as global
* [jsk_robot_startup] Initialize imu buffer in __init_\_ for ParticleOdometry
* [jsk_robot_startup] Tune odometry parameters for JAXON using calculate_covariance option
* [jsk_robot_startup] Trust stop state in covariance calculation in OdometryOffset when twist_proportional_sigma is false
* [jsk_robot_startup] Preserve odometry information when calculate_covarinace is True
* [jsk_robot_startup] Add options to overwrite covariance in odometry_offset
* [jsk_robot_startup] Adjust timestamp for viso offset calculation in camera_to_base_offset
* [jsk_robot_startup] Fix camera offset calculation
* [jsk_robot_startup] Add offset script to compensate camera motion relative to base_link in viso
* [jsk_robot_startup] Add jaxon odometry parameter files
* [jsk_robot_startup/lifelog/mongodb_local.launch] add launch file for local mongodb
* [jsk_robot_startup] Tune filter and viso parameters for HRP2JSKNT
* [jsk_robot_startup] Enable twist filter in HRP2JSKNT
* [jsk_robot_startup] fix fogotten Vector3 import
* [jsk_robot_startup] Remove source_skip_dt of ParticleOdometry and implement median filter in OdometryOffset
* [jsk_robot_startup] Add source_skip_dt for HRP2JSKNT
* [jsk_robot_startup] Tune robot specific params for HRP2JSKNT
* [jsk_robot_startup] Separate parameter config file from launch to tune robot specific params
* [jsk_robot_startup] Pass soruce_odom without dt check when source_odom is not initialized
* [jsk_robot_startup] Add source_skip_dt to detect and skip stacked odometry
* [jsk_robot_startup] Update twist covariance in calculate odometry for feedback wrapper
* {jsk_pr2_robot, jsk_robot_startup}/README.md: fix section/subsection
* [jsk_robot_startup] Rewrite weighted gaussian covariance estimation using numpy to speed up
* README.md: fix section/subsection
* [jsk_robot_startup] numpy was more efficient in average and covairance calculation, but weighted cov is supported from numpy 1.10
* [jsk_robot_startup] Calculate weighted mean and covariance directly, not through numpy
* [jsk_robot_startup] Calculate inverse matrix for norm_pdf_multivariate before weighting
* Merge branch 'speed-up-particle-odometry' of http://github.com/orikuma/jsk_robot into speed-up-particle-odometry
* [jsk_robot_startup] Replace tf.transformations.euler_from_quaternion to transform_quaternion_to_euler
* [jsk_robot_startup] sampling number of multivariate_normal should be integer
* [jsk_robot_startup] Call multivariate_normal once in sampling
* [jsk_robot_startup] stereo_namespace is no longer used in particle_odometry because viso is separated
* [jsk_robot_startup] Pass update when global twist cannot be calcluated because of tf problem
* [jsk_robot_startup] Separate viso from particle_odometry.launch
* [jsk_robot_startup] Modify constant height for slam through rqt_reconfigure
* [jsk_robot_startup] Add height options for slam_laser_scan
* [jsk_robot_startup] Fix forgotten subst_value in rosparam of slam_odom_scan_distance_filtered
* [jsk_robot_startup] Add stereo_namespace for viso to set multisense prefix
* [jsk_robot_startup] Separate laser nodelets for slam to reuse in other system
* Merge pull request `#490 <https://github.com/jsk-ros-pkg/jsk_robot/issues/490>`_ from orikuma/closed-loop-slam-odom-system
  [jsk_robot_startup] Add launch file to launch full SLAM and odometry system for biped robot
* [jsk_robot_startup] Add option to toggle setting multisense_laser options and using slam feedback
* [jsk_robot_startup] Add use_salm_feedback option to particle_odometry.launch to select standalone odometry or slam combination
* [jsk_robot_startup] Add full launch file for localization, which has integrate slam laser pointcloud parameters from multisense_local.launch of robots
* [jsk_robot_startup] Fix source_odom of viso feedbackwrapper: viso_odom->viso_odom_offset and make update rate from 50 to 100 instead of particles 50 to 20.
* [jsk_robot_startup] Add options for gmapping: iterations, lsigma, temporal_update and map_update_interval. defaults are same as gmapping default.
* [jsk_robot_startup] Add range_max option to determine simulated laser_scan range
* [jsk_robot_startup] Move viso_gaussian_point_cloud to use_ekf block
* [jsk_robot_startup] Broadcast /biped_odom_particle as parent of init_odom
* [jsk_robot_startup] Add OdomDiffTransformPublisher to broadcast tf as difference of target and intermediate frame
* [jsk_robot_startup] Enable map infromation feedback and modify some parameters for particle odometry
* [jsk_robot_startup] Add script to convert map information from slam to odometry msg
* [jsk_robot_startup] Time feedback is prevented when max_feedback_time <= 0
* [jsk_robot_startup] Normalize quaternion and fix matrix for quaternion integration
* [jsk_robot_startup] Use direct diviasion as same as particle odometry in odometry feedback wrapper
* [jsk_robot_startup] Use quaternion diviasion directly instead of using euler angle
* [jsk_robot_startup] Add some comments
* [jsk_robot_startup] Add odometry_offset to odometry_integration.launch
* [jsk_robot_startup] Update rate of particle odometries
* [jsk_robot_startup] Add queue_size option
* [jsk_robot_startup] Fix parameters for particle odometry
* [jsk_robot_startup] Add distribution_feedback_minimum_sigma, limit minimum sigma for check distribution error and do not execute feedback when feedback_odom has too small distribution
* [jsk_robot_startup] Fix offset calculation: wrong multipling homogeneous matrix order
* [jsk_robot_startup] Add use_imu_yaw option
* [jsk_robot_startup] Add comment
* [jsk_robot_startup] delegate offset calculation to OdometryOffset.py
* [jsk_robot_startup] Calculate transformation instead of integrate velocity in feedback wrapper
* [jsk_robot_startup] Use odometry feedback to prevent drift of viso
* [jsk_robot_startup] Integrate odometry when odometry feedback is enabled
* [jsk_robot_startup] Resume trapezoidal odometry integration and add init_sigma param
* Contributors: Yuki Furuta, Kei Okada, Kohei Kimura, Ryohei Ueda, Iori Kumagai

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
