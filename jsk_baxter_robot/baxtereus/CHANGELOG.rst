^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package baxtereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------
* baxtereus: test/*.test: time-limit=100000 is too large need to set 300-600 (`#802 <https://github.com/jsk-ros-pkg/jsk_robot/issues/802>`_)
  * test/*.test: time-limit=100000 is too large need to set 300-600

* [baxtereus] Fix run_depend to pr2eus_moveit (`#798 <https://github.com/jsk-ros-pkg/jsk_robot/issues/798>`_)
  * Fix run_depend to pr2eus_moveit

* [baxtereus] support tm :fast in moveit angle-vector methods (`#789 <https://github.com/jsk-ros-pkg/jsk_robot/issues/789>`_)
  * angle-vector-sequence accept tms as list of time
    related to https://github.com/jsk-ros-pkg/jsk_robot/pull/791#pullrequestreview-45324124
  * test tm :fast angle-vector methods and av-sequence
  * update doc for :angle-vecor methods
  * support tm :fast in moveit angle-vector methods

* [baxtereus] add key :robot in baxter-interface :init (`#784 <https://github.com/jsk-ros-pkg/jsk_robot/issues/784>`_)
  * add key :robot in baxter-interface :init

* [baxtereus] Use end-coords-interpolation (`#747 <https://github.com/jsk-ros-pkg/jsk_robot/issues/747>`_)
  * Use end-coords-interpolation in baxter-interface

* [baxtereus][jsk_baxter_startup] add baxter moveit test (`#779 <https://github.com/jsk-ros-pkg/jsk_robot/issues/779>`_)
  * add baxter_sim_controllers as test_depend
  * add gazebo moveit starting time assertion
  * add joint_state_controller as test_depend
  * update cmake and package for baxter moveit test
  * add baxter moveit test

* [baxtereus] set wait key default as t (`#769 <https://github.com/jsk-ros-pkg/jsk_robot/issues/769>`_)
  * set wait key default as t

* [baxtereus] add wait key in :start-grasp (`#768 <https://github.com/jsk-ros-pkg/jsk_robot/issues/768>`_)
* [baxtereus] remove unused baxter-moveit.l (`#764 <https://github.com/jsk-ros-pkg/jsk_robot/issues/764>`_)
  * remove baxter-moveit.l from CMakeLists.txt
  * remove unused baxter-moveit.l, it is replaced in baxter-interface.l

* [baxtereus] fix bug in :command-grasp and :calib-grasp (`#762 <https://github.com/jsk-ros-pkg/jsk_robot/issues/762>`_)
  * pass pos in :calib-grasp
  * pass arm as symbol to :command-grasp

* add ik-prepared-poses in baxter class (`#748 <https://github.com/jsk-ros-pkg/jsk_robot/issues/748>`_)
  * baxter-util.l : :ik-prepared-poses, enable to set nil
  * baxter-util.l : self-collision-check, fix symbol  symbol expected for function argument error on compile
  * define :ik-prepared-poses in baxter-robot.l

* [baxtereus] pass args to angle-vector-motion-plan (`#737 <https://github.com/jsk-ros-pkg/jsk_robot/issues/737>`_)
  * add moveit option in baxter-init
  * pass args to angle-vector-motion-plan in baxtereus

* [baxtereus] execute raw angle-vector methods in case tm is not number (`#734 <https://github.com/jsk-ros-pkg/jsk_robot/issues/734>`_)
  * execute raw angle-vector in case tm is not number
    common case is
    (send *ri* :angle-vector-sequnce avs :fast)

* [baxtereus] add ctype in angle-vector methods for moveit (`#730 <https://github.com/jsk-ros-pkg/jsk_robot/issues/730>`_)
  * use &key instead of &rest in baxter angle-vector
    use &key instead of &rest args in :angle-vector methods in baxtereus and
    refine codes.
  * add ctype in angle-vector-sequence for moveit
  * add ctype in angle-vector for moveit

* [baxtereus] add :angle-vector-sequence for moveit (`#728 <https://github.com/jsk-ros-pkg/jsk_robot/issues/728>`_)
  * fix typo in baxter-util.l
  * add :angle-vector-sequence for moveit
    :angle-vector-sequence -> :angle-vector-sequence-motion-plan

* [baxtereus] add :arms for baxter_moveit_config "both_arms" (`#731 <https://github.com/jsk-ros-pkg/jsk_robot/issues/731>`_)
  * add :arms for baxter_moveit_config "both_arms"

* [baxtereus] Fix bug to pass args in :angle-vector (`#729 <https://github.com/jsk-ros-pkg/jsk_robot/issues/729>`_)
  * fix bug in :angle-vector
    use :move-arm as key and pass other args to other methods
    currently args passing was not proper
  * fix typo in baxter-interface
  * remove tab and use space in baxter-interface
  * refine warning message in baxter-interface

* [baxtereus] override :angle-vector and rename existing methods (`#721 <https://github.com/jsk-ros-pkg/jsk_robot/issues/721>`_)
  * override :angle-vector and rename existing methods
    Before                       After
    :angle-vector-motion-plan -> :angle-vector
    :angle-vector             -> :angle-vector-raw
    :angle-vector-sequence    -> :angle-vector-sequence-raw

* [baxtereus] add moveit init option in baxter-interface (`#719 <https://github.com/jsk-ros-pkg/jsk_robot/issues/719>`_)
  * add moveit config init option in baxter-interface
    this option is needed for customize baxter like jsk_baxter_apc
  * add SRDF description in baxter-interface.l

* [baxtereus] add moveit in baxter-interface (`#716 <https://github.com/jsk-ros-pkg/jsk_robot/issues/716>`_)
  * add moveit in baxter-interface

* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Shun Hasegawa

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* JTA insert current position to the first point, this requries https://github.com/RethinkRobotics/baxter_interface/pull/73 (`#636 <https://github.com/jsk-ros-pkg/jsk_robot/issues/636>`_)
* fix for baxter_interface v1.1.1, which removes limitation on number of points to sent (`#635 <https://github.com/jsk-ros-pkg/jsk_robot/issues/635>`_)
  from v1.1.1 if num_points is 1, it will use current position to the first element of trajectory point https://github.com/RethinkRobotics/baxter_interface/commit/b38ec257fece0711adb260ed6bc161096aa3ecae
* [baxter-interface.l] Wait until all joint states are updated before moving `#627 <https://github.com/jsk-ros-pkg/jsk_robot/issues/627>`_ (`#628 <https://github.com/jsk-ros-pkg/jsk_robot/issues/628>`_)
  * Wait until all joint states are updated before moving
  * baxtereus/CMakeLists.txt: re-write using CATKIN_ENABLE_TESTING
  * baxtereus/test/test-baxter.l: test-baxter-interface only available after indigo
* [baxter-interface.l] fix removing torso joint in :ros-state-callback (`#622 <https://github.com/jsk-ros-pkg/jsk_robot/issues/622>`_)
  * [baxtereus/test/test-baxter.l] add test to check that torso joint is not contained in robot-state.
  * [baxtereus/baxter-interface.l] fix removing torso joint in :ros-state-callback of baxter-interface class.
* Contributors: Kei Okada, Masaki Murooka

1.0.6 (2016-06-17)
------------------
* [baxtereus] make ik-bin test faster (`#604 <https://github.com/jsk-ros-pkg/jsk_robot/issues/604>`_)
  * [baxtereus] ik-bin test fix coords pos
  * [baxtereus] make ik-bin test faster
* [baxtereus] Compute IK from prepared poses (using :ik-prepared-poses methods) (`#602 <https://github.com/jsk-ros-pkg/jsk_robot/issues/602>`_)
  * Refactor: Remove no need variable
  * Compute IK from prepared poses (using :ik-prepared-poses methods)
  * Documentation for :inverse-kinematics in baxter-util.l
* add ik-bin-test for apc
* Contributors: Kentaro Wada, Shingo Kitagawa

1.0.5 (2016-04-18)
------------------

1.0.4 (2016-03-21)
------------------
* baxtereus/baxter-util.l: fix code to revert the original posture, if ik failed
* test/test-baxter.l: add test to check the robot revert to start posture, if ik failed
* Contributors: Kei Okada

1.0.3 (2016-03-05)
------------------

1.0.2 (2016-02-14)
------------------
* [baxtereus] Add roseus in find_package to generate eus message
* Contributors: Kentaro Wada

1.0.1 (2015-11-19)
------------------

1.0.0 (2015-11-06)
------------------

0.0.13 (2015-11-06)
-------------------

0.0.12 (2015-11-06)
-------------------
* [baxtereus] :swap-arm-av -> :l/r-reverse
* [baxtereus] :swap-arm-av method
* [baxtereus] :hard-coded-pose method
* Contributors: Kentaro Wada

0.0.11 (2015-09-01)
-------------------
* [baxtereus/test/test-baxter.l] :debug-view :no-messages output too many messages for travis
* [baxtereus/CMakeLists.txt] forget installing baxter-util.l
* [baxtereus/CMakeLists.txt] install test directory
* Contributors: Kei Okada

0.0.10 (2015-08-16)
-------------------

0.0.9 (2015-08-03)
------------------

0.0.8 (2015-07-16)
------------------
* [package.xml] add roseus pr2eus to baxtereus/package.xml, (https://github.com/start-jsk/2014-semi/issues/816)

0.0.7 (2015-06-11)
------------------
* [baxtereus/test/test-baxter.l] add test code for baxter-interface
* [baxtereus] overwrite ros-state-callback in baxter-interface.l for suppress torso warning
* [baxter.l] expand joint limit to refrect precice robot model
* [baxter-interface.l] fix wrong joint name in head_controller head-neck-y -> head_pan
* [baxter-util.l] set default avoid-collision-distance from 200 to 5 for baxter
* [test/test-baxter.l] add ik test, see https://github.com/start-jsk/2014-semi/pull/411
* [test/test-baxgter.test] extend time-limit to 500
* [baxter-interface.l] :angle-vector-sequence use default if nil ctype was passed
* [baxtereus] add arm option for baxter-init
* [baxtereus] fix baxter-interface :init args passing
* [baxtereus] overwrite baxter max joint velocity
* [test/test-baxter.{l,test}] add test code for baxter model (:self-collision-check)
* [baxter-util.l] use :collision-check-paris to get collision link pair, instaed of combination
* [baxter-util.l] (length args) always retruns non nil, so it never goes to self-collision-check with pairs
* [baxter-interface.l, baxter-util.l] move baxter-robot-safe class definition to baxter-util.l
* [baxtereus] add baxter's custom self check collision
* [baxtereus] add Baxter Safe Interface
* [baxter-util.l] comment out test code
* [CMakeLists.txt] describe which branch is used to generate collada
* [CMakeLists.txt] use SOURCE_PREFIX instead of SOURCE_DIR
* [baxter.l] 1) rotate is wrong, we need , for python list, 2) the order of limb is head,larm,rarm
* [baxtereus/baxter-util.l] add util program for baxter
* Contributors: Kei Okada, Kentaro Wada, Yuto Inagaki

0.0.6 (2015-04-10)
------------------
* [baxter-interface.l] we found that input data must be larget then 3, and add dummy last element works very nice!
* Contributors: Yuto Inagaki

0.0.5 (2015-04-08)
------------------
* [baxter-interface.l] fix typo
* [baxter-interface.l] overwrite :angle-vector-seuqnce for tm = :fast
* [baxter-interface.l] notify this warning is ok
* [baxtereus] add head action client for baxter
* Contributors: Yuto Inagaki

0.0.4 (2015-01-30)
------------------
* currently we do not generate baxter.l from baxter_description on the fly
* [baxtereus] add wait key for stop-grasp in baxter-interface.l
* add groupname for baxter-interface.l

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------
* add install commands to cmake
* add baxter-moveit.l
* Contributors: Kei Okada, Yuto Inagaki

0.0.1 (2014-12-25)
------------------
* fix version number
* add wait time for suction
* get baxter hand type property
* fix baxter endcoords and rotate 90
* add action joint client left_w2 right_w2
* do not disable joint-action-enable if gripper action is not found, gazebo did not provide gripper joint action for now
* add tuck-pose and untuck-pose, thanks to wkentaro, iory
* update baxter.yaml (add wrist yaw, head end-coords) baxter.l
* add baxter nod function (send *ri* :nod)
* update baxtereus to use gripper action server
* add reset-manip-pose
* add baxter eus sample
* add :set-baxter-face interface
* do not generate baxter.l if already exists
* add start-grasp and stop-grasp for baxter
* depent to pr2eus speak.l
* add camera interface
* add sound tools and eus speak-en
* fix end-coords
* add baxter.l since baxter_simple.urdf is not released yet
* add code to use baxter_simple.urdf
* add roseus/preus to rundepend
* fix cmake syntax error
* fix for baxter_description is installed
* add missing depends
* change the reset pose
* add baxter-interface.l, validated with 73B2 baxter
* add depends to collada2eus
* use _simple model for smaller dae/lisp files
* add jsk_baxter_robot
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Tomoya Yoshizawa, Yuto Inagaki, Shintaro Noda
