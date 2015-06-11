^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package baxtereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
