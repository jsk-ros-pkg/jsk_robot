^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package baxtereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
